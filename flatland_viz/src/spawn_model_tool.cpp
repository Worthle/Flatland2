#include <flatland_viz/spawn_model_tool.h>

#include <OgreEntity.h>
#include <OgreException.h>
#include <OgreMaterial.h>
#include <OgrePass.h>
#include <OgrePlane.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>

#include <QMessageBox>

#include <cmath>

#include <Box2D/Box2D.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include <flatland_server/types.h>

#include "flatland_viz/load_model_dialog.h"

namespace flatland_viz {

QString SpawnModelTool::path_to_model_file_;
QString SpawnModelTool::model_name;

SpawnModelTool::SpawnModelTool()
    : moving_model_node_(nullptr), arrow_(nullptr) {
  shortcut_key_ = 'm';
}

SpawnModelTool::~SpawnModelTool() {
  delete arrow_;
  if (moving_model_node_) {
    scene_manager_->destroySceneNode(moving_model_node_);
  }
}

void SpawnModelTool::onInitialize() {
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  client_ = node_->create_client<flatland_msgs::srv::SpawnModel>("spawn_model");

  model_state = m_hidden;

  arrow_ = new rviz_rendering::Arrow(scene_manager_, nullptr, 2.0f, 0.2f, 0.3f,
                                     0.35f);
  arrow_->setColor(0.0f, 0.0f, 1.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);

  Ogre::Quaternion orientation(Ogre::Radian(M_PI), Ogre::Vector3(1, 0, 0));
  arrow_->setOrientation(orientation);

  moving_model_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  moving_model_node_->setVisible(false);

  SetMovingModelColor(Qt::green);
}

void SpawnModelTool::activate() {
  LoadModelDialog *model_dialog = new LoadModelDialog(nullptr, this);
  model_dialog->setModal(true);
  model_dialog->show();
}

void SpawnModelTool::BeginPlacement() {
  model_state = m_dragging;
  if (moving_model_node_) {
    moving_model_node_->setVisible(true);
  }
}

void SpawnModelTool::deactivate() {
  if (moving_model_node_) {
    moving_model_node_->setVisible(false);
  }
}

void SpawnModelTool::SpawnModelInFlatland() {
  // model names can not have an embedded period char
  model_name = model_name.replace(".", "_", Qt::CaseSensitive);

  auto request = std::make_shared<flatland_msgs::srv::SpawnModel::Request>();
  request->name = model_name.toStdString();
  request->ns = model_name.toStdString();
  request->yaml_path = path_to_model_file_.toStdString();
  request->pose.x = intersection[0];
  request->pose.y = intersection[1];
  request->pose.theta = initial_angle;

  if (!client_->service_is_ready()) {
    QMessageBox msgBox;
    msgBox.setText("Error: flatland spawn_model service is not available.");
    msgBox.exec();
    return;
  }
  client_->async_send_request(request);
}

void SpawnModelTool::SetMovingModelColor(QColor c) {
  try {
    Ogre::Entity *m_pEntity =
        static_cast<Ogre::Entity *>(moving_model_node_->getAttachedObject(0));
    const Ogre::MaterialPtr m_pMat = m_pEntity->getSubEntity(0)->getMaterial();
    m_pMat->getTechnique(0)->getPass(0)->setAmbient(1, 0, 0);
    m_pMat->getTechnique(0)->getPass(0)->setDiffuse(c.redF(), c.greenF(),
                                                    c.blueF(), 0);
  } catch (const Ogre::Exception &e) {
    // No attached entity (preview uses billboard lines) - nothing to color.
  }
}

int SpawnModelTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
  if (!moving_model_node_) {
    return Render;
  }

  auto projection_finder =
      std::make_shared<rviz_rendering::ViewportProjectionFinder>();

  if (model_state == m_dragging) {
    auto projection = projection_finder->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
    if (projection.first) {
      intersection = projection.second;
      moving_model_node_->setVisible(true);
      moving_model_node_->setPosition(intersection);

      if (event.leftDown()) {
        model_state = m_rotating;
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setPosition(intersection);
        return Render;
      }
    } else {
      moving_model_node_->setVisible(false);
    }
  }

  if (model_state == m_rotating) {
    auto projection = projection_finder->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
    if (projection.first) {
      Ogre::Vector3 intersection2 = projection.second;
      if (event.leftDown()) {
        model_state = m_hidden;
        arrow_->getSceneNode()->setVisible(false);
        intersection[2] = initial_angle;
        SpawnModelInFlatland();
        return Render | Finished;
      }
      moving_model_node_->setVisible(true);
      moving_model_node_->setPosition(intersection);
      Ogre::Vector3 dir = intersection2 - intersection;
      initial_angle = atan2(dir.y, dir.x);
      Ogre::Quaternion orientation(Ogre::Radian(initial_angle),
                                   Ogre::Vector3(0, 0, 1));
      moving_model_node_->setOrientation(orientation);
    }
  }
  return Render;
}

void SpawnModelTool::LoadPreview() {
  moving_model_node_->removeAllChildren();
  lines_list_.clear();

  flatland_server::YamlReader reader(path_to_model_file_.toStdString());

  try {
    flatland_server::YamlReader bodies_reader =
        reader.Subnode("bodies", flatland_server::YamlReader::LIST);
    for (int i = 0; i < bodies_reader.NodeSize(); i++) {
      flatland_server::YamlReader body_reader =
          bodies_reader.Subnode(i, flatland_server::YamlReader::MAP);
      if (!body_reader.Get<bool>("enabled", "true")) {
        continue;
      }

      auto pose = body_reader.GetPose("pose", flatland_server::Pose());

      flatland_server::YamlReader footprints_node =
          body_reader.Subnode("footprints", flatland_server::YamlReader::LIST);
      for (int j = 0; j < footprints_node.NodeSize(); j++) {
        flatland_server::YamlReader footprint =
            footprints_node.Subnode(j, flatland_server::YamlReader::MAP);

        lines_list_.push_back(std::make_shared<rviz_rendering::BillboardLine>(
            context_->getSceneManager(), moving_model_node_));
        auto lines = lines_list_.back();
        lines->setColor(0.0, 1.0, 0.0, 0.75);
        lines->setLineWidth(0.05);
        lines->setOrientation(
            Ogre::Quaternion(Ogre::Radian(pose.theta), Ogre::Vector3(0, 0, 1)));
        lines->setPosition(Ogre::Vector3(pose.x, pose.y, 0));

        std::string type = footprint.Get<std::string>("type");
        if (type == "circle") {
          LoadCircleFootprint(footprint, pose);
        } else if (type == "polygon") {
          LoadPolygonFootprint(footprint, pose);
        } else {
          throw flatland_server::YAMLException("Invalid footprint \"type\"");
        }
      }
    }
  } catch (const flatland_server::YAMLException &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("flatland_viz"),
                        "Couldn't load model bodies for preview" << e.what());
  }
}

void SpawnModelTool::LoadPolygonFootprint(
    flatland_server::YamlReader &footprint, const flatland_server::Pose pose) {
  auto lines = lines_list_.back();
  auto points = footprint.GetList<flatland_server::Vec2>("points", 3,
                                                         b2_maxPolygonVertices);
  for (auto p : points) {
    lines->addPoint(Ogre::Vector3(p.x, p.y, 0.));
  }
  if (points.size() > 0) {
    lines->addPoint(Ogre::Vector3(points.at(0).x, points.at(0).y, 0.));
  }
}

void SpawnModelTool::LoadCircleFootprint(flatland_server::YamlReader &footprint,
                                         const flatland_server::Pose pose) {
  auto lines = lines_list_.back();
  auto center = footprint.GetVec2("center", flatland_server::Vec2());
  auto radius = footprint.Get<float>("radius", 1.0);
  for (float a = 0.; a < M_PI * 2.0; a += M_PI / 8.) {
    lines->addPoint(Ogre::Vector3(center.x + radius * cos(a),
                                  center.y + radius * sin(a), 0.));
  }
  lines->addPoint(Ogre::Vector3(center.x + radius, center.y, 0.));
}

void SpawnModelTool::SavePath(QString p) {
  path_to_model_file_ = p;
  LoadPreview();
}

void SpawnModelTool::SaveName(QString n) { model_name = n; }

}  // namespace flatland_viz

PLUGINLIB_EXPORT_CLASS(flatland_viz::SpawnModelTool, rviz_common::Tool)
