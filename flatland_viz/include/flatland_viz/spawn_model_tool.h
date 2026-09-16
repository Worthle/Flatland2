#ifndef FLATLAND_VIZ_SPAWN_MODEL_TOOL_H
#define FLATLAND_VIZ_SPAWN_MODEL_TOOL_H

#include <memory>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <QColor>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <flatland_server/yaml_reader.h>
#include <flatland_msgs/srv/spawn_model.hpp>

namespace flatland_viz {

/**
 * @brief rviz2 tool to interactively spawn a flatland model: pick a yaml file,
 * drag the preview to a location, then click to set the heading and spawn it
 * through the flatland_server /spawn_model service.
 */
class SpawnModelTool : public rviz_common::Tool {
  Q_OBJECT

 public:
  SpawnModelTool();
  ~SpawnModelTool() override;

  void BeginPlacement();
  void SavePath(QString p);
  void SaveName(QString n);
  void SpawnModelInFlatland();

 private:
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;

  void SetMovingModelColor(QColor c);
  void LoadPreview();
  void LoadPolygonFootprint(flatland_server::YamlReader &footprint,
                            const flatland_server::Pose pose);
  void LoadCircleFootprint(flatland_server::YamlReader &footprint,
                           const flatland_server::Pose pose);

  Ogre::Vector3 intersection;
  float initial_angle;
  Ogre::SceneNode *moving_model_node_;
  enum ModelState { m_hidden, m_dragging, m_rotating };
  ModelState model_state;
  static QString path_to_model_file_;
  static QString model_name;

  rviz_rendering::Arrow *arrow_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<flatland_msgs::srv::SpawnModel>::SharedPtr client_;
  std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> lines_list_;
};

}  // namespace flatland_viz

#endif  // FLATLAND_VIZ_SPAWN_MODEL_TOOL_H
