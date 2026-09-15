#include <flatland_server/interactive_marker_manager.h>
#include <flatland_server/ros_node.h>

#include <cmath>
#include <functional>

namespace flatland_server {

InteractiveMarkerManager::InteractiveMarkerManager(
    std::vector<Model *> *model_list_ptr, PluginManager *plugin_manager_ptr) {
  models_ = model_list_ptr;
  plugin_manager_ = plugin_manager_ptr;
  manipulating_model_ = false;

  // Initialize interactive marker server
  interactive_marker_server_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>(
          "interactive_model_markers", ros_node());

  // Add "Delete Model" context menu option to menu handler and bind callback
  menu_handler_.setCheckState(
      menu_handler_.insert(
          "Delete Model",
          std::bind(&InteractiveMarkerManager::deleteModelMenuCallback, this,
                      std::placeholders::_1)),
      interactive_markers::MenuHandler::NO_CHECKBOX);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::createInteractiveMarker(
    const std::string &model_name, const Pose &pose,
    const visualization_msgs::msg::MarkerArray &body_markers) {
  // Set up interactive marker control objects to allow both translation and
  // rotation movement
  visualization_msgs::msg::InteractiveMarkerControl plane_control;
  plane_control.always_visible = true;
  plane_control.orientation.w = 0.707;
  plane_control.orientation.y = 0.707;
  plane_control.name = "move_xy";
  plane_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  visualization_msgs::msg::InteractiveMarkerControl rotate_control;
  rotate_control.always_visible = true;
  rotate_control.orientation.w = 0.707;
  rotate_control.orientation.y = 0.707;
  rotate_control.name = "rotate_z";
  rotate_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

  // Add a non-interactive text marker with the model name
  visualization_msgs::msg::InteractiveMarkerControl no_control;
  no_control.always_visible = true;
  no_control.name = "no_control";
  no_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::NONE;
  visualization_msgs::msg::Marker text_marker;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.pose.position.x = 0.5;
  text_marker.pose.position.y = 0.0;
  text_marker.scale.z = 0.5;
  text_marker.text = model_name;
  no_control.markers.push_back(text_marker);

  // Add a cube marker to be an easy-to-manipulate target in Rviz
  visualization_msgs::msg::Marker easy_to_click_cube;
  easy_to_click_cube.type = visualization_msgs::msg::Marker::CUBE;
  easy_to_click_cube.color.r = 0.0;
  easy_to_click_cube.color.g = 1.0;
  easy_to_click_cube.color.b = 0.0;
  easy_to_click_cube.color.a = 0.5;
  easy_to_click_cube.scale.x = 0.5;
  easy_to_click_cube.scale.y = 0.5;
  easy_to_click_cube.scale.z = 0.05;
  easy_to_click_cube.pose.position.x = 0.0;
  plane_control.markers.push_back(easy_to_click_cube);

  // NOTE: The ROS 1 version also embedded a copy of the model body outline
  // markers in the interactive marker. In ROS 2 those same body outlines are
  // already rendered by the flatland debug MarkerArray displays, so embedding
  // them here produced two coincident, coplanar line sets that z-fight and
  // shimmer ("flutter") in rviz. We keep only the clickable cube + name text;
  // the outline is provided by the debug markers. (body_markers unused.)
  (void)body_markers;

  // Send new interactive marker to server
  visualization_msgs::msg::InteractiveMarker new_interactive_marker;
  new_interactive_marker.header.frame_id = "map";
  new_interactive_marker.header.stamp = rclcpp::Time(0, 0);
  new_interactive_marker.name = model_name;
  new_interactive_marker.pose.position.x = pose.x;
  new_interactive_marker.pose.position.y = pose.y;
  new_interactive_marker.pose.orientation.w = cos(0.5 * pose.theta);
  new_interactive_marker.pose.orientation.z = sin(0.5 * pose.theta);
  new_interactive_marker.controls.push_back(plane_control);
  new_interactive_marker.controls.push_back(rotate_control);
  new_interactive_marker.controls.push_back(no_control);
  interactive_marker_server_->insert(new_interactive_marker);

  // Bind feedback callbacks for the new interactive marker
  interactive_marker_server_->setCallback(
      model_name,
      std::bind(&InteractiveMarkerManager::processMouseUpFeedback, this, std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP);
  interactive_marker_server_->setCallback(
      model_name,
      std::bind(&InteractiveMarkerManager::processMouseDownFeedback, this,
                  std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN);
  interactive_marker_server_->setCallback(
      model_name,
      std::bind(&InteractiveMarkerManager::processPoseUpdateFeedback, this,
                  std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);

  // Add context menu to the new interactive marker
  menu_handler_.apply(*interactive_marker_server_, model_name);

  // Apply changes to server
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::deleteModelMenuCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  // Delete the model just as when the DeleteModel service is called
  for (unsigned int i = 0; i < (*models_).size(); i++) {
    if ((*models_)[i]->GetName() == feedback->marker_name) {
      // delete the plugins associated with the model
      plugin_manager_->DeleteModelPlugin((*models_)[i]);
      delete (*models_)[i];
      (*models_).erase((*models_).begin() + i);

      // Also remove corresponding interactive marker
      deleteInteractiveMarker(feedback->marker_name);
      break;
    }
  }

  // Update menu handler and server
  menu_handler_.apply(*interactive_marker_server_, feedback->marker_name);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::deleteInteractiveMarker(
    const std::string &model_name) {
  // Remove target interactive marker by name and
  // update the server
  interactive_marker_server_->erase(model_name);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::processMouseUpFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  // Update model that was manipulated the same way
  // as when the MoveModel service is called
  for (unsigned int i = 0; i < models_->size(); i++) {
    if ((*models_)[i]->GetName() == feedback->marker_name) {
      Pose new_pose;
      new_pose.x = feedback->pose.position.x;
      new_pose.y = feedback->pose.position.y;
      new_pose.theta = atan2(
          2.0 * feedback->pose.orientation.w * feedback->pose.orientation.z,
          1.0 -
              2.0 * feedback->pose.orientation.z *
                  feedback->pose.orientation.z);
      (*models_)[i]->SetPose(new_pose);
      break;
    }
  }
  manipulating_model_ = false;
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::processMouseDownFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  manipulating_model_ = true;
}

void InteractiveMarkerManager::processPoseUpdateFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  pose_update_stamp_ = steady_clock_.now();
}

void InteractiveMarkerManager::update() {
  // Loop through each model, extract the pose of the root body,
  // and use it to update the interactive marker pose. Only
  // necessary to compute if user is not currently dragging
  // an interactive marker
  if (!manipulating_model_) {
    bool changed = false;
    for (size_t i = 0; i < (*models_).size(); i++) {
      geometry_msgs::msg::Pose new_pose;
      new_pose.position.x =
          (*models_)[i]->bodies_[0]->physics_body_->GetPosition().x;
      new_pose.position.y =
          (*models_)[i]->bodies_[0]->physics_body_->GetPosition().y;
      double theta = (*models_)[i]->bodies_[0]->physics_body_->GetAngle();
      new_pose.orientation.w = cos(0.5 * theta);
      new_pose.orientation.z = sin(0.5 * theta);

      // Only push an update when the pose actually changed. Re-publishing
      // identical poses every physics step (~100 Hz) makes the rviz
      // interactive markers ("Move Objects") flicker.
      const std::string &name = (*models_)[i]->GetName();
      auto it = last_poses_.find(name);
      const double eps = 1e-5;
      if (it == last_poses_.end() ||
          std::abs(it->second.position.x - new_pose.position.x) > eps ||
          std::abs(it->second.position.y - new_pose.position.y) > eps ||
          std::abs(it->second.orientation.z - new_pose.orientation.z) > eps ||
          std::abs(it->second.orientation.w - new_pose.orientation.w) > eps) {
        interactive_marker_server_->setPose(name, new_pose);
        last_poses_[name] = new_pose;
        changed = true;
      }
    }
    if (changed) {
      interactive_marker_server_->applyChanges();
    }
  }

  // Detect when interaction stops without triggering a MOUSE_UP event by
  // monitoring the time since the last pose update feedback, which comes
  // in at 33 Hz if the user is dragging the marker.  When the stream of
  // pose update feedback stops, automatically clear the manipulating_model_
  // flag to unpause the simulation.
  double dt = 0;
  try {
    dt = (steady_clock_.now() - pose_update_stamp_).seconds();
  } catch (std::runtime_error &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("flatland"), 
        "Flatland Interactive Marker Manager runtime error: (%f - %f) [%s]",
        steady_clock_.now().seconds(), pose_update_stamp_.seconds(), ex.what());
  }
  if (manipulating_model_ && dt > 0.1 && dt < 1.0) {
    manipulating_model_ = false;
  }
}

InteractiveMarkerManager::~InteractiveMarkerManager() {
  interactive_marker_server_.reset();
}
}
