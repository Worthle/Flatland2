#include <flatland_viz/pause_sim_tool.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace flatland_viz {

PauseSimTool::PauseSimTool() {}

PauseSimTool::~PauseSimTool() {}

void PauseSimTool::onInitialize() {
  setName("Pause/Resume");
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pause_client_ = node_->create_client<std_srvs::srv::Empty>("toggle_pause");
}

// Each time the user selects the tool, toggle the simulation pause state.
void PauseSimTool::activate() {
  if (!pause_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "toggle_pause service not available yet");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  pause_client_->async_send_request(request);
}

void PauseSimTool::deactivate() {}

}  // namespace flatland_viz

PLUGINLIB_EXPORT_CLASS(flatland_viz::PauseSimTool, rviz_common::Tool)
