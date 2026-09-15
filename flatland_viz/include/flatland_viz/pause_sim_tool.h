#ifndef FLATLAND_VIZ_PAUSE_SIM_TOOL_H
#define FLATLAND_VIZ_PAUSE_SIM_TOOL_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <std_srvs/srv/empty.hpp>

namespace flatland_viz {

/**
 * @brief rviz2 tool to pause / resume the flatland simulation by calling the
 * flatland_server /toggle_pause service.
 */
class PauseSimTool : public rviz_common::Tool {
  Q_OBJECT

 public:
  PauseSimTool();
  ~PauseSimTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
};

}  // namespace flatland_viz

#endif  // FLATLAND_VIZ_PAUSE_SIM_TOOL_H
