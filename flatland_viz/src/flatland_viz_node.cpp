/*
 * @name   flatland_viz_node.cpp
 * @brief  ROS 2 rviz2-based visualization app for flatland.
 *
 * Builds on rviz_common::VisualizationFrame (the standard rviz2 window, which
 * provides the menus / toolbar / tool add-remove / config load-save that the
 * ROS 1 flatland_viz reimplemented by hand). On top of that it:
 *   - loads the rviz config passed with -d,
 *   - adds the custom flatland SpawnModel and PauseSim tools,
 *   - auto-creates a MarkerArray display for every flatland debug topic
 *     advertised on /flatland_server/debug/topics.
 */

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <QApplication>
#include <QString>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>

#include <flatland_msgs/msg/debug_topic_list.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Parse "-d <config>" like rviz.
  QString config_path;
  for (int i = 1; i < argc; ++i) {
    if ((std::string(argv[i]) == "-d" ||
         std::string(argv[i]) == "--display-config") &&
        i + 1 < argc) {
      config_path = QString::fromLocal8Bit(argv[i + 1]);
      ++i;
    }
  }

  QApplication app(argc, argv);

  auto rviz_ros_node =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
          "flatland_viz");

  auto *frame = new rviz_common::VisualizationFrame(rviz_ros_node);
  frame->setApp(&app);
  frame->initialize(rviz_ros_node, config_path);
  frame->show();

  auto *manager = frame->getManager();
  manager->getToolManager()->addTool("flatland_viz/SpawnModel");
  manager->getToolManager()->addTool("flatland_viz/PauseSim");

  // Auto-create / remove MarkerArray displays for flatland debug topics.
  auto node = rviz_ros_node->get_raw_node();
  std::mutex topics_mutex;
  std::vector<std::string> latest_topics;
  bool dirty = false;
  std::map<std::string, rviz_common::Display *> debug_displays;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  auto sub = node->create_subscription<flatland_msgs::msg::DebugTopicList>(
      "/flatland_server/debug/topics", qos,
      [&](const flatland_msgs::msg::DebugTopicList::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(topics_mutex);
        latest_topics = msg->topics;
        dirty = true;
      });

  QTimer debug_timer;
  QObject::connect(&debug_timer, &QTimer::timeout, [&]() {
    std::vector<std::string> topics;
    {
      std::lock_guard<std::mutex> lock(topics_mutex);
      if (!dirty) {
        return;
      }
      topics = latest_topics;
      dirty = false;
    }

    // Remove displays whose topic disappeared.
    for (auto it = debug_displays.begin(); it != debug_displays.end();) {
      if (std::count(topics.begin(), topics.end(), it->first) == 0) {
        delete it->second;
        it = debug_displays.erase(it);
      } else {
        ++it;
      }
    }

    // Add displays for new topics.
    for (const auto &topic : topics) {
      if (debug_displays.count(topic) == 0) {
        rviz_common::Display *display = manager->createDisplay(
            "rviz_default_plugins/MarkerArray",
            QString::fromLocal8Bit(topic.c_str()), true);
        if (display) {
          QString topic_qt = QString::fromLocal8Bit(
              (std::string("/flatland_server/debug/") + topic).c_str());
          display->subProp("Topic")->setValue(topic_qt);
          debug_displays[topic] = display;
        }
      }
    }
  });
  debug_timer.start(250);

  app.exec();

  rclcpp::shutdown();
  return 0;
}
