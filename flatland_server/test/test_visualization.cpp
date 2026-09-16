// Copyright (c) 2026, Levent Soysal (Worthle).
// SPDX-License-Identifier: BSD-3-Clause

#include <flatland_server/body.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/ros_node.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>

namespace flatland_server {

TEST(Pose, ReadsHeadingFromThreeElementArray) {
  const Pose pose(std::array<double, 3>{1.0, 2.0, 0.75});
  EXPECT_DOUBLE_EQ(1.0, pose.x);
  EXPECT_DOUBLE_EQ(2.0, pose.y);
  EXPECT_DOUBLE_EQ(0.75, pose.theta);
}

TEST(CollisionFilterRegistry, SkipsUnknownLayersWithOrWithoutErrorOutput) {
  CollisionFilterRegistry registry;
  ASSERT_EQ(0, registry.RegisterLayer("ground"));
  EXPECT_EQ(1u, registry.GetCategoryBits({"ground", "missing"}));
  EXPECT_EQ(0u, registry.GetCategoryBits({"missing"}));
  std::vector<std::string> invalid;
  EXPECT_EQ(1u, registry.GetCategoryBits({"ground", "missing"}, &invalid));
  EXPECT_EQ((std::vector<std::string>{"missing"}), invalid);
  EXPECT_EQ(65535u, registry.GetCategoryBits({"all"}));
}

class VisualizationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    ros_node() = std::make_shared<rclcpp::Node>("visualization_test");
  }

  static void TearDownTestSuite() {
    ros_node().reset();
    rclcpp::shutdown();
  }

  VisualizationTest()
      : world_(b2Vec2(0, 0)),
        body_(&world_, nullptr, "body", Color(1, 1, 1, 1), Pose(2, 3, 0),
              b2_dynamicBody, YAML::Node()) {
    b2PolygonShape shape;
    shape.SetAsBox(0.5, 0.25);
    body_.physics_body_->CreateFixture(&shape, 1.0);
    DebugVisualization::Get().default_extrude_height_ = 0.0;
  }

  visualization_msgs::msg::MarkerArray Render() {
    visualization_msgs::msg::MarkerArray markers;
    DebugVisualization::Get().BodyToMarkers(markers, body_.physics_body_, 1, 1,
                                            1, 1);
    return markers;
  }

  b2World world_;
  Body body_;
};

TEST_F(VisualizationTest, ExtrudesPolygonsAndAppliesVisualHeight) {
  body_.extrude_height_ = 0.4;
  body_.elevation_ = 0.2;
  body_.visual_z_offset_ = 0.1;
  const auto markers = Render();
  ASSERT_EQ(1u, markers.markers.size());
  const auto &marker = markers.markers[0];
  EXPECT_EQ(visualization_msgs::msg::Marker::TRIANGLE_LIST, marker.type);
  EXPECT_NEAR(0.3, marker.pose.position.z, 1e-9);
  const auto heights = std::minmax_element(
      marker.points.begin(), marker.points.end(),
      [](const auto &a, const auto &b) { return a.z < b.z; });
  ASSERT_FALSE(marker.points.empty());
  EXPECT_DOUBLE_EQ(0.0, heights.first->z);
  EXPECT_NEAR(0.4, heights.second->z, 1e-7);
  EXPECT_FLOAT_EQ(2.0f, body_.physics_body_->GetPosition().x);
  EXPECT_FLOAT_EQ(3.0f, body_.physics_body_->GetPosition().y);
}

TEST_F(VisualizationTest, RendersMeshWithoutReplacingCollisionFixtures) {
  body_.visual_mesh_ = "package://example/meshes/body.dae";
  body_.elevation_ = 0.3;
  const auto markers = Render();
  ASSERT_EQ(1u, markers.markers.size());
  const auto &marker = markers.markers[0];
  EXPECT_EQ(visualization_msgs::msg::Marker::MESH_RESOURCE, marker.type);
  EXPECT_EQ(body_.visual_mesh_, marker.mesh_resource);
  EXPECT_TRUE(marker.mesh_use_embedded_materials);
  EXPECT_DOUBLE_EQ(0.3, marker.pose.position.z);
  EXPECT_EQ(1, body_.GetFixturesCount());
}

TEST_F(VisualizationTest, AnimatesWheelRollFromTravelledDistance) {
  body_.wheel_visual_.radius = 0.1;
  body_.wheel_visual_.width = 0.05;
  const auto stationary = Render();
  ASSERT_FALSE(stationary.markers.empty());
  const double initial = body_.wheel_visual_.rotation;
  body_.physics_body_->SetTransform(b2Vec2(2.1f, 3.0f), 0.0f);
  const auto moved = Render();
  EXPECT_EQ(stationary.markers.size(), moved.markers.size());
  EXPECT_NEAR(1.0, std::abs(body_.wheel_visual_.rotation - initial), 1e-5);
  EXPECT_EQ(1, body_.GetFixturesCount());
}

}  // namespace flatland_server
