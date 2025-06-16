/*********************************************************************
 *
 * Software License Agreement (BSD License)
 * Copyright (c) 2016,
 * TU Dortmund - Institute of Control Theory and Systems Engineering.
 * All rights reserved.
 *
 *********************************************************************/
 
#include <functional>
#include <memory>
#include <thread>
 
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
 
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
 
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_converter/costmap_converter_node.h>
#include <pluginlib/class_loader.hpp>
 
CostmapStandaloneConversion::CostmapStandaloneConversion(const rclcpp::NodeOptions & options)
: rclcpp::Node("costmap_converter", options),
  converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
{
  // Construct Costmap2DROS using ROS2-compliant constructor
  bool use_sim_time = this->get_parameter_or("use_sim_time", false);
costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("map", "base_link", "converter_costmap", use_sim_time);
 
  // Spin up separate thread to run the costmap's lifecycle node
  costmap_thread_ = std::make_unique<std::thread>(
    [](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
      rclcpp::spin(node->get_node_base_interface());
    },
    costmap_ros_);
 
  rclcpp_lifecycle::State state;
  costmap_ros_->on_configure(state);
  costmap_ros_->on_activate(state);
 
  n_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
 
  std::string converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
  declare_parameter("converter_plugin", converter_plugin);
  get_parameter("converter_plugin", converter_plugin);
 
  try {
    converter_ = converter_loader_.createSharedInstance(converter_plugin);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load plugin. Error: %s", ex.what());
    rclcpp::shutdown();
    return;
  }
 
  RCLCPP_INFO(get_logger(), "Standalone costmap converter: %s loaded.", converter_plugin.c_str());
 
  std::string obstacles_topic = "costmap_obstacles";
  declare_parameter("obstacles_topic", obstacles_topic);
  get_parameter("obstacles_topic", obstacles_topic);
 
  std::string polygon_marker_topic = "costmap_polygon_markers";
  declare_parameter("polygon_marker_topic", polygon_marker_topic);
  get_parameter("polygon_marker_topic", polygon_marker_topic);
 
  obstacle_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(obstacles_topic, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(polygon_marker_topic, 10);
 
  declare_parameter("occupied_min_value", 100);
  get_parameter("occupied_min_value", occupied_min_value_);
 
  declare_parameter("conversion_interval", 500);
  get_parameter("conversion_interval", conversion_interval_);
 
  std::string odom_topic = "/odom";
  declare_parameter("odom_topic", odom_topic);
  get_parameter("odom_topic", odom_topic);
 
  if (converter_) {
    converter_->setOdomTopic(odom_topic);
    converter_->initialize(shared_from_this());
    converter_->setCostmap2D(costmap_ros_->getCostmap());
  }
 
  last_publish_time_ = now();
 
  cb_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 
  pub_timer_ = n_->create_wall_timer(
    std::chrono::milliseconds(conversion_interval_),
    std::bind(&CostmapStandaloneConversion::publishCallback, this),
    cb_group1_);
 
  health_check_timer_ = n_->create_wall_timer(
    std::chrono::milliseconds(5000),
    std::bind(&CostmapStandaloneConversion::healthCheck, this),
    cb_group2_);
}
 
void CostmapStandaloneConversion::healthCheck()
{
  if (respawn_ && now() - last_publish_time_ > std::chrono::seconds(20)) {
    exit(0);
  }
  if (now() - last_publish_time_ > std::chrono::seconds(10)) {
    RCLCPP_ERROR(get_logger(), "costmap_converter_node has not published for 10 seconds, terminating...");
    respawn_ = true;
  }
}
 
void CostmapStandaloneConversion::publishCallback()
{
  converter_->workerCallback();
 
  if (respawn_) {
    RCLCPP_INFO(get_logger(), "getting obstacles...");
  }
 
  auto obstacles = converter_->getObstacles();
  if (!obstacles) return;
 
  frame_id_ = costmap_ros_->getGlobalFrameID();
  obstacles->header.frame_id = frame_id_;
  obstacles->header.stamp = now();
 
  obstacle_pub_->publish(*obstacles);
 
  if (respawn_) {
    RCLCPP_INFO(get_logger(), "published obstacles");
  }
 
  publishAsMarker(*obstacles);
 
  if (respawn_) {
    RCLCPP_INFO(get_logger(), "published markers");
  }
 
  last_publish_time_ = now();
}
 
void CostmapStandaloneConversion::publishAsMarker(
  const costmap_converter_msgs::msg::ObstacleArrayMsg & obstacles)
{
  visualization_msgs::msg::Marker line_list;
  line_list.header.frame_id = obstacles.header.frame_id;
  line_list.header.stamp = obstacles.header.stamp;
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::msg::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;
 
  for (const auto & obstacle : obstacles.obstacles) {
    for (int j = 0; j < static_cast<int>(obstacle.polygon.points.size()) - 1; ++j) {
      geometry_msgs::msg::Point start, end;
      start.x = obstacle.polygon.points[j].x;
      start.y = obstacle.polygon.points[j].y;
      start.z = obstacle.polygon.points[j].z;
 
      end.x = obstacle.polygon.points[j + 1].x;
      end.y = obstacle.polygon.points[j + 1].y;
      end.z = obstacle.polygon.points[j + 1].z;
 
      line_list.points.push_back(start);
      line_list.points.push_back(end);
    }
 
    // Close polygon
    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() > 2) {
      geometry_msgs::msg::Point last, first;
      last.x = obstacle.polygon.points.back().x;
      last.y = obstacle.polygon.points.back().y;
      last.z = obstacle.polygon.points.back().z;
 
      first.x = obstacle.polygon.points.front().x;
      first.y = obstacle.polygon.points.front().y;
      first.z = obstacle.polygon.points.front().z;
 
      line_list.points.push_back(last);
      line_list.points.push_back(first);
    }
  }
 
  marker_pub_->publish(line_list);
}
 
 
RCLCPP_COMPONENTS_REGISTER_NODE(CostmapStandaloneConversion)
 
