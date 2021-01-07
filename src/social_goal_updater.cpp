// Copyright 2020
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Jonatan Ginés jonatan.gines@urjc.es */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/time.hpp"

#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "social_navigation_msgs/msg/set_human_action.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "social_navigation2_actions/social_goal_updater.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using CallbackReturnT =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GetParameters = rcl_interfaces::srv::GetParameters;
using SetParameters = rcl_interfaces::srv::SetParameters;
using SetHumanAction = social_navigation_msgs::msg::SetHumanAction;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace social_nav2_actions
{
  SocialGoalUpdater::SocialGoalUpdater(const std::string & node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name), 
    tf2_buffer_(get_clock()),
    tf2_listener_(tf2_buffer_, true)
  {
    initParams();
    node_name_ = node_name;
  }

  CallbackReturnT SocialGoalUpdater::on_configure(const rclcpp_lifecycle::State & state) {
    costmap_sub_ =
      std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
      shared_from_this(), "global_costmap/costmap_raw");
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    goal_update_pub_ =
      std::make_shared<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>(
        get_node_base_interface().get(), 
        goal_update_topic_, 
        rclcpp::QoS(1),
        options);
    action_pub_ =
      std::make_shared<rclcpp_lifecycle::LifecyclePublisher<SetHumanAction>>(
        get_node_base_interface().get(), 
        std::string("/social_navigation/set_agent_action"), 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
        options);

    add_publisher_handle(goal_update_pub_);
    add_publisher_handle(action_pub_);

    goal_update_pub_->on_activate();
    action_pub_->on_activate();
    private_node_ = rclcpp::Node::make_shared("hri_goal_updater_get_params");
    params_client_ = private_node_->create_client<GetParameters>(
      "/global_costmap/global_costmap/get_parameters");
    agent_id_service_ = create_service<SetParameters>(
      node_name_ + "/set_agent_id", std::bind(&SocialGoalUpdater::setAgentId, this, _1, _2));
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  CallbackReturnT SocialGoalUpdater::on_activate(const rclcpp_lifecycle::State & state) {
    bool is_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for params...");
      is_server_ready =
        params_client_->wait_for_service(std::chrono::seconds(5));
    } while (!is_server_ready);

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back("robot_radius");
    request->names.push_back("social_layer.intimate_z_radius");

    params_future_ = params_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(private_node_, params_future_) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      params_handle_ = params_future_.get();
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call service reverse_string");
    }

    params_map.insert(std::pair<std::string, float>(
        "robot_radius",
        params_handle_->values[0].double_value));
    params_map.insert(std::pair<std::string, float>(
        "intimate_z_radius",
        params_handle_->values[1].double_value));
    RCLCPP_INFO(get_logger(), "Lifecyclenode ready");

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  bool SocialGoalUpdater::getTF(std::string target, std::string source, tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped global2agent;
    try {
      // Check if the transform is available
      global2agent = tf2_buffer_.lookupTransform(target, source, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      return false;
    }
    tf2::impl::Converter<true, false>::convert(global2agent.transform, tf);
    return true;
  }

  geometry_msgs::msg::Pose SocialGoalUpdater::tf2ToPose(
    tf2::Vector3 position, 
    tf2::Quaternion rotation)
  {
    geometry_msgs::msg::Pose output;
    output.position.x = position.getX();
    output.position.y = position.getY();
    output.position.z = position.getZ();

    output.orientation.x = rotation.getX();
    output.orientation.y = rotation.getY();
    output.orientation.z = rotation.getZ();
    output.orientation.w = rotation.getW();
    return output;
  }

  void SocialGoalUpdater::updateGoal(const geometry_msgs::msg::Pose & goal)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;

    pose_stamped.header.frame_id = static_frame_;
    pose_stamped.header.stamp = now();
    pose_stamped.pose = goal;

    goal_update_pub_->publish(pose_stamped);

    RCLCPP_INFO(get_logger(), "Update goal!");
  }

  void SocialGoalUpdater::setAgentId(
    const std::shared_ptr<SetParameters::Request> request,
    std::shared_ptr<SetParameters::Response> response)
  {
    if (request->parameters[0].name == "agent_id") {
      agent_id_ = request->parameters[0].value.string_value;
      RCLCPP_INFO(get_logger(), "Agent id was set [%s]", agent_id_.c_str());
    }
  }
  void SocialGoalUpdater::setSocialAction(std::string action) 
  {
    auto message = SetHumanAction();
    message.agent_id = agent_id_;
    message.action = action;
    action_pub_->publish(message);
  }

  void SocialGoalUpdater::initParams()
  {
    agent_id_ = "";
    static_frame_ = "map";
    goal_update_topic_ = "/goal_update";
  }
}