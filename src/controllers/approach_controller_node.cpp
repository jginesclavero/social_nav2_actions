// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>

#include "plansys2_msgs/action/execute_action.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class Controller : public rclcpp::Node
{
public:
  Controller()
  : rclcpp::Node("social_nav2_actions_controller"), state_(STARTING)
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
    executor_state_client_ = shared_from_this()->create_client<lifecycle_msgs::srv::GetState>(
      "/executor/get_state");
    knowledge_ready = false;
    update_approach_tf_pub_ =
      create_publisher<std_msgs::msg::Empty>(
      "social_navigation/update_approach_tf",
      rclcpp::SystemDefaultsQoS());
    plot_csv_pub_ =
      create_publisher<std_msgs::msg::Empty>(
      "social_nav_exp/task_finished",
      rclcpp::SystemDefaultsQoS());
  }

  void init_knowledge()
  {
    bool is_server_ready;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for executor...");
      is_server_ready =
        executor_state_client_->wait_for_service(std::chrono::seconds(5));
    } while (!is_server_ready);
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = executor_state_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      if (result.get()->current_state.label == "active") {
        problem_expert_->addInstance(plansys2::Instance{"leia", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"agent_1", "agent_id"});
        problem_expert_->addInstance(plansys2::Instance{"agent_2", "agent_id"});
        problem_expert_->addInstance(plansys2::Instance{"agent_3", "agent_id"});
        problem_expert_->addInstance(plansys2::Instance{"wp_home", "waypoint"});
        problem_expert_->addInstance(plansys2::Instance{"wp_aux", "waypoint"});
        problem_expert_->addPredicate(plansys2::Predicate("(robot_at leia wp_home)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected wp_aux wp_home)"));

        knowledge_ready = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to call service executor/get_state");
    }
  }

  void step()
  {
    if (!knowledge_ready) {
      init_knowledge();
    } else {
      switch (state_) {
        case STARTING:
          {
            if (executor_client_) {
              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(approached agent_3))"));
              if (executor_client_->executePlan()) {
                state_ = RUNNING;
              }
            }
          }
          break;
        case RUNNING:
          {
            if (executor_client_->getResult().has_value()) {
              if (executor_client_->getResult().value().success) {
                std::cout << "RUNNING Successful finished " << std::endl;

                // Cleanning up
                problem_expert_->removePredicate(plansys2::Predicate("(approached agent_3)"));
                problem_expert_->removePredicate(plansys2::Predicate("(robot_at leia wp_home)"));
                problem_expert_->addPredicate(plansys2::Predicate("(robot_at leia wp_aux)"));
                // Set the goal for next state, and execute plan
                problem_expert_->setGoal(plansys2::Goal("(and(robot_at leia wp_home))"));

                if (executor_client_->executePlan()) {
                  state_ = RETURNING;
                }
              } else {
                std::cout << "RUNNING Finished with error: " <<
                  executor_client_->getResult().value().error_info <<
                  std::endl;
                executor_client_->executePlan();  // replan and execute
              }
            }
          }
          break;
          case RETURNING:
          {
            if (executor_client_->getResult().has_value()) {
              if (executor_client_->getResult().value().success) {
                update_approach_tf_pub_->publish(std_msgs::msg::Empty());
                plot_csv_pub_->publish(std_msgs::msg::Empty());
                std::cout << "RETURNING Successful finished " << std::endl;
                state_ = STARTING;
              } else {
                std::cout << "RETURNING Finished with error: " <<
                  executor_client_->getResult().value().error_info <<
                  std::endl;
                executor_client_->executePlan();  // replan and execute
              }
            }
          }
          break;
      }
    }
  }

private:
  typedef enum {STARTING, RUNNING, RETURNING} StateType;
  StateType state_;
  bool knowledge_ready;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> executor_state_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr update_approach_tf_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr plot_csv_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
