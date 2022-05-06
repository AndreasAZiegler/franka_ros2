// Copyright (c) 2021 Franka Emika GmbH
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

#include <balance_controller/balance_controller.hpp>

#include <exception>
#include <string>

namespace balance_controller
{
BalanceController::BalanceController() : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration BalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i)
  {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration BalanceController::state_interface_configuration() const
{
  return {};
}

controller_interface::return_type BalanceController::update()
{
  /*
  auto now = std::chrono::system_clock::now();
  delta_t_ = (now - last_update_time_stamp_).count();
  last_update_time_stamp_ = now;

  // Integral of the error
  auto x_error_integral = x_prev_error_integral_ + current_position_.x * delta_t_;
  auto y_error_integral = y_prev_error_integral_ + current_position_.y * delta_t_;

  x_prev_error_integral_ = x_error_integral;
  y_prev_error_integral_ = y_error_integral;

  // Anti-windup
  // x_error_integral = std::max(std::min(x_error_integral, MAX_VALUE), -MAX_VALUE);

  // Derivative of the error
  auto x_error_derivative = (current_position_.x - previous_position_.x) / delta_t_;
  auto y_error_derivative = (current_position_.y - previous_position_.y) / delta_t_;

  // Controll coefficients
  double k_p = 5;
  double k_i = 0.2;
  double k_d = 0.1;

  auto x_delta =
      k_p * current_position_.x + k_i * x_prev_error_integral_ + k_d * x_error_derivative;
  auto y_delta =
      k_p * current_position_.y + k_i * y_prev_error_integral_ + k_d * y_error_derivative;
  */

  updateJointStates();

  JointTrajectoryPoint state_current, state_desired, state_error;
  state_current.positions.resize(num_joints);
  state_current.velocities.resize(num_joints);
  state_current.effort.resize(num_joints);
  state_current.accelerations.resize(num_joints);
  state_desired.positions.resize(num_joints);
  state_desired.effort.resize(num_joints);
  state_desired.accelerations.resize(num_joints);
  state_error.positions.resize(num_joints);

  Eigen::Matrix<double, 2, 1> difference;
  difference << position_goal_(5) - joint_positions_(5), position_goal_(6) - joint_positions_(6);
  if (difference.norm() > 0.05)
  {
  //std::cout << "(position_goal_ - joint_positions_).norm(): " << (position_goal_ - joint_positions_).norm() << std::endl;

  auto time = this->node_->now() - start_time_;
  //double delta_angle = M_PI / 12.0 * (1 - std::cos(M_PI / 2.5 * time.seconds()));

  //std::cout << "joint_positions_: " << joint_positions_ << std::endl;
  //std::cout << "position_goal: " << position_goal << std::endl;

  const double kAlpha = 0.99;
  joint_velocities_filtered_ = (1 - kAlpha) * joint_velocities_filtered_ + kAlpha * joint_velocities_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(position_goal_ - joint_positions_) + d_gains_.cwiseProduct(-joint_velocities_filtered_);

  /*
  std::cout << "joint_positions_(6): " << joint_positions_(6) << std::endl;
  std::cout << "position_goal_(6): " << position_goal_(6) << std::endl;
  std::cout << "k_gains_.cwiseProduct(position_goal_ - joint_positions_)(6): " << k_gains_.cwiseProduct(position_goal_ - joint_positions_)(6) << std::endl;
  std::cout << "d_gains_.cwimeProduct(-joint_velocities_filtered_)(6): " << d_gains_.cwiseProduct(-joint_velocities_filtered_)(6) << std::endl;
  */

  for (int i = 0; i < num_joints; ++i)
  {
    command_interfaces_[i].set_value(tau_d_calculated(i));
    //state_current.effort.at(i) = tau_d_calculated(i);
    state_desired.effort.at(i) = tau_d_calculated(i);
  }

  state_desired.accelerations.at(0) = k_gains_.cwiseProduct(position_goal_ - joint_positions_)(6);
  state_desired.accelerations.at(1) = d_gains_.cwiseProduct(-joint_velocities_filtered_)(6);


  /*
  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0);
  }
  // command_interfaces_.at(6).set_value(-0.6);
  command_interfaces_.at(5).set_value(1.25);
  */
  }

  // current state update
  state_current.time_from_start.set__sec(0);

  for (auto i = 0; i < num_joints; ++i)
  {
    state_current.positions.at(i) = joint_positions_(i);
    state_current.velocities.at(i) = joint_velocities_(i);
  }

  state_desired.positions.at(5) = position_goal_(5);
  state_desired.positions.at(6) = position_goal_(6);

  state_error.positions.at(5) = position_goal_(5) - joint_positions_(5);
  state_error.positions.at(6) = position_goal_(6) - joint_positions_(6);

  publish_state(state_desired, state_current, state_error);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BalanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  arm_id_ = node_->get_parameter("arm_id").as_string();

  auto k_gains = node_->get_parameter("k_gains").as_double_array();
  auto d_gains = node_->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(node_->get_logger(), "k_gains should be of size %d but is of size %d", num_joints,
                 k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(node_->get_logger(), "d_gains should be of size %d but is of size %d", num_joints,
                 d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  joint_velocities_filtered_.setZero();

  // State publisher
  //const double state_publish_rate = node_->get_parameter("state_publish_rate").get_value<double>();
  const double state_publish_rate = 100;
  RCLCPP_INFO(node_->get_logger(), "Controller state will be published at %.2f Hz.", state_publish_rate);
  if (state_publish_rate > 0.0)
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);
  }
  else
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
  }

  publisher_ = node_->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);


  state_publisher_->lock();
  //state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.desired.positions.resize(num_joints);
  state_publisher_->msg_.desired.velocities.resize(num_joints);
  state_publisher_->msg_.desired.accelerations.resize(num_joints);
  state_publisher_->msg_.actual.positions.resize(num_joints);
  state_publisher_->msg_.error.positions.resize(num_joints);
  state_publisher_->msg_.actual.velocities.resize(num_joints);
  state_publisher_->msg_.error.velocities.resize(num_joints);
  state_publisher_->unlock();

  last_state_publish_time_ = node_->now();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BalanceController::init(const std::string& controller_name)
{
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  tracking_sub_ = node_->create_subscription<ball_tracker_msgs::msg::TrackingUpdate>(
      "/camera1/tracking_update", rclcpp::SystemDefaultsQoS(),
      std::bind(&BalanceController::tracking_callback, this, std::placeholders::_1));

  joint_position_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>("joint_position", rclcpp::SystemDefaultsQoS(), std::bind(&BalanceController::position_callback, this, std::placeholders::_1));

  try
  {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void BalanceController::updateJointStates()
{
  for (auto i = 0; i < num_joints; ++i)
  {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_(i) = position_interface.get_value();
    joint_velocities_(i) = velocity_interface.get_value();
  }
}

void BalanceController::tracking_callback(const ball_tracker_msgs::msg::TrackingUpdate::SharedPtr msg)
{
  current_position_.x = msg->x;
  current_position_.y = msg->y;
}


void BalanceController::position_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  position_goal_(6) = msg->data;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BalanceController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  updateJointStates();
  start_time_ = this->node_->now();

  position_goal_ = joint_positions_;
  double delta_angle = 0.035;
  //position_goal(5) += delta_angle;
  //position_goal_(6) += delta_angle;

  return CallbackReturn::SUCCESS;
}

void BalanceController::publish_state(
  const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
  const JointTrajectoryPoint & state_error)
{
  if (state_publisher_period_.seconds() <= 0.0)
  {
    return;
  }

  if (node_->now() < (last_state_publish_time_ + state_publisher_period_))
  {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    last_state_publish_time_ = node_->now();
    state_publisher_->msg_.header.stamp = last_state_publish_time_;
    state_publisher_->msg_.desired.positions = desired_state.positions;
    state_publisher_->msg_.desired.effort = desired_state.effort;
    state_publisher_->msg_.desired.velocities = desired_state.velocities;
    state_publisher_->msg_.desired.accelerations = desired_state.accelerations;
    state_publisher_->msg_.actual.positions = current_state.positions;
    state_publisher_->msg_.actual.effort = current_state.effort;
    state_publisher_->msg_.actual.accelerations = current_state.accelerations;
    state_publisher_->msg_.error.positions = state_error.positions;
    state_publisher_->msg_.actual.velocities = current_state.velocities;
    state_publisher_->msg_.error.velocities = state_error.velocities;

    state_publisher_->unlockAndPublish();
  }
}

}  // namespace balance_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerInterface)
