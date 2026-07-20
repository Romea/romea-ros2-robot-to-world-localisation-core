// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

// std
#include <memory>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_robot_to_world_localisation_core/robot_to_world_localisation.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
R2WLocalisation<FilterType_>::R2WLocalisation(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("robot_to_world_localisation", options)),
  filter_(nullptr),
  tf_publisher_(nullptr),
  odom_publisher_(nullptr),
  diagnostic_publisher_(nullptr),
  fsm_event_publisher_(nullptr),
  console_fsm_event_logger_(nullptr)
{
  declare_debug(node_);
  declare_log_directory(node_);
  declare_base_footprint_frame_id(node_);
  declare_map_frame_id(node_);
  declare_publish_rate(node_);

  make_filter_();
  make_tf_publisher_();
  make_odom_publisher_();
  make_diagnostic_publisher_();
  make_status_publisher_();
  make_fsm_event_publisher_();
  make_console_fsm_event_logger_();
  make_timer_();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
R2WLocalisation<FilterType_>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_filter_()
{
  filter_ = std::make_unique<R2WLocalisationFilter<FilterType_>>(
    node_, [this](const core::FSMEvent & event) { publish_fsm_event_(event); });
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_status_publisher_()
{
  using DataType = core::FSMState;
  using MsgType = romea_common_msgs::msg::FSMState;
  status_publisher_ = make_data_publisher<DataType, MsgType>(node_, "status", reliable(1), true);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_fsm_event_publisher_()
{
  using DataType = core::FSMEvent;
  using MsgType = romea_common_msgs::msg::FSMEvent;
  fsm_event_publisher_ =
    make_stamped_data_publisher<DataType, MsgType>(
      node_, "debug/fsm", "", reliable(10), get_debug(node_));
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_console_fsm_event_logger_()
{
  console_fsm_event_logger_ = std::make_unique<ConsoleFSMLogger>(node_->get_logger());
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_odom_publisher_()
{
  odom_publisher_ = make_odom_publisher<core::PoseAndTwist3D>(
    node_,
    "filtered_odom",
    get_map_frame_id(node_),
    get_base_footprint_frame_id(node_),
    reliable(1),
    true);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_tf_publisher_()
{
  tf_publisher_ = make_transform_publisher<core::Pose3D>(
    node_, get_map_frame_id(node_), get_base_footprint_frame_id(node_), true);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_diagnostic_publisher_()
{
  diagnostic_publisher_ = make_diagnostic_publisher<core::DiagnosticReport>(
    node_, node_->get_fully_qualified_name(), 1.0);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::make_timer_()
{
  auto timer_period = core::durationFromSecond(1. / get_publish_rate(node_));
  auto callback = std::bind(&R2WLocalisation::timer_callback_, this);
  timer_ = node_->create_wall_timer(timer_period, callback);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::timer_callback_()
{
  // publish odom
  auto stamp = node_->get_clock()->now();

  auto fsm_state = filter_->get_fsm_state();
  if (fsm_state == core::localisation::FSMState::RUNNING) {
    const auto results = filter_->get_results(to_romea_duration(stamp));
    if (!results) {
      publish_status_(stamp, fsm_state);
      publish_diagnostics_(stamp);
      return;
    }

    core::PoseAndTwist3D odom;
    odom.pose = results->robot_pose;
    odom.twist = results->robot_twist;

    // std::cout << "odom " <<
    //   odom.pose.position.x() << " " <<
    //   odom.pose.position.y() << " " <<
    //   odom.pose.position.z() << " " <<
    //   odom.pose.orientation.x() << " " <<
    //   odom.pose.orientation.y() << " " <<
    //   odom.pose.orientation.z() << std::endl;

    odom_publisher_->publish(stamp, odom);
    tf_publisher_->publish(stamp, odom.pose);
  }

  publish_status_(stamp, fsm_state);
  publish_diagnostics_(stamp);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::publish_status_(
  const rclcpp::Time & /*stamp*/, const core::localisation::FSMState & fsm_state)
{
  auto current_state = core::localisation::to_common_fsm_state(fsm_state);
  status_publisher_->publish(current_state);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::publish_fsm_event_(const core::FSMEvent & event)
{
  fsm_event_publisher_->publish(node_->get_clock()->now(), event);
  console_fsm_event_logger_->log(event);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisation<FilterType_>::publish_diagnostics_(const rclcpp::Time & stamp)
{
  auto report = filter_->make_diagnostic_report(to_romea_duration(stamp));
  diagnostic_publisher_->publish(stamp, report);
}

template class R2WLocalisation<core::FilterType::KALMAN>;
template class R2WLocalisation<core::FilterType::PARTICLE>;

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::localisation::R2WKalmanLocalisation)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::localisation::R2WParticleLocalisation)
