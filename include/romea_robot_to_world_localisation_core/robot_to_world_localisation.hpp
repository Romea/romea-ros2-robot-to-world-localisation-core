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

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_HPP_

// std
#include <memory>

// romea
#include "romea_common_utils/conversions/fsm_conversions.hpp"
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include "romea_common_utils/log/console_fsm_logger.hpp"
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_common_utils/publishers/odom_publisher.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/publishers/transform_publisher.hpp"

// local
#include "romea_robot_to_world_localisation_core/robot_to_world_localisation_filter.hpp"
#include "romea_robot_to_world_localisation_core/visibility_control.h"

namespace romea
{
namespace ros2
{
namespace localisation
{

template<core::FilterType FilterType_>
class R2WLocalisation
{
public:
  ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE_PUBLIC
  explicit R2WLocalisation(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE_PUBLIC
  virtual ~R2WLocalisation() = default;

  ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  void make_filter_();

  void make_timer_();

  void make_status_publisher_();

  void make_fsm_event_publisher_();

  void make_console_fsm_event_logger_();

  void make_odom_publisher_();

  void make_tf_publisher_();

  void make_diagnostic_publisher_();

  void publish_diagnostics_(
    const rclcpp::Time & stamp,
    const core::localisation::FSMState & fsm_state);

  void publish_status_(const rclcpp::Time & stamp, const core::localisation::FSMState & fsm_state);

  void publish_fsm_event_(const core::FSMEvent & event);

  void timer_callback_();

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::unique_ptr<R2WLocalisationFilter<FilterType_>> filter_;

  std::shared_ptr<StampedPublisherBase<core::Pose3D>> tf_publisher_;
  std::shared_ptr<StampedPublisherBase<core::PoseAndTwist3D>> odom_publisher_;
  std::shared_ptr<StampedPublisherBase<core::DiagnosticReport>> diagnostic_publisher_;
  std::shared_ptr<PublisherBase<core::FSMState>> status_publisher_;
  std::shared_ptr<StampedPublisherBase<core::FSMEvent>> fsm_event_publisher_;
  std::unique_ptr<ConsoleFSMLogger> console_fsm_event_logger_;
};

using R2WKalmanLocalisation = R2WLocalisation<core::FilterType::KALMAN>;
using R2WParticleLocalisation = R2WLocalisation<core::FilterType::PARTICLE>;

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_HPP_
