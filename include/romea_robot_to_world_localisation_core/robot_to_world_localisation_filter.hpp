// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_core_localisation/filters.hpp"
#include "romea_core_localisation/robot_to_world/traits.hpp"
#include "romea_core_localisation/updater_traits.hpp"
#include "romea_localisation_utils/filter/loggers.hpp"
#include "romea_localisation_utils/filter/parameters.hpp"
#include "romea_localisation_utils/filter/updater_interface.hpp"
#include "romea_localisation_utils/filter/updater_interfaces.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

namespace ros_msg = romea_localisation_msgs::msg;

template<core::FilterType FilterType_>
class R2WLocalisationFilter
{
public:
  using Traits = core::localisation::R2WTraits<FilterType_>;
  using Filter = core::localisation::Filter<FilterType_, Traits>;
  using Results = typename Traits::Results;
  using Query = core::localisation::FilterQuery<Results>;
  using UpdaterTwist = typename Traits::UpdaterTwist;
  using UpdaterLinearSpeed = typename Traits::UpdaterLinearSpeed;
  using UpdaterLinearSpeeds = typename Traits::UpdaterLinearSpeeds;
  using UpdaterAngularSpeed = typename Traits::UpdaterAngularSpeed;
  using UpdaterAttitude = typename Traits::UpdaterAttitude;
  using UpdaterCourse = typename Traits::UpdaterCourse;
  using UpdaterRange = typename Traits::UpdaterRange;
  using UpdaterPose = typename Traits::UpdaterPose;
  using UpdaterPosition = typename Traits::UpdaterPosition;

  using UpdaterInterfaceTwist =
    UpdaterInterface<UpdaterTwist, ros_msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeed =
    UpdaterInterface<UpdaterLinearSpeed, ros_msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeeds =
    UpdaterInterface<UpdaterLinearSpeeds, ros_msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceAngularSpeed =
    UpdaterInterface<UpdaterAngularSpeed, ros_msg::ObservationAngularSpeedStamped>;
  using UpdaterInterfaceAttitude =
    UpdaterInterface<UpdaterAttitude, ros_msg::ObservationAttitudeStamped>;
  using UpdaterInterfaceCourse =
    UpdaterInterface<UpdaterCourse, ros_msg::ObservationCourseStamped>;
  using UpdaterInterfacePosition =
    UpdaterInterface<UpdaterPosition, ros_msg::ObservationPosition2DStamped>;
  using UpdaterInterfaceRange =
    UpdaterInterface<UpdaterRange, ros_msg::ObservationRangeStamped>;
  using UpdaterInterfacePose =
    UpdaterInterface<UpdaterPose, ros_msg::ObservationPose2DStamped>;

public:
  explicit R2WLocalisationFilter(
    std::shared_ptr<rclcpp::Node> node, core::FSMEventCallback fsm_event_callback = nullptr);

  void reset();

  Query get_results(const core::Duration & duration);

  core::DiagnosticReport make_diagnostic_report(
    const core::Duration & duration,
    const core::localisation::FSMState & fsm_state);

private:
  void configure_(std::shared_ptr<rclcpp::Node> node);

  void make_filter_(std::shared_ptr<rclcpp::Node> node);

  void add_updaters_(std::shared_ptr<rclcpp::Node> node);

  void initialize_filter_(std::shared_ptr<rclcpp::Node> node);

  typename Filter::LoggerMap make_loggers_(std::shared_ptr<rclcpp::Node> node);

  template<typename Interface>
  void add_updater_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name);

private:
  rclcpp::Logger logger_;
  std::unique_ptr<Filter> filter_;
  UpdaterInterfaces updater_interfaces_;
  core::FSMEventCallback fsm_event_callback_;
};

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
R2WLocalisationFilter<FilterType_>::R2WLocalisationFilter(
  std::shared_ptr<rclcpp::Node> node, core::FSMEventCallback fsm_event_callback)
: logger_(node->get_logger()),
  filter_(nullptr),
  updater_interfaces_(),
  fsm_event_callback_(std::move(fsm_event_callback))
{
  configure_(node);
  make_filter_(node);
  add_updaters_(node);
  initialize_filter_(node);
  updater_interfaces_.create(node);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::reset()
{
  filter_->reset();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::configure_(std::shared_ptr<rclcpp::Node> node)
{
  declare_filter_parameters<FilterType_>(node);
  declare_predictor_parameters(node, 2.0, 10.0);
  declare_proprioceptive_updater_parameters(node, "twist_updater", 10);
  declare_proprioceptive_updater_parameters(node, "linear_speed_updater", 0);
  declare_proprioceptive_updater_parameters(node, "linear_speeds_updater", 0);
  declare_proprioceptive_updater_parameters(node, "angular_speed_updater", 0);
  declare_proprioceptive_updater_parameters(node, "attitude_updater", 10);
  declare_exteroceptive_updater_parameters(node, "position_updater", 1, "always");
  declare_exteroceptive_updater_parameters(node, "course_updater", 1, "once");
  declare_exteroceptive_updater_parameters(node, "range_updater", 0, "always");
  declare_exteroceptive_updater_parameters(node, "pose_updater", 0, "always");
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::make_filter_(std::shared_ptr<rclcpp::Node> node)
{
  if constexpr (FilterType_ == core::FilterType::KALMAN) {
    filter_ = std::make_unique<Filter>(
      get_filter_state_pool_size(node), get_dead_reckoning_limits(node));
  } else {
    filter_ = std::make_unique<Filter>(
      get_filter_state_pool_size(node),
      get_filter_number_of_particles(node),
      get_dead_reckoning_limits(node));
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "filter: started ");
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::add_updaters_(std::shared_ptr<rclcpp::Node> node)
{
  add_updater_<UpdaterInterfaceTwist>(node, "twist_updater", "twist");
  add_updater_<UpdaterInterfaceLinearSpeed>(node, "linear_speed_updater", "twist");
  add_updater_<UpdaterInterfaceLinearSpeeds>(node, "linear_speeds_updater", "twist");
  add_updater_<UpdaterInterfaceAngularSpeed>(node, "angular_speed_updater", "angular_speed");
  add_updater_<UpdaterInterfaceAttitude>(node, "attitude_updater", "attitude");
  add_updater_<UpdaterInterfacePosition>(node, "position_updater", "position");
  add_updater_<UpdaterInterfaceCourse>(node, "course_updater", "course");
  add_updater_<UpdaterInterfaceRange>(node, "range_updater", "range");
  add_updater_<UpdaterInterfacePose>(node, "pose_updater", "pose");
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::initialize_filter_(std::shared_ptr<rclcpp::Node> node)
{
  const auto event_log_callback = [this](const std::string & message) {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  };

  const auto get_results_log_callback = [this, node](const std::string & message) {
    RCLCPP_INFO_THROTTLE(logger_, *node->get_clock(), 1000, "%s", message.c_str());
  };

  filter_->initialize(
    event_log_callback,
    get_results_log_callback,
    fsm_event_callback_,
    make_loggers_(node));
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
typename R2WLocalisationFilter<FilterType_>::Filter::LoggerMap
R2WLocalisationFilter<FilterType_>::make_loggers_(std::shared_ptr<rclcpp::Node> node)
{
  if (!get_debug(node)) {
    return {};
  }

  return {
    {"predictor", make_topic_logger(node, "predictor")},
    {"position_updater", make_topic_logger(node, "position_updater")},
    {"course_updater", make_topic_logger(node, "course_updater")},
    {"range_updater", make_topic_logger(node, "range_updater")},
    {"pose_updater", make_topic_logger(node, "pose_updater")}};
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_updater_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name)
{
  if (get_updater_minimal_rate(node, updater_name) != 0) {
    using Updater = typename Interface::Updater;
    auto callback = [&]() {
      if constexpr (core::localisation::is_proprioceptive_updater<Updater>()) {
        return filter_->template add_proprioceptive_updater<Updater>(
          updater_name, get_updater_minimal_rate(node, updater_name));
      } else {
        return filter_->template add_exteroceptive_updater<Updater>(
          updater_name,
          get_updater_minimal_rate(node, updater_name),
          core::localisation::to_trigger_mode(get_updater_trigger_mode(node, updater_name)),
          get_updater_mahalanobis_distance_rejection_threshold(node, updater_name));
      }
    }();

    if (!callback) {
      return;
    }

    updater_interfaces_.add<Interface>(updater_name, topic_name, std::move(*callback));
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
typename R2WLocalisationFilter<FilterType_>::Query
R2WLocalisationFilter<FilterType_>::get_results(const core::Duration & duration)
{
  return filter_->get_results(duration);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
core::DiagnosticReport R2WLocalisationFilter<FilterType_>::make_diagnostic_report(
  const core::Duration & stamp,
  const core::localisation::FSMState & fsm_state)
{
  return filter_->make_diagnostic_report(stamp, fsm_state);
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
