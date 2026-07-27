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
#include <cassert>
#include <limits>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>

// romea
#include "romea_core_localisation/robot_to_world/traits.hpp"
#include "romea_localisation_utils/filter/factory.hpp"
#include "romea_localisation_utils/filter/updater_interface.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

template<core::FilterType FilterType_>
class R2WLocalisationFilter
{
public:
  using Filter = typename core::localisation::R2WTraits<FilterType_>::Filter;
  using MetaState = typename core::localisation::R2WTraits<FilterType_>::MetaState;
  using Predictor = typename core::localisation::R2WTraits<FilterType_>::Predictor;
  using UpdaterTwist = typename core::localisation::R2WTraits<FilterType_>::UpdaterTwist;
  using UpdaterLinearSpeed =
    typename core::localisation::R2WTraits<FilterType_>::UpdaterLinearSpeed;
  using UpdaterLinearSpeeds =
    typename core::localisation::R2WTraits<FilterType_>::UpdaterLinearSpeeds;
  using UpdaterAngularSpeed =
    typename core::localisation::R2WTraits<FilterType_>::UpdaterAngularSpeed;
  using UpdaterAttitude = typename core::localisation::R2WTraits<FilterType_>::UpdaterAttitude;
  using UpdaterCourse = typename core::localisation::R2WTraits<FilterType_>::UpdaterCourse;
  using UpdaterRange = typename core::localisation::R2WTraits<FilterType_>::UpdaterRange;
  using UpdaterPose = typename core::localisation::R2WTraits<FilterType_>::UpdaterPose;
  using UpdaterPosition = typename core::localisation::R2WTraits<FilterType_>::UpdaterPosition;
  using Results = typename core::localisation::R2WTraits<FilterType_>::Results;
  using MetaStateToResults =
    typename core::localisation::R2WTraits<FilterType_>::MetaStateToResults;

  using UpdaterInterfaceTwist =
    UpdaterInterface<Filter, UpdaterTwist, romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeed = UpdaterInterface<
    Filter,
    UpdaterLinearSpeed,
    romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeeds = UpdaterInterface<
    Filter,
    UpdaterLinearSpeeds,
    romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceAngularSpeed = UpdaterInterface<
    Filter,
    UpdaterAngularSpeed,
    romea_localisation_msgs::msg::ObservationAngularSpeedStamped>;
  using UpdaterInterfaceAttitude = UpdaterInterface<
    Filter,
    UpdaterAttitude,
    romea_localisation_msgs::msg::ObservationAttitudeStamped>;
  using UpdaterInterfaceCourse =
    UpdaterInterface<Filter, UpdaterCourse, romea_localisation_msgs::msg::ObservationCourseStamped>;
  using UpdaterInterfacePosition = UpdaterInterface<
    Filter,
    UpdaterPosition,
    romea_localisation_msgs::msg::ObservationPosition2DStamped>;
  using UpdaterInterfaceRange =
    UpdaterInterface<Filter, UpdaterRange, romea_localisation_msgs::msg::ObservationRangeStamped>;
  using UpdaterInterfacePose =
    UpdaterInterface<Filter, UpdaterPose, romea_localisation_msgs::msg::ObservationPose2DStamped>;

public:
  explicit R2WLocalisationFilter(
    std::shared_ptr<rclcpp::Node> node, core::FSMEventCallback fsm_event_callback = nullptr);

  void reset();

public:
  core::localisation::FSMState get_fsm_state();

  std::optional<Results> get_results(const core::Duration & duration);

  core::DiagnosticReport make_diagnostic_report(const core::Duration & duration);

private:
  void configure_(std::shared_ptr<rclcpp::Node> node);

  void make_filter_(std::shared_ptr<rclcpp::Node> node);

  void add_updater_interfaces_(std::shared_ptr<rclcpp::Node> node);

  void make_meta_state_to_results_(std::shared_ptr<rclcpp::Node> node);

  void downgrade_exteroceptive_diagnostics_(
    core::DiagnosticReport & report,
    const core::localisation::FSMState & fsm_state);

  template<typename Interface>
  void add_proprioceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name);

  template<typename interface>
  void add_exteroceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name);

private:
  rclcpp::Logger logger_;
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<MetaState> current_meta_state_;
  std::unique_ptr<MetaStateToResults> meta_state_to_results_;
  std::list<std::unique_ptr<UpdaterInterfaceBase>> proprioceptive_updater_interfaces_;
  std::list<std::unique_ptr<UpdaterInterfaceBase>> exteroceptive_updater_interfaces_;
  core::FSMEventCallback fsm_event_callback_;
};

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
R2WLocalisationFilter<FilterType_>::R2WLocalisationFilter(
  std::shared_ptr<rclcpp::Node> node, core::FSMEventCallback fsm_event_callback)
: logger_(node->get_logger()),
  filter_(nullptr),
  current_meta_state_(nullptr),
  meta_state_to_results_(nullptr),
  proprioceptive_updater_interfaces_(),
  exteroceptive_updater_interfaces_(),
  fsm_event_callback_(std::move(fsm_event_callback))
{
  configure_(node);
  make_filter_(node);
  add_updater_interfaces_(node);
  make_meta_state_to_results_(node);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
core::localisation::FSMState R2WLocalisationFilter<FilterType_>::get_fsm_state()
{
  return filter_->get_fsm_state();
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
  typename Predictor::ObservationAgeLimits observation_age_limits;
  UpdaterTwist::update_observation_age_limits(
    observation_age_limits, get_updater_minimal_rate(node, "twist_updater"));
  UpdaterLinearSpeed::update_observation_age_limits(
    observation_age_limits, get_updater_minimal_rate(node, "linear_speed_updater"));
  UpdaterLinearSpeeds::update_observation_age_limits(
    observation_age_limits, get_updater_minimal_rate(node, "linear_speeds_updater"));
  UpdaterAngularSpeed::update_observation_age_limits(
    observation_age_limits, get_updater_minimal_rate(node, "angular_speed_updater"));

  filter_ =
    make_filter<Filter, Predictor, FilterType_>(node, fsm_event_callback_, observation_age_limits);
  RCLCPP_INFO_STREAM(node->get_logger(), "filter: started ");
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::add_updater_interfaces_(std::shared_ptr<rclcpp::Node> node)
{
  add_proprioceptive_updater_interface_<UpdaterInterfaceTwist>(node, "twist_updater", "twist");
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeed>(
    node, "linear_speed_updater", "linear_speed");
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeeds>(
    node, "linear_speeds_updater", "linear_speeds");
  add_proprioceptive_updater_interface_<UpdaterInterfaceAngularSpeed>(
    node, "angular_speed_updater", "angular_speed");
  add_proprioceptive_updater_interface_<UpdaterInterfaceAttitude>(
    node, "attitude_updater", "attitude");
  add_exteroceptive_updater_interface_<UpdaterInterfacePosition>(
    node, "position_updater", "position");
  add_exteroceptive_updater_interface_<UpdaterInterfaceCourse>(node, "course_updater", "course");
  add_exteroceptive_updater_interface_<UpdaterInterfaceRange>(node, "range_updater", "range");
  add_exteroceptive_updater_interface_<UpdaterInterfacePose>(node, "pose_updater", "pose");
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::make_meta_state_to_results_(
  std::shared_ptr<rclcpp::Node> node)
{
  current_meta_state_ = make_meta_state<MetaState, FilterType_>(node);
  meta_state_to_results_ = make_meta_state_to_results<MetaStateToResults, FilterType_>(node);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_proprioceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name)
{
  if (get_updater_minimal_rate(node, updater_name) != 0) {
    using Updater = typename Interface::Updater;
    auto updater = make_proprioceptive_updater<Updater>(node, updater_name);

    auto plugin = make_updater_interface<Interface>(node, topic_name, filter_, std::move(updater));

    proprioceptive_updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
template<typename Interface>
void R2WLocalisationFilter<FilterType_>::add_exteroceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name)
{
  if (get_updater_minimal_rate(node, updater_name) != 0) {
    using Updater = typename Interface::Updater;
    auto updater = make_exteroceptive_updater<Updater, FilterType_>(
      node, updater_name, make_topic_logger(node, updater_name), fsm_event_callback_);
    auto plugin = make_updater_interface<Interface>(node, topic_name, filter_, std::move(updater));

    exteroceptive_updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name << ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
std::optional<typename R2WLocalisationFilter<FilterType_>::Results>
R2WLocalisationFilter<FilterType_>::get_results(const core::Duration & duration)
{
  const auto status = filter_->get_state(duration, current_meta_state_.get());
  switch (status) {
    case core::FilterGetStateStatus::AVAILABLE:
      return meta_state_to_results_->convert(*current_meta_state_);
    case core::FilterGetStateStatus::EMPTY:
      RCLCPP_INFO(logger_, "cannot get results because filter has no state yet");
      return std::nullopt;
    case core::FilterGetStateStatus::TOO_OLD:
      RCLCPP_INFO(logger_, "cannot get results because requested state is too old");
      return std::nullopt;
    case core::FilterGetStateStatus::TOO_FAR:
      RCLCPP_INFO(logger_, "cannot get results because requested state is too far");
      return std::nullopt;
    default:
      assert(false);
      return std::nullopt;
  }
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
core::DiagnosticReport R2WLocalisationFilter<FilterType_>::make_diagnostic_report(
  const core::Duration & stamp)
{
  const auto fsm_state = get_fsm_state();
  core::DiagnosticReport report;
  for (auto & interface_ptr : proprioceptive_updater_interfaces_) {
    interface_ptr->heartbeat_callback(stamp);
    report += interface_ptr->get_report();
  }

  for (auto & interface_ptr : exteroceptive_updater_interfaces_) {
    interface_ptr->heartbeat_callback(stamp);
    auto exteroceptive_report = interface_ptr->get_report();
    downgrade_exteroceptive_diagnostics_(exteroceptive_report, fsm_state);
    report += exteroceptive_report;
  }

  return report;
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WLocalisationFilter<FilterType_>::downgrade_exteroceptive_diagnostics_(
  core::DiagnosticReport & report,
  const core::localisation::FSMState & fsm_state)
{
  if (fsm_state != core::localisation::FSMState::RUNNING) {
    return;
  }

  for (auto & diagnostic : report.diagnostics) {
    if (
      diagnostic.status != core::DiagnosticStatus::OK &&
      diagnostic.status != core::DiagnosticStatus::WARN)
    {
      diagnostic.status = core::DiagnosticStatus::WARN;
    }
  }
}

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION_CORE__ROBOT_TO_WORLD_LOCALISATION_FILTER_HPP_
