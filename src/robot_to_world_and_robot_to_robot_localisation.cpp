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

// local
#include "romea_robot_to_world_localisation_core/robot_to_world_and_robot_to_robot_localisation.hpp"

namespace romea
{
namespace ros2
{
namespace localisation
{

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
R2WR2RLocalisation<FilterType_>::R2WR2RLocalisation(const rclcpp::NodeOptions & options)
: R2WLocalisation<FilterType_>(options)
{
  make_leader_odom_sub_();
  make_leader_pose_and_twist_pub_();
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WR2RLocalisation<FilterType_>::make_leader_pose_and_twist_pub_()
{
  leader_pose_and_twist_publisher_ = make_stamped_data_publisher<core::PoseAndTwist2D,
      romea_common_msgs::msg::PoseAndTwist2DStamped>(
    this->node_,
    "filtered_leader_pose",
    get_base_footprint_frame_id(this->node_),
    reliable(1),
    true);
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WR2RLocalisation<FilterType_>::make_leader_odom_sub_()
{
  auto callback =
    std::bind(&R2WR2RLocalisation::leader_odom_callback_, this, std::placeholders::_1);

  leader_odom_sub_ = this->node_->template create_subscription<OdometryMsg>(
    "leader_filtered_odom", best_effort(10), callback);

  std::cout << "leader_odom_sub_->get_topic_name() " << leader_odom_sub_->get_topic_name() <<
    std::endl;
}

//-----------------------------------------------------------------------------
template<core::FilterType FilterType_>
void R2WR2RLocalisation<FilterType_>::leader_odom_callback_(OdometryMsg::ConstSharedPtr msg)
{
  auto fsm_state = this->filter_->get_fsm_state();
  if (fsm_state == core::localisation::FSMState::RUNNING) {
    const auto & results = this->filter_->get_results(extract_duration(*msg));

    core::Pose2D follower_pose = results.to_pose2d();
    core::Pose2D leader_pose = core::toPose2D(to_romea(msg->pose));

    core::PoseAndTwist2D follower_to_leader_pose;
    follower_to_leader_pose.pose.yaw =
      core::betweenMinusPiAndPi(leader_pose.yaw - follower_pose.yaw);

    follower_to_leader_pose.pose.position =
      core::eulerAngleToRotation2D(-follower_pose.yaw) *
      (leader_pose.position - follower_pose.position);

    // TOTO(jean) add covariance
    follower_to_leader_pose.twist = core::toTwist2D(to_romea(msg->twist));

    leader_pose_and_twist_publisher_->publish(msg->header.stamp, follower_to_leader_pose);
  }
}

template class R2WR2RLocalisation<core::FilterType::KALMAN>;
template class R2WR2RLocalisation<core::FilterType::PARTICLE>;

}  // namespace localisation
}  // namespace ros2
}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::localisation::R2WR2RKalmanLocalisation)
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::localisation::R2WR2RParticleLocalisation)
