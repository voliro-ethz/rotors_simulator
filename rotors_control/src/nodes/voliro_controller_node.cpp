/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "voliro_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

VoliroControllerNode::VoliroControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &VoliroControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &VoliroControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &VoliroControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &VoliroControllerNode::TimedCommandCallback, this,
                                  true, false);
}

VoliroControllerNode::~VoliroControllerNode() { }

//Load the parameters for the controller into the member voliro controller
void VoliroControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_kp/x",
                  voliro_controller_.controller_parameters_.position_kp_.x(),
                  &voliro_controller_.controller_parameters_.position_kp_.x());
  GetRosParameter(pnh, "position_kp/y",
                  voliro_controller_.controller_parameters_.position_kp_.y(),
                  &voliro_controller_.controller_parameters_.position_kp_.y());
  GetRosParameter(pnh, "position_kp/z",
                  voliro_controller_.controller_parameters_.position_kp_.z(),
                  &voliro_controller_.controller_parameters_.position_kp_.z());
  GetRosParameter(pnh, "velocity_kp/x",
                  voliro_controller_.controller_parameters_.velocity_kp_.x(),
                  &voliro_controller_.controller_parameters_.velocity_kp_.x());
  GetRosParameter(pnh, "velocity_kp/y",
                  voliro_controller_.controller_parameters_.velocity_kp_.y(),
                  &voliro_controller_.controller_parameters_.velocity_kp_.y());
  GetRosParameter(pnh, "velocity_kp/z",
                  voliro_controller_.controller_parameters_.velocity_kp_.z(),
                  &voliro_controller_.controller_parameters_.velocity_kp_.z());
  GetRosParameter(pnh, "attitude_kp/x",
                  voliro_controller_.controller_parameters_.attitude_kp_.x(),
                  &voliro_controller_.controller_parameters_.attitude_kp_.x());
  GetRosParameter(pnh, "attitude_kp/y",
                  voliro_controller_.controller_parameters_.attitude_kp_.y(),
                  &voliro_controller_.controller_parameters_.attitude_kp_.y());
  GetRosParameter(pnh, "attitude_kp/z",
                  voliro_controller_.controller_parameters_.attitude_kp_.z(),
                  &voliro_controller_.controller_parameters_.attitude_kp_.z());
  GetRosParameter(pnh, "angular_rate_kp/x",
                  voliro_controller_.controller_parameters_.angular_rate_kp_.x(),
                  &voliro_controller_.controller_parameters_.angular_rate_kp_.x());
  GetRosParameter(pnh, "angular_rate_kp/y",
                  voliro_controller_.controller_parameters_.angular_rate_kp_.y(),
                  &voliro_controller_.controller_parameters_.angular_rate_kp_.y());
  GetRosParameter(pnh, "angular_rate_kp/z",
                  voliro_controller_.controller_parameters_.angular_rate_kp_.z(),
                  &voliro_controller_.controller_parameters_.angular_rate_kp_.z());
  GetRosParameter(pnh, "position_ki/x",
                  voliro_controller_.controller_parameters_.position_ki_.x(),
                  &voliro_controller_.controller_parameters_.position_ki_.x());
  GetRosParameter(pnh, "position_ki/y",
                  voliro_controller_.controller_parameters_.position_ki_.y(),
                  &voliro_controller_.controller_parameters_.position_ki_.y());
  GetRosParameter(pnh, "position_ki/z",
                  voliro_controller_.controller_parameters_.position_ki_.z(),
                  &voliro_controller_.controller_parameters_.position_ki_.z());
  GetRosParameter(pnh, "velocity_ki/x",
                  voliro_controller_.controller_parameters_.velocity_ki_.x(),
                  &voliro_controller_.controller_parameters_.velocity_ki_.x());
  GetRosParameter(pnh, "velocity_ki/y",
                  voliro_controller_.controller_parameters_.velocity_ki_.y(),
                  &voliro_controller_.controller_parameters_.velocity_ki_.y());
  GetRosParameter(pnh, "velocity_ki/z",
                  voliro_controller_.controller_parameters_.velocity_ki_.z(),
                  &voliro_controller_.controller_parameters_.velocity_ki_.z());
  GetRosParameter(pnh, "attitude_ki/x",
                  voliro_controller_.controller_parameters_.attitude_ki_.x(),
                  &voliro_controller_.controller_parameters_.attitude_ki_.x());
  GetRosParameter(pnh, "attitude_ki/y",
                  voliro_controller_.controller_parameters_.attitude_ki_.y(),
                  &voliro_controller_.controller_parameters_.attitude_ki_.y());
  GetRosParameter(pnh, "attitude_ki/z",
                  voliro_controller_.controller_parameters_.attitude_ki_.z(),
                  &voliro_controller_.controller_parameters_.attitude_ki_.z());
  GetRosParameter(pnh, "angular_rate_ki/x",
                  voliro_controller_.controller_parameters_.angular_rate_ki_.x(),
                  &voliro_controller_.controller_parameters_.angular_rate_ki_.x());
  GetRosParameter(pnh, "angular_rate_ki/y",
                  voliro_controller_.controller_parameters_.angular_rate_ki_.y(),
                  &voliro_controller_.controller_parameters_.angular_rate_ki_.y());
  GetRosParameter(pnh, "angular_rate_ki/z",
                  voliro_controller_.controller_parameters_.angular_rate_ki_.z(),
                  &voliro_controller_.controller_parameters_.angular_rate_ki_.z());
  GetRosParameter(pnh, "position_kd/x",
                  voliro_controller_.controller_parameters_.position_kd_.x(),
                  &voliro_controller_.controller_parameters_.position_kd_.x());
  GetRosParameter(pnh, "position_kd/y",
                  voliro_controller_.controller_parameters_.position_kd_.y(),
                  &voliro_controller_.controller_parameters_.position_kd_.y());
  GetRosParameter(pnh, "position_kd/z",
                  voliro_controller_.controller_parameters_.position_kd_.z(),
                  &voliro_controller_.controller_parameters_.position_kd_.z());
  GetRosParameter(pnh, "velocity_kd/x",
                  voliro_controller_.controller_parameters_.velocity_kd_.x(),
                  &voliro_controller_.controller_parameters_.velocity_kd_.x());
  GetRosParameter(pnh, "velocity_kd/y",
                  voliro_controller_.controller_parameters_.velocity_kd_.y(),
                  &voliro_controller_.controller_parameters_.velocity_kd_.y());
  GetRosParameter(pnh, "velocity_kd/z",
                  voliro_controller_.controller_parameters_.velocity_kd_.z(),
                  &voliro_controller_.controller_parameters_.velocity_kd_.z());
  GetRosParameter(pnh, "attitude_kd/x",
                  voliro_controller_.controller_parameters_.attitude_kd_.x(),
                  &voliro_controller_.controller_parameters_.attitude_kd_.x());
  GetRosParameter(pnh, "attitude_kd/y",
                  voliro_controller_.controller_parameters_.attitude_kd_.y(),
                  &voliro_controller_.controller_parameters_.attitude_kd_.y());
  GetRosParameter(pnh, "attitude_kd/z",
                  voliro_controller_.controller_parameters_.attitude_kd_.z(),
                  &voliro_controller_.controller_parameters_.attitude_kd_.z());
  GetRosParameter(pnh, "angular_rate_kd/x",
                  voliro_controller_.controller_parameters_.angular_rate_kd_.x(),
                  &voliro_controller_.controller_parameters_.angular_rate_kd_.x());
  GetRosParameter(pnh, "angular_rate_kd/y",
                  voliro_controller_.controller_parameters_.angular_rate_kd_.y(),
                  &voliro_controller_.controller_parameters_.angular_rate_kd_.y());
  GetRosParameter(pnh, "angular_rate_kd/z",
                  voliro_controller_.controller_parameters_.angular_rate_kd_.z(),
                  &voliro_controller_.controller_parameters_.angular_rate_kd_.z());
  GetVehicleParameters(pnh, &voliro_controller_.vehicle_parameters_);
  voliro_controller_.InitializeParameters();
}
void VoliroControllerNode::Publish() {
}

void VoliroControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  voliro_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void VoliroControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  voliro_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}
//nicht interessant für uns
void VoliroControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  voliro_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

// Heres is the action happening. This is the important callback function ------------------------------------
void VoliroControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("VoliroController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  voliro_controller_.SetOdometry(odometry);  // Load the actual odometry in the member of voliro_controller_

  Eigen::VectorXd ref_rotor_velocities;
  voliro_controller_.CalculateRotorVelocities(&ref_rotor_velocities); //Do the control and allocation and give rotorvelocities

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  //Add here also the angles for the actuators in the same actuator msg
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "voliro_controller_node");

  rotors_control::VoliroControllerNode voliro_controller_node;

  ros::spin();

  return 0;
}
