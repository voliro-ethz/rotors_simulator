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

#ifndef ROTORS_CONTROL_VOLIRO_CONTROLLER_H
#define ROTORS_CONTROL_VOLIRO_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the voliro position controller and the Asctec Firefly.
// Proportional part of PID
static const Eigen::Vector3d kDefaultPositionKp = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityKp = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeKp = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateKp = Eigen::Vector3d(0.52, 0.52, 0.025);
//Integrational part of PID
static const Eigen::Vector3d kDefaultPositionKi = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultVelocityKi = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultAttitudeKi = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultAngularRateKi = Eigen::Vector3d(0, 0, 0);
//derivative part of PID
static const Eigen::Vector3d kDefaultPositionKd = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultVelocityKd = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultAttitudeKd = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultAngularRateKd = Eigen::Vector3d(0, 0, 0);

class VoliroControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoliroControllerParameters()
      : position_kp_(kDefaultPositionKp),
        velocity_kp_(kDefaultVelocityKp),
        attitude_kp_(kDefaultAttitudeKp),
        angular_rate_kp_(kDefaultAngularRateKp),
        position_ki_(kDefaultPositionKi),
        velocity_ki_(kDefaultVelocityKi),
        attitude_ki_(kDefaultAttitudeKi),
        angular_rate_ki_(kDefaultAngularRateKi),
        position_kd_(kDefaultPositionKd),
        velocity_kd_(kDefaultVelocityKd),
        attitude_kd_(kDefaultAttitudeKd),
        angular_rate_kd_(kDefaultAngularRateKd) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_kp_;
  Eigen::Vector3d velocity_kp_;
  Eigen::Vector3d attitude_kp_;
  Eigen::Vector3d angular_rate_kp_;
  Eigen::Vector3d position_ki_;
  Eigen::Vector3d velocity_ki_;
  Eigen::Vector3d attitude_ki_;
  Eigen::Vector3d angular_rate_ki_;
  Eigen::Vector3d position_kd_;
  Eigen::Vector3d velocity_kd_;
  Eigen::Vector3d attitude_kd_;
  Eigen::Vector3d angular_rate_kd_;
  RotorConfiguration rotor_configuration_;
};

class VoliroController {
 public:
  VoliroController();
  ~VoliroController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  void SetOdometry(const EigenOdometry& odometry);
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  VoliroControllerParameters controller_parameters_;  //definded below in this file
  VehicleParameters vehicle_parameters_;  //incudesrotorconfiguration

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
};
}

#endif // ROTORS_CONTROL_VOLIRO_CONTROLLER_H
