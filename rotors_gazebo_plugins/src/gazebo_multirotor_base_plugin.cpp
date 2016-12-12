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

#include "rotors_gazebo_plugins/gazebo_multirotor_base_plugin.h"

#include <ctime>

//#include <mav_msgs/Actuators.h>

namespace gazebo {

GazeboMultirotorBasePlugin::~GazeboMultirotorBasePlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

void GazeboMultirotorBasePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_, true);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);
  getSdfParam<std::string>(_sdf, "motorPubTopic", motor_pub_topic_, motor_pub_topic_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_,
                      rotor_velocity_slowdown_sim_);

  //node_handle_ = new ros::NodeHandle(namespace_);
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  //motor_pub_ = node_handle_->advertise<mav_msgs::Actuators>(motor_pub_topic_, 10);
  motor_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Actuators>(motor_pub_topic_, 10);
  gzmsg << "motor_pub_topic_ = \"" << motor_pub_topic_ << "\"." << std::endl;

  //joint_state_pub_ = node_handle_->advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 1);
  joint_state_pub_ = node_handle_->Advertise<sensor_msgs::msgs::JointState>(joint_state_pub_topic_, 1);
  gzmsg << "joint_state_pub_topic = \"" << joint_state_pub_topic_ << "\"." << std::endl;
  frame_id_ = link_name_;

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_multirotor_base_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMultirotorBasePlugin::OnUpdate, this, _1));

  child_links_ = link_->GetChildJointsLinks();
  for (unsigned int i = 0; i < child_links_.size(); i++) {
    std::string link_name = child_links_[i]->GetScopedName();

    // Check if link contains rotor_ in its name.
    int pos = link_name.find("rotor_");
    if (pos != link_name.npos) {
      std::string motor_number_str = link_name.substr(pos + 6);
      unsigned int motor_number = std::stoi(motor_number_str);
      std::string joint_name = child_links_[i]->GetName() + "_joint";
      physics::JointPtr joint = this->model_->GetJoint(joint_name);
      motor_joints_.insert(MotorNumberToJointPair(motor_number, joint));
    }
  }
}

// This gets called by the world update start event.
void GazeboMultirotorBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  msg.mutable_header()->mutable_stamp()->set_sec(now.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  msg.mutable_header()->set_frame_id(frame_id_);

  joint_state_msg.mutable_header()->mutable_stamp()->set_sec(now.sec);
  joint_state_msg.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  joint_state_msg.mutable_header()->set_frame_id(frame_id_);

  //mav_msgs::ActuatorsPtr msg(new mav_msgs::Actuators);

  //msg->angular_velocities.resize(motor_joints_.size());
  msg.clear_angular_velocities();

  //sensor_msgs::JointStatePtr joint_state_msg(new sensor_msgs::JointState);

//  joint_state_msg->name.resize(motor_joints_.size());
//  joint_state_msg->position.resize(motor_joints_.size());
  joint_state_msg.clear_name();
  joint_state_msg.clear_position();

  MotorNumberToJointMap::iterator m;
  for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
    double motor_rot_vel = m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;

    //msg->angular_velocities[m->first] = motor_rot_vel;
    msg.add_angular_velocities(motor_rot_vel);

//    joint_state_msg->name[m->first] = m->second->GetName();
//    joint_state_msg->position[m->first] = m->second->GetAngle(0).Radian();
    joint_state_msg.add_name(m->second->GetName());
    joint_state_msg.add_position(m->second->GetAngle(0).Radian());

  }

  joint_state_pub_->Publish(joint_state_msg);
  motor_pub_->Publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMultirotorBasePlugin);

} // namespace gazebo
