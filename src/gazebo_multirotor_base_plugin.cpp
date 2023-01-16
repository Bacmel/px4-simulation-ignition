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

#include "gazebo_multirotor_base_plugin.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    multirotor_base_plugin::MultirotorBasePlugin,
    ignition::gazebo::System,
    multirotor_base_plugin::MultirotorBasePlugin::ISystemConfigure,
    multirotor_base_plugin::MultirotorBasePlugin::ISystemPreUpdate,
    multirotor_base_plugin::MultirotorBasePlugin::ISystemPostUpdate)

using namespace multirotor_base_plugin;

MultirotorBasePlugin::MultirotorBasePlugin()
{
}

MultirotorBasePlugin::~MultirotorBasePlugin()
{
}

void MultirotorBasePlugin::getSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{
  if (sdf->HasElement("motorPubTopic"))
  {
    motor_pub_topic_ = sdf->Get<std::string>("motorPubTopic");
  }
  else
  {
    motor_pub_topic_ = kDefaultMotorPubTopic;
    ignwarn << "[multirotor_base_plugin] Using default topic name of " << motor_pub_topic_ << "\n";
  }

  if (sdf->HasElement("rotorVelocitySlowdownSim"))
  {
    rotor_velocity_slowdown_sim_ = sdf->Get<double>("rotorVelocitySlowdownSim");
  }
  else
  {
    rotor_velocity_slowdown_sim_ = kDefaultRotorVelocitySlowdownSim;
    ignwarn << "[multirotor_base_plugin] Using default velocity slowdown of " << rotor_velocity_slowdown_sim_ << " Hz\n";
  }

  link_name_ = sdf->Get<std::string>("link_name");
}

void MultirotorBasePlugin::Configure(const ignition::gazebo::Entity &_entity,
                                     const std::shared_ptr<const sdf::Element> &_sdf,
                                     ignition::gazebo::EntityComponentManager &_ecm,
                                     ignition::gazebo::EventManager & /*_eventMgr*/)
{
  getSdfParams(_sdf);

  model_ = ignition::gazebo::Model(_entity);
  std::string model_name_ = model_.Name(_ecm);

  // Get link entity
  model_link_ = model_.LinkByName(_ecm, link_name_);

  std::vector<ignition::gazebo::Entity> child_links = model_.Links(_ecm);
  for (auto child = child_links.begin(); child != child_links.end(); ++child)
  {
    std::string link_name = ignition::gazebo::Model(*child).Name(_ecm);

    // Check if link contains rotor_ in its name.
    int pos = link_name.find("rotor_");
    if (pos != link_name.npos)
    {
      std::string motor_number_str = link_name.substr(pos + 6);
      unsigned int motor_number = std::stoi(motor_number_str);
      std::string joint_name = link_name + "_joint";
      ignition::gazebo::Entity joint = model_.JointByName(_ecm, joint_name);
      motor_joints_.insert(std::pair<unsigned int, ignition::gazebo::Entity>(motor_number, joint));
    }
  }
}

void MultirotorBasePlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                     ignition::gazebo::EntityComponentManager &_ecm)
{
}

void MultirotorBasePlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                      const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Get the current simulation time.
  const std::chrono::steady_clock::duration current_time = _info.simTime;

  mav_msgs::msgs::MotorSpeed msg;
  MotorNumberToJointMap::iterator m;
  for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m)
  {
    const std::vector<double> joint_velocity = _ecm.Component<ignition::gazebo::components::JointVelocity>(m->second)->Data();
    double motor_rot_vel = joint_velocity.at(0) * rotor_velocity_slowdown_sim_;
    msg.add_motor_speed(motor_rot_vel);
  }
  // motor_pub_->WaitForConnection();
  // Add time header
  // FIXME: Commented out to prevent warnings about queue limit reached.
  // motor_pub_->Publish(msg);
}