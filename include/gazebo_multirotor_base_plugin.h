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

#ifndef MULTIROTOR_BASE_PLUGIN_HH_
#define MULTIROTOR_BASE_PLUGIN_HH_

#include <string>

#include <MotorSpeed.pb.h>

#include <ignition/transport/Node.hh>
#include <ignition/math.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Name.hh>

#include <common.h>

namespace multirotor_base_plugin
{

  // Default values
  static constexpr auto kDefaultMotorPubTopic = "/motors";
  static constexpr auto kDefaultLinkName = "base_link";
  static constexpr auto kDefaultFrameId = "base_link";

  static constexpr auto kDefaultRotorVelocitySlowdownSim = 10.0;

  /// \brief This plugin publishes the motor speeds of your multirotor model.
  class IGNITION_GAZEBO_VISIBLE MultirotorBasePlugin :
      // This is class a system.
      public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
  {
  typedef std::map<const unsigned int, const ignition::gazebo::Entity> MotorNumberToJointMap;
  public:
    MultirotorBasePlugin();

  public:
    ~MultirotorBasePlugin() override;

  public:
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager & /*_eventMgr*/);

  public:
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

  public:
    void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) override;

  private:
    void getSdfParams(const std::shared_ptr<const sdf::Element> &sdf);
    /// \brief Pointer to the update event connection.
    ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

    MotorNumberToJointMap motor_joints_;

    std::string namespace_;
    std::string motor_pub_topic_;
    std::string link_name_;
    std::string frame_id_;
    double rotor_velocity_slowdown_sim_;

    ignition::transport::Node node;
    ignition::transport::Node::Publisher motor_pub_;
  };
}

#endif // MULTIROTOR_BASE_PLUGIN_HH_