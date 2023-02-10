
/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 * @brief Odri Joint Controller Plugin
 *
 * This plugin simulates the odri controller on a joint
 *
 * @author Hugo Duarte <hduarte@iri.upc.edu>
 */

#ifndef ODRI_JOINT_CONTROLLER_HH_
#define ODRI_JOINT_CONTROLLER_HH_

#include <ignition/gazebo/System.hh>
#include <memory>

namespace odri_joint_controller
{
  // Forward declaration
  class OdriJointControllerPrivate;

  /// \brief Odri joint controller which can be attached to a model with a reference
  /// to a single joint.
  ///
  /// ## System Parameters
  ///
  /// `<joint_name>` The name of the joint to control. Required parameter.
  ///  Can also include multiple `<joint_name>` for identical joints.
  ///
  /// `<joint_index>` Axis of the joint to control. Optional parameter.
  ///  The default value is 0.
  ///
  /// `<topic>` Topic to receive commands in. Defaults to
  ///     `/model/<model_name>/joint/<joint_name>/cmd_vel`.
  ///
  class IGNITION_GAZEBO_VISIBLE OdriJointController : public ignition::gazebo::System,
                                                      public ignition::gazebo::ISystemConfigure,
                                                      public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
  public:
    OdriJointController();

    /// \brief Destructor
  public:
    ~OdriJointController() override = default;

    // Documentation inherited
  public:
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
  public:
    void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
  private:
    std::unique_ptr<OdriJointControllerPrivate> dataPtr;
  };
}

#endif // ODRI_JOINT_CONTROLLER_HH_