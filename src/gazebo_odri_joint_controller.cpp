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

#include "gazebo_odri_joint_controller.h"

#include <OdriCmd.pb.h>
#include <OdriState.pb.h>

#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/Model.hh"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    odri_joint_controller::OdriJointController,
    ignition::gazebo::System,
    odri_joint_controller::OdriJointController::ISystemConfigure,
    odri_joint_controller::OdriJointController::ISystemPreUpdate)

using namespace odri_joint_controller;

class odri_joint_controller::OdriJointControllerPrivate
{
  /// \brief Callback for odri subscription
  /// \param[in] _msg Odri message
public:
  void OnOdriCmd(const cmd_msgs::msgs::OdriCmd &_msg);

  /// \brief Gazebo communication node.
public:
  ignition::transport::Node node_;

  /// \brief Gazebo state publisher.
public:
  ignition::transport::Node::Publisher pub_state_;

  /// \brief Joint Entity
public:
  ignition::gazebo::Entity entity_joint_;

  /// \brief Joint name
public:
  std::string name_joint_;

  /// \brief last publication time
public:
  std::chrono::steady_clock::duration time_last_pub_{std::chrono::steady_clock::duration::zero()};

  /// \brief Commanded joint
public:
  struct Cmd
  {
    double pos;
    double vel;
    double force{0.0};
    double kp{1.0};
    double kd{0.0};
    double max_i{30.0};
  } cmd_;

  /// \brief Parameters plugin
public:
  struct Params
  {
    double motor_constant_{0.025};
    double gear_ratio_{9};
    double max_current_{4};
    double max_velocity_{80.0};
    double lower_limit_{-1.5};
    double upper_limit_{1.5};
    double kp_{0.5};
    double kd_{5.0};
    double kff_{1.0};
    double dt_{0.001};
  } params_;

  /// \brief mutex to protect Cmd
public:
  std::mutex mutex_cmd_;

  /// \brief Model interface
public:
  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
};

//////////////////////////////////////////////////
OdriJointController::OdriJointController()
    : dataPtr(std::make_unique<OdriJointControllerPrivate>())
{
}

//////////////////////////////////////////////////
void OdriJointController::Configure(const ignition::gazebo::Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    ignition::gazebo::EntityComponentManager &_ecm,
                                    ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model_ = ignition::gazebo::Model(_entity);

  if (!this->dataPtr->model_.Valid(_ecm))
  {
    ignerr << "OdriJointController plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  bool has_joint_name = false;
  if (_sdf->HasElement("joint_name"))
  {
    if (!_sdf->Get<std::string>("joint_name").empty())
    {
      this->dataPtr->name_joint_ = _sdf->Get<std::string>("joint_name");
      has_joint_name = true;
    }
    else
    {
      ignerr << "<joint_name> provided but is empty." << std::endl;
    }
  }
  if (!has_joint_name)
  {
    ignerr << "Failed to get any <joint_name>." << std::endl;
    return;
  }

  // Subscribe to commands
  std::string topic_cmd;
  if (!_sdf->HasElement("topic_cmd"))
  {
    topic_cmd = ignition::transport::TopicUtils::AsValidTopic("/model/" +
                                                              this->dataPtr->model_.Name(_ecm) + "/joint/" +
                                                              this->dataPtr->name_joint_ + "/cmd");
    if (topic_cmd.empty())
    {
      ignerr << "Failed to create topic for joint ["
             << this->dataPtr->name_joint_
             << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("topic_cmd"))
  {
    topic_cmd = ignition::transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic_cmd"));

    if (topic_cmd.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("topic_cmd")
             << "]"
             << " for joint [" << this->dataPtr->name_joint_
             << "]" << std::endl;
      return;
    }
  }
  this->dataPtr->node_.Subscribe(topic_cmd, &OdriJointControllerPrivate::OnOdriCmd,
                                 this->dataPtr.get());

  ignmsg << "OdriJointController subscribing to OdriCmd messages on [" << topic_cmd
         << "]" << std::endl;

  // Publish states
  std::string topic_state;
  if (!_sdf->HasElement("topic_state"))
  {
    topic_state = ignition::transport::TopicUtils::AsValidTopic("/model/" +
                                                                this->dataPtr->model_.Name(_ecm) + "/joint/" +
                                                                this->dataPtr->name_joint_ + "/cmd");
    if (topic_state.empty())
    {
      ignerr << "Failed to create topic for joint ["
             << this->dataPtr->name_joint_
             << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("topic_state"))
  {
    topic_state = ignition::transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic_state"));

    if (topic_state.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("topic_state")
             << "]"
             << " for joint [" << this->dataPtr->name_joint_
             << "]" << std::endl;
      return;
    }
  }
  this->dataPtr->pub_state_ = this->dataPtr->node_.Advertise<sensor_msgs::msgs::OdriState>(topic_state);

  // Get other parameters
  if (_sdf->HasElement("motor_constant"))
  {
    this->dataPtr->params_.motor_constant_ = _sdf->Get<double>("motor_constant");
  }
  if (_sdf->HasElement("gear_ratio"))
  {
    this->dataPtr->params_.gear_ratio_ = _sdf->Get<double>("gear_ratio");
  }
  if (_sdf->HasElement("max_current"))
  {
    this->dataPtr->params_.max_current_ = _sdf->Get<double>("max_current");
  }
  if (_sdf->HasElement("max_velocity"))
  {
    this->dataPtr->params_.max_velocity_ = _sdf->Get<double>("max_velocity");
  }
  if (_sdf->HasElement("lower_limit"))
  {
    this->dataPtr->params_.lower_limit_ = _sdf->Get<double>("lower_limit");
  }
  if (_sdf->HasElement("upper_limit"))
  {
    this->dataPtr->params_.upper_limit_ = _sdf->Get<double>("upper_limit");
  }
  if (_sdf->HasElement("kp"))
  {
    this->dataPtr->params_.kp_ = _sdf->Get<double>("kp");
  }
  if (_sdf->HasElement("kd"))
  {
    this->dataPtr->params_.kd_ = _sdf->Get<double>("kd");
  }
  if (_sdf->HasElement("kff"))
  {
    this->dataPtr->params_.kff_ = _sdf->Get<double>("kff");
  }
  if (_sdf->HasElement("dt"))
  {
    this->dataPtr->params_.dt_ = _sdf->Get<double>("dt");
  }
}

//////////////////////////////////////////////////
void OdriJointController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OdriJointController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  if (!this->dataPtr->entity_joint_)
  {
    ignition::gazebo::Entity joint = this->dataPtr->model_.JointByName(_ecm, this->dataPtr->name_joint_);
    if (joint != ignition::gazebo::kNullEntity)
    {
      this->dataPtr->entity_joint_ = joint;
    }
    else
    {
      ignwarn << "Failed to find joint [" << this->dataPtr->name_joint_ << "]" << std::endl;
    }
  }
  if (!this->dataPtr->entity_joint_)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
  {
    return;
  }
  // Create joint position component if one doesn't exist
  auto comp_joint_pos = _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->entity_joint_);
  if (!comp_joint_pos)
  {
    _ecm.CreateComponent(this->dataPtr->entity_joint_, ignition::gazebo::components::JointPosition());
  }

  // Create joint velocity component if one doesn't exist
  auto comp_joint_vel = _ecm.Component<ignition::gazebo::components::JointVelocity>(this->dataPtr->entity_joint_);
  if (!comp_joint_vel)
  {
    _ecm.CreateComponent(this->dataPtr->entity_joint_, ignition::gazebo::components::JointVelocity());
  }

  // We just created the joint position component, give one iteration for the
  // physics system to update its size
  if (comp_joint_pos == nullptr || comp_joint_pos->Data().empty())
  {
    return;
  }

  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (comp_joint_vel == nullptr || comp_joint_vel->Data().empty())
  {
    return;
  }
  // We just created the joint force component, give one iteration for the
  // physics system to update its size

  const std::chrono::steady_clock::duration current_time = _info.simTime;
  const double dt = std::chrono::duration<double>(current_time - this->dataPtr->time_last_pub_).count();
  if (dt < this->dataPtr->params_.dt_)
  {
    return;
  }

  double cmd_pos;
  double cmd_vel;
  double cmd_force{0.0};
  double kp{1.0};
  double kd{0.0};
  double max_i{30.0};
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex_cmd_);
    cmd_pos = this->dataPtr->cmd_.pos;
    cmd_vel = this->dataPtr->cmd_.vel;
    cmd_force = this->dataPtr->cmd_.force;
    kp = this->dataPtr->cmd_.kp;
    kd = this->dataPtr->cmd_.kd;
    max_i = this->dataPtr->cmd_.max_i;
  }

  double error_pos = cmd_pos - comp_joint_pos->Data().at(0);
  double force_pos = kp * error_pos;

  double error_vel = cmd_vel - comp_joint_vel->Data().at(0);
  double force_vel = kd * error_vel;

  // Force
  double force = force_pos + force_vel + cmd_force;
  // ignerr << "force pos: " << force_pos << "\n";
  // ignerr << "force vel: " << force_vel << "\n";
  // ignerr << "force ff: " << cmd_force << "\n";
  // ignerr << "force total: " << force << "\n";
  auto com_joint_force_cmd = _ecm.Component<ignition::gazebo::components::JointForceCmd>(this->dataPtr->entity_joint_);
  if (com_joint_force_cmd == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->entity_joint_,
                         ignition::gazebo::components::JointForceCmd({force}));
  }
  else
  {
    *com_joint_force_cmd = ignition::gazebo::components::JointForceCmd({force});
  }

  sensor_msgs::msgs::OdriState msg_state;

  msg_state.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());
  msg_state.set_joint_name(this->dataPtr->name_joint_);
  msg_state.set_pos(comp_joint_pos->Data().at(0));
  msg_state.set_vel(comp_joint_vel->Data().at(0));

  this->dataPtr->pub_state_.Publish(msg_state);
  this->dataPtr->time_last_pub_ = current_time;
}

//////////////////////////////////////////////////
void OdriJointControllerPrivate::OnOdriCmd(const cmd_msgs::msgs::OdriCmd &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_cmd_);
  this->cmd_.pos = _msg.cmd_pos();
  this->cmd_.vel = _msg.cmd_vel();
  if (_msg.has_cmd_force())
  {
    this->cmd_.force = _msg.cmd_force();
  }
  if (_msg.has_kp())
  {
    this->cmd_.kp = _msg.kp();
  }
  if (_msg.has_kd())
  {
    this->cmd_.kd = _msg.kd();
  }
  if (_msg.has_max_i())
  {
    this->cmd_.max_i = _msg.max_i();
  }
}