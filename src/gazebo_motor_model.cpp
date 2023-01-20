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

#include "gazebo_motor_model.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    gazebo_motor_model::GazeboMotorModel,
    ignition::gazebo::System,
    gazebo_motor_model::GazeboMotorModel::ISystemConfigure,
    gazebo_motor_model::GazeboMotorModel::ISystemPreUpdate,
    gazebo_motor_model::GazeboMotorModel::ISystemPostUpdate)

using namespace gazebo_motor_model;

GazeboMotorModel::GazeboMotorModel()
{
}

GazeboMotorModel::~GazeboMotorModel()
{
}

void GazeboMotorModel::getSdfParams(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (_sdf->HasElement("jointName"))
  {
    joint_name_ = _sdf->Get<std::string>("jointName");
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  }

  if (_sdf->HasElement("linkName"))
  {
    link_name_ = _sdf->Get<std::string>("linkName");
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  }

  if (_sdf->HasElement("motorNumber"))
  {
    motor_number_ = _sdf->Get<int>("motorNumber");
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a motorNumber.\n";
  }

  if (_sdf->HasElement("turningDirection"))
  {
    std::string turning_direction = _sdf->Get<std::string>("turningDirection");
    if (turning_direction == "cw")
    {
      turning_direction_ = turning_direction::CW;
    }
    else if (turning_direction == "ccw")
    {
      turning_direction_ = turning_direction::CCW;
    }
    else
    {
      ignerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
    }
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";
  }

  if (_sdf->HasElement("motorSpeedPubTopic"))
  {
    motor_speed_pub_topic_ = _sdf->Get<std::string>("motorSpeedPubTopic");
  }
  else
  {
    motor_speed_pub_topic_ = kDefaultMotorSpeedPubTopic;
    ignwarn << "[gazebo_motor_model] Using default value for motorSpeedPubTopic: "
            << motor_speed_pub_topic_ << "\n";
  }

  if (_sdf->HasElement("commandSubTopic"))
  {
    command_sub_topic_ = _sdf->Get<std::string>("commandSubTopic");
  }
  else
  {
    command_sub_topic_ = kDefaultCommandSubTopic;
    ignwarn << "[gazebo_motor_model] Using default value for commandSubTopic: "
            << command_sub_topic_ << "\n";
  }

  if (_sdf->HasElement("timeConstantUp"))
  {
    time_constant_up_ = _sdf->Get<double>("timeConstantUp");
  }
  else
  {
    time_constant_up_ = kDefaultTimeConstantUp;
    ignwarn << "[gazebo_motor_model] Using default value for timeConstantUp: "
            << time_constant_up_ << "\n";
  }

  if (_sdf->HasElement("timeConstantDown"))
  {
    time_constant_down_ = _sdf->Get<double>("timeConstantDown");
  }
  else
  {
    time_constant_down_ = kDefaultTimeConstantDown;
    ignwarn << "[gazebo_motor_model] Using default value for timeConstantDown: "
            << time_constant_down_ << "\n";
  }

  if (_sdf->HasElement("maxRotVelocity"))
  {
    max_rot_velocity_ = _sdf->Get<double>("maxRotVelocity");
  }
  else
  {
    max_rot_velocity_ = kDefaultMaxRotVelocity;
    ignwarn << "[gazebo_motor_model] Using default value for maxRotVelocity: "
            << max_rot_velocity_ << "\n";
  }

  if (_sdf->HasElement("motorConstant"))
  {
    motor_constant_ = _sdf->Get<double>("motorConstant");
  }
  else
  {
    motor_constant_ = kDefaultMotorConstant;
    ignwarn << "[gazebo_motor_model] Using default value for motorConstant: "
            << motor_constant_ << "\n";
  }

  if (_sdf->HasElement("momentConstant"))
  {
    moment_constant_ = _sdf->Get<double>("momentConstant");
  }
  else
  {
    moment_constant_ = kDefaultMomentConstant;
    ignwarn << "[gazebo_motor_model] Using default value for momentConstant: "
            << moment_constant_ << "\n";
  }

  if (_sdf->HasElement("rotorDragCoefficient"))
  {
    rotor_drag_coefficient_ = _sdf->Get<double>("rotorDragCoefficient");
  }
  else
  {
    rotor_drag_coefficient_ = kDefaultRotorDragCoefficient;
    ignwarn << "[gazebo_motor_model] Using default value for rotorDragCoefficient: "
            << rotor_drag_coefficient_ << "\n";
  }

  if (_sdf->HasElement("rollingMomentCoefficient"))
  {
    rolling_moment_coefficient_ = _sdf->Get<double>("rollingMomentCoefficient");
  }
  else
  {
    rolling_moment_coefficient_ = kDefaultRollingMomentCoefficient;
    ignwarn << "[gazebo_motor_model] Using default value for rollingMomentCoefficient: "
            << rolling_moment_coefficient_ << "\n";
  }

  if (_sdf->HasElement("rotorVelocitySlowdownSim"))
  {
    rotor_velocity_slowdown_sim_ = _sdf->Get<double>("rotorVelocitySlowdownSim");
  }
  else
  {
    rotor_velocity_slowdown_sim_ = kDefaultRotorVelocitySlowdownSim;
    ignwarn << "[gazebo_motor_model] Using default value for rotorVelocitySlowdownSim: "
            << rotor_velocity_slowdown_sim_ << "\n";
  }

  if (_sdf->HasElement("reversible"))
  {
    reversible_ = _sdf->Get<bool>("reversible");
  }
  else
  {
    reversible_ = kDefaultReversible;
    ignwarn << "[gazebo_motor_model] Using default value for reversible: "
            << reversible_ << "\n";
  }
}

void GazeboMotorModel::Configure(const ignition::gazebo::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 ignition::gazebo::EntityComponentManager &_ecm,
                                 ignition::gazebo::EventManager & /*_eventMgr*/)
{
  model_ = ignition::gazebo::Model(_entity);

  getSdfParams(_sdf);

  std::string model_name_ = model_.Name(_ecm);

  // Get link and joint entities
  model_link_ = model_.LinkByName(_ecm, link_name_);
  model_joint_ = model_.JointByName(_ecm, joint_name_);

  auto list = model_.Links(_ecm);
  for (auto a = list.begin(); a != list.end(); ++a)
  {
    auto link = ignition::gazebo::Link(*a);
    ignerr << link.Name(_ecm).value() << "\n";
  }

  auto list2 = model_.Joints(_ecm);
  for (auto a = list2.begin(); a != list2.end(); ++a)
  {
    auto joint = _ecm.ComponentData<ignition::gazebo::components::Name>(*a).value();
    ignerr << joint << "\n";
  }

  ignerr << _ecm.ComponentData<ignition::gazebo::components::Name>(model_link_).value() << "\n";
  ignerr << _ecm.ComponentData<ignition::gazebo::components::Name>(model_joint_).value() << "\n";

  link_rotor_ = ignition::gazebo::Link(model_link_);

  motor_velocity_pub_ = this->node.Advertise<std_msgs::msgs::Float>("/" + model_name_ + motor_speed_pub_topic_);
  node.Subscribe("/" + model_name_ + command_sub_topic_, &GazeboMotorModel::VelocityCallback, this);
  //  node.Subscribe(motor_failure_sub_topic_, &GazeboMotorModel::MotorFailureCallback, this);
  node.Subscribe(wind_sub_topic_, &GazeboMotorModel::WindVelocityCallback, this);

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));

  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::LinearVelocity::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::LinearVelocity());
  }

  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::ParentEntity::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::ParentEntity());
  }

  // Assure that the parent link as a world pose component
  // TODO: GET THE PARENT LINK NOT THE ENTITY
  parent_link_ = _ecm.ComponentData<ignition::gazebo::components::ParentEntity>(model_joint_).value();
  link_parent_ = ignition::gazebo::Link(parent_link_);
  ignerr << link_parent_.Name(_ecm).value();

  if (!_ecm.EntityHasComponentType(parent_link_, ignition::gazebo::components::WorldPose::typeId))
  {
    _ecm.CreateComponent(parent_link_, ignition::gazebo::components::WorldPose());
  }

  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldPose::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldPose());
  }

  if (!_ecm.EntityHasComponentType(model_joint_, ignition::gazebo::components::JointVelocity::typeId))
  {
    _ecm.CreateComponent(model_joint_, ignition::gazebo::components::JointVelocity());
  }
  if (!_ecm.EntityHasComponentType(model_joint_, ignition::gazebo::components::JointAxis::typeId))
  {
    _ecm.CreateComponent(model_joint_, ignition::gazebo::components::JointAxis());
  }
  if (!_ecm.EntityHasComponentType(model_joint_, ignition::gazebo::components::JointVelocityCmd::typeId))
  {
    _ecm.CreateComponent(model_joint_, ignition::gazebo::components::JointVelocityCmd());
  }
}

void GazeboMotorModel::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 ignition::gazebo::EntityComponentManager &_ecm)
{
  if (is_first_update)
  {
    is_first_update = false;
    sampling_time_ = 0;
  }
  else
  {
    sampling_time_ = std::chrono::duration<double>(_info.simTime - prev_sim_time_).count();
  }
  prev_sim_time_ = _info.simTime;
  UpdateForcesAndMoments(_ecm);
}

void GazeboMotorModel::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  const ignition::gazebo::EntityComponentManager &_ecm)
{
  // UpdateMotorFail();
  Publish(_ecm);
}

void GazeboMotorModel::Publish(const ignition::gazebo::EntityComponentManager &_ecm)
{
  std::vector<double> joint_vel = _ecm.ComponentData<ignition::gazebo::components::JointVelocity>(model_joint_).value();

  double vel = 0.0;

  if (joint_vel.size() != 0)
  {
    vel = joint_vel.at(0);
  }
  turning_velocity_msg_.set_data(vel);
  // FIXME: Commented out to prevent warnings about queue limit reached.
  motor_velocity_pub_.Publish(turning_velocity_msg_);
}

void GazeboMotorModel::VelocityCallback(const mav_msgs::msgs::CommandMotorSpeed &rot_velocities)
{
  if (rot_velocities.motor_speed_size() < motor_number_)
  {
    std::cout << "You tried to access index " << motor_number_
              << " of the MotorSpeed message array which is of size " << rot_velocities.motor_speed_size() << "." << std::endl;
  }
  else
    ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities.motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

/*void GazeboMotorModel::MotorFailureCallback(const msgs::Int &fail_msg)
{
  motor_Failure_Number_ = fail_msg.data();
}*/

void GazeboMotorModel::UpdateForcesAndMoments(ignition::gazebo::EntityComponentManager &_ecm)
{

  std::vector<double> joint_vel = _ecm.ComponentData<ignition::gazebo::components::JointVelocity>(model_joint_).value();
  if (joint_vel.size() == 0)
  {
    motor_rot_vel_ = 0.0;
  }
  else
  {
    motor_rot_vel_ = joint_vel.at(0);
  }

  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_))
  {
    ignerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }

  const double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;

  double force = real_motor_velocity * std::abs(real_motor_velocity) * motor_constant_;
  if (!reversible_)
  {
    // Not allowed to have negative thrust.
    force = std::abs(force);
  }

  // scale down force linearly with forward speed
  // XXX this has to be modelled better
  //
  // ignition::math::Vector3d body_velocity = _ecm.ComponentData<ignition::gazebo::components::LinearVelocity>(model_link_).value();
  const auto body_velocity = link_rotor_.WorldLinearVelocity(_ecm);
  // sdf::JointAxis joint_sdf = _ecm.ComponentData<ignition::gazebo::components::JointAxis>(model_joint_).value();
  const ignition::math::Vector3d joint_axis = _ecm.ComponentData<ignition::gazebo::components::JointAxis>(model_joint_).value().Xyz();

  const ignition::math::Vector3d relative_wind_velocity = body_velocity.value_or(ignition::math::Vector3d(0, 0, 0)) - wind_vel_;
  const ignition::math::Vector3d velocity_parallel_to_rotor_axis = (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  const double vel = velocity_parallel_to_rotor_axis.Length();

  double scalar = 1 - vel / 25.0; // at 25 m/s the rotor will not produce any force anymore
  scalar = ignition::math::clamp(scalar, 0.0, 1.0);
  // Apply a force to the link.

  const auto pose_link = link_rotor_.WorldPose(_ecm).value();
  const ignition::math::Vector3d force_relative(0.0, 0.0, force * scalar);
  link_rotor_.AddWorldForce(_ecm, pose_link.Rot().RotateVector(force_relative));

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  const ignition::math::Vector3d velocity_perpendicular_to_rotor_axis = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  const ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
  // Apply air_drag to link.
  link_rotor_.AddWorldForce(_ecm, air_drag);

  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  const auto pose_parent_link = link_parent_.WorldPose(_ecm).value();
  // // The tansformation from the parent_link to the link_.
  const ignition::math::Pose3d pose_difference = pose_link - pose_parent_link;

  const ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * force * moment_constant_);
  // // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  const ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
  // link_parent_.AddWorldWrench(_ecm, ignition::math::Vector3d(0, 0, 0), pose_parent_link.Rot().RotateVector(drag_torque_parent_frame));
  //  // - \omega * \mu_1 * V_A^{\perp}
  const ignition::math::Vector3d rolling_moment = -std::abs(real_motor_velocity) * turning_direction_ * rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
  link_rotor_.AddWorldWrench(_ecm, ignition::math::Vector3d(0, 0, 0), rolling_moment);

  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

  ignition::gazebo::components::JointVelocityCmd vel_cmd({turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_});
  _ecm.SetComponentData<ignition::gazebo::components::JointVelocityCmd>(model_joint_, vel_cmd.Data());
}

/*void GazeboMotorModel::UpdateMotorFail()
{
  if (motor_number_ == motor_Failure_Number_ - 1)
  {
    // motor_constant_ = 0.0;
    joint_->SetVelocity(0, 0);
    if (screen_msg_flag)
    {
      std::cout << "Motor number [" << motor_Failure_Number_ << "] failed!  [Motor thrust = 0]" << std::endl;
      tmp_motor_num = motor_Failure_Number_;

      screen_msg_flag = 0;
    }
  }
  else if (motor_Failure_Number_ == 0 && motor_number_ == tmp_motor_num - 1)
  {
    if (!screen_msg_flag)
    {
      // motor_constant_ = kDefaultMotorConstant;
      std::cout << "Motor number [" << tmp_motor_num << "] running! [Motor thrust = (default)]" << std::endl;
      screen_msg_flag = 1;
    }
  }
}*/

void GazeboMotorModel::WindVelocityCallback(const physics_msgs::msgs::Wind &msg)
{
  wind_vel_ = ignition::math::Vector3d(msg.velocity().x(),
                                       msg.velocity().y(),
                                       msg.velocity().z());
}
