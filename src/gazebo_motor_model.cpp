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
    : command_sub_topic_(kDefaultCommandSubTopic),
      // motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic),
      motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic),
      motor_number_(0),
      // motor_Failure_Number_(0),
      turning_direction_(turning_direction::CW),
      max_force_(kDefaultMaxForce),
      max_rot_velocity_(kDefaulMaxRotVelocity),
      moment_constant_(kDefaultMomentConstant),
      motor_constant_(kDefaultMotorConstant),
      // motor_test_sub_topic_(kDefaultMotorTestSubTopic),
      ref_motor_rot_vel_(0.0),
      rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
      rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
      rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
      time_constant_down_(kDefaultTimeConstantDown),
      time_constant_up_(kDefaultTimeConstantUp),
      reversible_(false)
{
}

GazeboMotorModel::~GazeboMotorModel()
{
}

void GazeboMotorModel::InitializeParams()
{
}

void GazeboMotorModel::getSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{

  if (sdf->HasElement("jointName"))
  {
    joint_name_ = sdf->Get<std::string>("jointName");
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  }

  // setup joint control pid to control joint
  /*if (sdf->HasElement("joint_control_pid"))
  {
    sdf::Element pid = sdf->GetElement("joint_control_pid");
    double p = 0.1;
    if (pid->HasElement("p"))
      p = pid->Get<double>("p");
    double i = 0;
    if (pid->HasElement("i"))
      i = pid->Get<double>("i");
    double d = 0;
    if (pid->HasElement("d"))
      d = pid->Get<double>("d");
    double iMax = 0;
    if (pid->HasElement("iMax"))
      iMax = pid->Get<double>("iMax");
    double iMin = 0;
    if (pid->HasElement("iMin"))
      iMin = pid->Get<double>("iMin");
    double cmdMax = 3;
    if (pid->HasElement("cmdMax"))
      cmdMax = pid->Get<double>("cmdMax");
    double cmdMin = -3;
    if (pid->HasElement("cmdMin"))
      cmdMin = pid->Get<double>("cmdMin");
    pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    use_pid_ = true;
  }
  else
  {
    use_pid_ = false;
  }*/

  if (sdf->HasElement("linkName"))
  {
    link_name_ = sdf->Get<std::string>("linkName");
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  }

  if (sdf->HasElement("motorNumber"))
  {
    motor_number_ = sdf->Get<int>("motorNumber");
  }
  else
  {
    ignerr << "[gazebo_motor_model] Please specify a motorNumber.\n";
  }

  if (sdf->HasElement("turningDirection"))
  {
    std::string turning_direction = sdf->Get<std::string>("turningDirection");
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      ignerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    ignerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

  if (sdf->HasElement("reversible"))
  {
    reversible_ = sdf->Get<bool>("reversible");
  }

  gazebo::getSdfParam<std::string>(sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  gazebo::getSdfParam<std::string>(sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                                   motor_speed_pub_topic_);

  gazebo::getSdfParam<double>(sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  gazebo::getSdfParam<double>(sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                              rolling_moment_coefficient_);
  gazebo::getSdfParam<double>(sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  gazebo::getSdfParam<double>(sdf, "motorConstant", motor_constant_, motor_constant_);
  gazebo::getSdfParam<double>(sdf, "momentConstant", moment_constant_, moment_constant_);

  gazebo::getSdfParam<double>(sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  gazebo::getSdfParam<double>(sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  gazebo::getSdfParam<double>(sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);
}

void GazeboMotorModel::Publish(const ignition::gazebo::EntityComponentManager &_ecm)
{
  const double vel = _ecm.ComponentData<ignition::gazebo::components::JointVelocity>(joint_)->at(0);
  turning_velocity_msg_.set_data(vel);
  // FIXME: Commented out to prevent warnings about queue limit reached.
  // motor_velocity_pub_->Publish(turning_velocity_msg_);
}

void GazeboMotorModel::Configure(const ignition::gazebo::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 ignition::gazebo::EntityComponentManager &_ecm,
                                 ignition::gazebo::EventManager & /*_eventMgr*/)
{
  getSdfParams(_sdf);

  model_ = ignition::gazebo::Model(_entity);
  std::string model_name_ = model_.Name(_ecm);

  // Get link entity
  model_link_ = model_.LinkByName(_ecm, link_name_);
  /*
  std::cout << "Subscribing to: " << motor_test_sub_topic_ << std::endl;
  motor_sub_ = node_handle_->Subscribe<mav_msgs::msgs::MotorSpeed>("~/" + model_->GetName() + motor_test_sub_topic_, &GazeboMotorModel::testProto, this);
  */

  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
  joint_ = model_.JointByName(_ecm, joint_name_);

  motor_velocity_pub_ = this->node.Advertise<std_msgs::msgs::Float>("/" + model_name_ + motor_speed_pub_topic_);
  node.Subscribe("/" + model_name_ + command_sub_topic_, &GazeboMotorModel::VelocityCallback, this);
  // node.Subscribe(motor_failure_sub_topic_, &GazeboMotorModel::MotorFailureCallback, this);
  node.Subscribe(wind_sub_topic_, &GazeboMotorModel::WindVelocityCallback, this);

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

void GazeboMotorModel::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 ignition::gazebo::EntityComponentManager &_ecm)
{
  sampling_time_ = std::chrono::duration<double>(_info.simTime - prev_sim_time_).count();
  prev_sim_time_ = _info.simTime;
  UpdateForcesAndMoments(_ecm);
}

void GazeboMotorModel::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  const ignition::gazebo::EntityComponentManager &_ecm)
{
  // UpdateMotorFail();
  Publish(_ecm);
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
  motor_rot_vel_ = _ecm.ComponentData<ignition::gazebo::components::JointVelocity>(joint_)->at(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_))
  {
    ignerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * std::abs(real_motor_velocity) * motor_constant_;
  if (!reversible_)
  {
    // Not allowed to have negative thrust.
    force = std::abs(force);
  }

  // scale down force linearly with forward speed
  // XXX this has to be modelled better
  //
  ignition::math::Vector3d body_velocity = _ecm.ComponentData<ignition::gazebo::components::LinearVelocity>(model_link_).value();
  ignition::math::Vector3d joint_axis = _ecm.ComponentData<ignition::gazebo::components::JointAxis>(joint_).value().Xyz();

  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;
  ignition::math::Vector3d velocity_parallel_to_rotor_axis = (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  double vel = velocity_parallel_to_rotor_axis.Length();
  double scalar = 1 - vel / 25.0; // at 25 m/s the rotor will not produce any force anymore
  scalar = ignition::math::clamp(scalar, 0.0, 1.0);
  // Apply a force to the link.
  link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force * scalar));

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  ignition::math::Vector3d velocity_perpendicular_to_rotor_axis = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
  // Apply air_drag to link.
  _ecm.SetComponentData(model_link_, ignition::gazebo::components::JointForceCmd({air_drag}));
  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
#else
  ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
#endif
  ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * force * moment_constant_);
  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  ignition::math::Vector3d rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * turning_direction_ * rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

#if 0 // FIXME: disable PID for now, it does not play nice with the PX4 CI system.
  if (use_pid_)
  {
    double err = joint_->GetVelocity(0) - turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_;
    double rotorForce = pid_.Update(err, sampling_time_);
    joint_->SetForce(0, rotorForce);
    // ignerr << "rotor " << joint_->GetName() << " : " << rotorForce << "\n";
  }
  else
  {
#if GAZEBO_MAJOR_VERSION >= 7
    // Not desirable to use SetVelocity for parts of a moving model
    // impact on rest of the dynamic system is non-physical.
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#elif GAZEBO_MAJOR_VERSION >= 6
    // Not ideal as the approach could result in unrealistic impulses, and
    // is only available in ODE
    joint_->SetParam("fmax", 0, 2.0);
    joint_->SetParam("vel", 0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif
  }
#else
  _ecm.SetComponentData(joint_, ignition::gazebo::components::JointVelocityCmd(turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_));

#endif /* if 0 */
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
