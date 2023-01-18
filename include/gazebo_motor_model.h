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

#ifndef MOTOR_MODEL_HH_
#define MOTOR_MODEL_HH_

#include <Eigen/Eigen>

#include <ignition/transport/Node.hh>
#include <ignition/math.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointAxis.hh>

#include <CommandMotorSpeed.pb.h>
#include <MotorSpeed.pb.h>
#include <Float.pb.h>
#include <Wind.pb.h>

#include <common.h>

namespace gazebo_motor_model
{

  namespace turning_direction
  {
    const static int CCW = 1;
    const static int CW = -1;
  }

  // Default values
  static constexpr auto kDefaultNamespace = "";
  static constexpr auto kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
  // static constexpr auto kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
  static constexpr auto kDefaultMotorVelocityPubTopic = "/motor_speed";
  static constexpr auto wind_sub_topic_ = "/world_wind";

  // Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
  static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
  static constexpr double kDefaultMotorConstant = 8.54858e-06;
  static constexpr double kDefaultMomentConstant = 0.016;
  static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
  static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
  static constexpr double kDefaulMaxRotVelocity = 838.0;
  static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
  static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
  static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

  /// \brief This plugin publishes the motor speeds of your multirotor model.
  class IGNITION_GAZEBO_VISIBLE GazeboMotorModel :
      // This is class a system.
      public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
  {
  public:
    GazeboMotorModel();

  public:
    ~GazeboMotorModel() override;

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
    void InitializeParams();
    void Publish(const ignition::gazebo::EntityComponentManager &_ecm);
    void UpdateForcesAndMoments(ignition::gazebo::EntityComponentManager &_ecm);
    /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
    /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
    // void UpdateMotorFail();

    void VelocityCallback(const mav_msgs::msgs::CommandMotorSpeed &rot_velocities);
    // void MotorFailureCallback(const std_msgs::msgs::Int &fail_msg); /*!< Callback for the motor_failure_sub_ subscriber */
    void WindVelocityCallback(const physics_msgs::msgs::Wind &msg);

  private:
    ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

    std::string command_sub_topic_;
    // std::string motor_failure_sub_topic_;
    std::string joint_name_;
    std::string link_name_;
    std::string motor_speed_pub_topic_;
    std::string namespace_;

    int motor_number_;
    int turning_direction_;

    // int motor_Failure_Number_; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly */
    int tmp_motor_num; // A temporary variable used to print msg

    int screen_msg_flag = 1;

    double max_force_;
    double max_rot_velocity_;
    double moment_constant_;
    double motor_constant_;
    double ref_motor_rot_vel_;
    double rolling_moment_coefficient_;
    double rotor_drag_coefficient_;
    double rotor_velocity_slowdown_sim_;
    double time_constant_down_;
    double time_constant_up_;

    bool reversible_;

    ignition::transport::Node node;
    ignition::transport::Node::Publisher motor_velocity_pub_;

    ignition::math::Vector3d wind_vel_;

    ignition::gazebo::Entity joint_;

    // ignition::math::PID pid_;
    // bool use_pid_;

    std_msgs::msgs::Float turning_velocity_msg_;

    double motor_rot_vel_;

    double sampling_time_;
    std::chrono::steady_clock::duration prev_sim_time_;

    std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
    /*
      // Protobuf test
      std::string motor_test_sub_topic_;
      transport::SubscriberPtr motor_sub_;
    */
  };
}

#endif // MOTOR_MODEL_HH_