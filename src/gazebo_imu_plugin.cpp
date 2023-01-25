/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief IMU Plugin
 *
 * This plugin simulates imu data
 *
 * @author Hugo Duarte <duarte.hugo@lilo.org>
 */

#include "gazebo_imu_plugin.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    imu_plugin::ImuPlugin,
    ignition::gazebo::System,
    imu_plugin::ImuPlugin::ISystemConfigure,
    imu_plugin::ImuPlugin::ISystemPreUpdate,
    imu_plugin::ImuPlugin::ISystemPostUpdate)

using namespace imu_plugin;

ImuPlugin::ImuPlugin()
{
}

ImuPlugin::~ImuPlugin()
{
}

void ImuPlugin::Configure(const ignition::gazebo::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          ignition::gazebo::EntityComponentManager &_ecm,
                          ignition::gazebo::EventManager & /*_eventMgr*/)
{

  getSdfParams(_sdf);

  model_ = ignition::gazebo::Model(_entity);
  std::string model_name_ = model_.Name(_ecm);

  // Get link entity
  model_link_ = model_.LinkByName(_ecm, link_name_);

  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldPose::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldPose());
  }
  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldAngularVelocity::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldAngularVelocity());
  }
  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldLinearAcceleration::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldLinearAcceleration());
  }

  // Gravity
  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::Gravity::typeId))
  {
    gravity_W_ = ignition::math::Vector3d(0, 0, -kDefaultGravityMagnitude);
  }
  else
  {
    gravity_W_ = _ecm.Component<ignition::gazebo::components::Gravity>(model_link_)->Data();
  }
  imu_parameters_.gravity_magnitude = gravity_W_.Length();

  pub_imu_ = this->node.Advertise<sensor_msgs::msgs::Imu>("/" + model_name_ + imu_topic_);

  // Fill imu message.
  imu_message_.set_allocated_frame_id(&frame_id_);
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  for (int i = 0; i < 9; i++)
  {
    switch (i)
    {
    case 0:
    case 4:
    case 8:
      imu_message_.add_angular_velocity_covariance(imu_parameters_.gyroscope_noise_density *
                                                   imu_parameters_.gyroscope_noise_density);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(imu_parameters_.accelerometer_noise_density *
                                                      imu_parameters_.accelerometer_noise_density);
      break;
    case 1:
    case 2:
    case 3:
    case 5:
    case 6:
    case 7:
      imu_message_.add_angular_velocity_covariance(0.0);

      imu_message_.add_orientation_covariance(-1.0);

      imu_message_.add_linear_acceleration_covariance(0.0);
      break;
    }
  }

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i)
  {
    gyroscope_bias_[i] =
        sigma_bon_g * standard_normal_distribution_(random_generator_);
    accelerometer_bias_[i] =
        sigma_bon_a * standard_normal_distribution_(random_generator_);
  }
}

void ImuPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
}

// This gets called by the world update start event.
void ImuPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &_ecm)
{
  const std::chrono::steady_clock::duration current_time = _info.simTime;
  const double dt = std::chrono::duration<double>(current_time - last_pub_time_).count();
  if (dt > 1.0 / pub_rate_)
  {
    // TODO: Get valid data for

    const ignition::math::Pose3d T_W_I = _ecm.Component<ignition::gazebo::components::WorldPose>(model_link_)->Data();

    ignition::math::Quaterniond C_W_I = T_W_I.Rot();

    ignition::math::Vector3d angular_vel_I = _ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(model_link_)->Data();

    ignition::math::Vector3d acceleration_I = _ecm.Component<ignition::gazebo::components::WorldLinearAcceleration>(model_link_)->Data() - C_W_I.RotateVectorReverse(gravity_W_);

    Eigen::Vector3d linear_acceleration_I(acceleration_I.X(),
                                          acceleration_I.Y(),
                                          acceleration_I.Z());
    Eigen::Vector3d angular_velocity_I(angular_vel_I.X(),
                                       angular_vel_I.Y(),
                                       angular_vel_I.Z());

    addNoise(linear_acceleration_I, angular_velocity_I, dt);

    // Copy Eigen::Vector3d to ignition::msgs::Vector3d
    ignition::msgs::Vector3d *linear_acceleration = new ignition::msgs::Vector3d();
    linear_acceleration->set_x(linear_acceleration_I[0]);
    linear_acceleration->set_y(linear_acceleration_I[1]);
    linear_acceleration->set_z(linear_acceleration_I[2]);

    // Copy Eigen::Vector3d to ignition::msgs::Vector3d
    ignition::msgs::Vector3d *angular_velocity = new ignition::msgs::Vector3d();
    angular_velocity->set_x(angular_velocity_I[0]);
    angular_velocity->set_y(angular_velocity_I[1]);
    angular_velocity->set_z(angular_velocity_I[2]);

    // Copy ignition::math::Quaterniond to ignition::msgs::Quaternion
    ignition::msgs::Quaternion *orientation = new ignition::msgs::Quaternion();
    orientation->set_x(C_W_I.X());
    orientation->set_y(C_W_I.Y());
    orientation->set_z(C_W_I.Z());
    orientation->set_w(C_W_I.W());

    // Fill IMU message.
    // ADD HEaders
    // imu_message_.header.stamp.sec = current_time.sec;
    // imu_message_.header.stamp.nsec = current_time.nsec;
    imu_message_.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());
    imu_message_.set_seq(seq_++);

    // TODO(burrimi): Add orientation estimator.
    // imu_message_.orientation.w = 1;
    // imu_message_.orientation.x = 0;
    // imu_message_.orientation.y = 0;
    // imu_message_.orientation.z = 0;

    imu_message_.set_allocated_orientation(orientation);
    imu_message_.set_allocated_linear_acceleration(linear_acceleration);
    imu_message_.set_allocated_angular_velocity(angular_velocity);

    last_pub_time_ = current_time;

    // publish mag msg
    pub_imu_.Publish(imu_message_);
  }
}

void ImuPlugin::getSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{

  if (sdf->HasElement("imuTopic"))
  {
    imu_topic_ = sdf->Get<std::string>("imuTopic");
  }
  else
  {
    imu_topic_ = kDefaultImuTopic;
    ignwarn << "[gazebo_imu_plugin] Using default imu topic " << imu_topic_ << "\n";
  }

  if (sdf->HasElement("gyroscopeNoiseDensity"))
  {
    imu_parameters_.gyroscope_noise_density = sdf->Get<double>("gyroscopeNoiseDensity");
  }
  else
  {
    imu_parameters_.gyroscope_noise_density = kDefaultAdisGyroscopeNoiseDensity;
    ignwarn << "[gazebo_imu_plugin] Using gyroscope noise density " << imu_parameters_.gyroscope_noise_density << "\n";
  }

  if (sdf->HasElement("gyroscopeRandomWalk"))
  {
    imu_parameters_.gyroscope_random_walk = sdf->Get<double>("gyroscopeRandomWalk");
  }
  else
  {
    imu_parameters_.gyroscope_random_walk = kDefaultAdisGyroscopeRandomWalk;
    ignwarn << "[gazebo_imu_plugin] Using gyroscope random walk " << imu_parameters_.gyroscope_random_walk << "\n";
  }

  if (sdf->HasElement("gyroscopeBiasCorrelationTime"))
  {
    imu_parameters_.gyroscope_bias_correlation_time = sdf->Get<double>("gyroscopeBiasCorrelationTime");
  }
  else
  {
    imu_parameters_.gyroscope_bias_correlation_time = kDefaultAdisGyroscopeBiasCorrelationTime;
    ignwarn << "[gazebo_imu_plugin] Using gyroscope bias correlation time " << imu_parameters_.gyroscope_bias_correlation_time << "\n";
  }
  assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);

  if (sdf->HasElement("gyroscopeTurnOnBiasSigma"))
  {
    imu_parameters_.gyroscope_turn_on_bias_sigma = sdf->Get<double>("gyroscopeTurnOnBiasSigma");
  }
  else
  {
    imu_parameters_.gyroscope_turn_on_bias_sigma = kDefaultAdisGyroscopeTurnOnBiasSigma;
    ignwarn << "[gazebo_imu_plugin] Using gyroscope turn on bias sigma " << imu_parameters_.gyroscope_turn_on_bias_sigma << "\n";
  }

  if (sdf->HasElement("accelerometerNoiseDensity"))
  {
    imu_parameters_.accelerometer_noise_density = sdf->Get<double>("accelerometerNoiseDensity");
  }
  else
  {
    imu_parameters_.accelerometer_noise_density = kDefaultAdisAccelerometerNoiseDensity;
    ignwarn << "[gazebo_imu_plugin] Using accelerometer noise density " << imu_parameters_.accelerometer_noise_density << "\n";
  }

  if (sdf->HasElement("accelerometerRandomWalk"))
  {
    imu_parameters_.accelerometer_random_walk = sdf->Get<double>("accelerometerRandomWalk");
  }
  else
  {
    imu_parameters_.accelerometer_random_walk = kDefaultAdisAccelerometerRandomWalk;
    ignwarn << "[gazebo_imu_plugin] Using accelerometer random walk " << imu_parameters_.accelerometer_random_walk << "\n";
  }

  if (sdf->HasElement("accelerometerBiasCorrelationTime"))
  {
    imu_parameters_.accelerometer_bias_correlation_time = sdf->Get<double>("accelerometerBiasCorrelationTime");
  }
  else
  {
    imu_parameters_.accelerometer_bias_correlation_time = kDefaultAdisAccelerometerBiasCorrelationTime;
    ignwarn << "[gazebo_imu_plugin] Using accelerometer bias correlation time " << imu_parameters_.accelerometer_bias_correlation_time << "\n";
  }
  assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);

  if (sdf->HasElement("accelerometerTurnOnBiasSigma"))
  {
    imu_parameters_.accelerometer_turn_on_bias_sigma = sdf->Get<double>("accelerometerTurnOnBiasSigma");
  }
  else
  {
    imu_parameters_.accelerometer_turn_on_bias_sigma = kDefaultAdisAccelerometerTurnOnBiasSigma;
    ignwarn << "[gazebo_imu_plugin] Using accelerometer turn on bias sigma " << imu_parameters_.accelerometer_turn_on_bias_sigma << "\n";
  }

  link_name_ = sdf->Get<std::string>("linkName");
  frame_id_ = link_name_;
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void ImuPlugin::addNoise(
    Eigen::Vector3d &linear_acceleration,
    Eigen::Vector3d &angular_velocity,
    const double dt)
{
  // CHECK(linear_acceleration);
  // CHECK(angular_velocity);
  assert(dt > 0.0);

  // Gyrosocpe
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d =
      sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 *
           (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i)
  {
    gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
                         sigma_b_g_d * standard_normal_distribution_(random_generator_);
    angular_velocity[i] = angular_velocity[i] +
                          gyroscope_bias_[i] +
                          sigma_g_d * standard_normal_distribution_(random_generator_);
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d =
      sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 *
           (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i)
  {
    accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
                             sigma_b_a_d * standard_normal_distribution_(random_generator_);
    linear_acceleration[i] = linear_acceleration[i] +
                             accelerometer_bias_[i] +
                             sigma_a_d * standard_normal_distribution_(random_generator_);
  }
}
