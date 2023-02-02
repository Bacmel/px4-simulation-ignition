/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @brief Barometer Plugin
 *
 * This plugin simulates barometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 */

#include "gazebo_barometer_plugin.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    barometer_plugin::BarometerPlugin,
    ignition::gazebo::System,
    barometer_plugin::BarometerPlugin::ISystemConfigure,
    barometer_plugin::BarometerPlugin::ISystemPreUpdate,
    barometer_plugin::BarometerPlugin::ISystemPostUpdate)

using namespace barometer_plugin;

BarometerPlugin::BarometerPlugin()
{
}

BarometerPlugin::~BarometerPlugin()
{
}

void BarometerPlugin::getSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{

  const char *env_alt = std::getenv("PX4_HOME_ALT");
  if (env_alt)
  {
    alt_home_ = std::stod(env_alt);
    ignmsg << "[gazebo_barometer_plugin] Home altitude is set to " << alt_home_ << " m\n";
  }
  else if (sdf->HasElement("homeAltitude"))
  {
    alt_home_ = sdf->Get<double>("homeAltitude");
  }
  else
  {
    alt_home_ = kDefaultHomeAltitude;
    ignwarn << "[gazebo_barometer_plugin] Using default home altitude of " << alt_home_ << " m\n";
  }

  if (sdf->HasElement("pubRate"))
  {
    pub_rate_ = sdf->Get<unsigned int>("pubRate");
  }
  else
  {
    pub_rate_ = kDefaultPubRate;
    ignwarn << "[gazebo_barometer_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  if (sdf->HasElement("baroTopic"))
  {
    baro_topic_ = sdf->Get<std::string>("baroTopic");
  }
  else
  {
    baro_topic_ = kDefaultBarometerTopic;
    ignwarn << "[gazebo_barometer_plugin] Using default barometer topic " << baro_topic_ << "\n";
  }

  if (sdf->HasElement("baroDriftPaPerSec"))
  {
    baro_drift_pa_per_sec_ = sdf->Get<double>("baroDriftPaPerSec");
  }
  else
  {
    baro_drift_pa_per_sec_ = kDefaultBaroDriftPaPerSec;
    ignwarn << "[gazebo_barometer_plugin] Using default barometer drift per second of " << baro_drift_pa_per_sec_ << "\n";
  }

  link_name_ = sdf->Get<std::string>("linkName");
}

void BarometerPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &_em)
{
  getSdfParams(_sdf);

  model_ = ignition::gazebo::Model(_entity);
  std::string model_name_ = model_.Name(_ecm);

  // Get link entity
  
  model_link_ = model_.LinkByName(_ecm, link_name_);

  // Get initial pose of the model that the plugin is attached to

  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldPose::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldPose());
  }

  pose_model_start_ = _ecm.ComponentData<ignition::gazebo::components::WorldPose>(model_link_).value();

  // Get gravity of the world
  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::Gravity::typeId))
  {
    gravity_magnitude_ = kDefaultGravityMagnitude;
  }
  else
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::Gravity());
    ignition::math::Vector3d gravity_W_ = _ecm.ComponentData<ignition::gazebo::components::Gravity>(model_link_).value();
    gravity_magnitude_ = gravity_W_.Length();
  }

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  pub_baro_ = this->node.Advertise<sensor_msgs::msgs::Pressure>("/" + model_name_ + baro_topic_);
}

void BarometerPlugin::addNoise(double &absolute_pressure, double &pressure_altitude, double &temperature_local, const double dt)
{
  assert(dt > 0.0);

  // Generate Gaussian noise sequence using polar form of Box-Muller transformation
  double y1;
  if (!baro_rnd_use_last_)
  {
    double x1, x2, w;
    do
    {
      x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
      x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);
    w = sqrt((-2.0 * log(w)) / w);
    // Calculate two values - the second value can be used next time because it is uncorrelated
    y1 = x1 * w;
    baro_rnd_y2_ = x2 * w;
    baro_rnd_use_last_ = true;
  }
  else
  {
    // Use the second value from last update
    y1 = baro_rnd_y2_;
    baro_rnd_use_last_ = false;
  }

  // Apply noise and drift
  const double abs_pressure_noise = 1.0f * (double)y1; // 1 Pa RMS noise
  baro_drift_pa_ += baro_drift_pa_per_sec_ * dt;
  const double noise = abs_pressure_noise + baro_drift_pa_;

  // Apply noise and drift
  absolute_pressure += noise;

  // Calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
  const double density_ratio = powf(kDefaultTemperatureMsl / temperature_local, 4.256f);
  const double rho = 1.225f / density_ratio;

  // Calculate pressure altitude including effect of pressure noise
  pressure_altitude -= noise / (gravity_magnitude_ * rho);
}

void BarometerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                ignition::gazebo::EntityComponentManager &_ecm)
{
}

void BarometerPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                 const ignition::gazebo::EntityComponentManager &_ecm)
{
  const std::chrono::steady_clock::duration current_time = _info.simTime;
  const double dt = std::chrono::duration<double>(current_time - last_pub_time_).count();

  if (dt > 1.0 / pub_rate_)
  {
    // Get pose of the model that the plugin is attached to
    const ignition::math::Pose3d pose_model_world = _ecm.Component<ignition::gazebo::components::WorldPose>(model_link_)->Data();
    ignition::math::Pose3d pose_model; // Z-component pose in local frame (relative to where it started)
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - pose_model_start_.Pos().Z();
    const double pose_n_z = -pose_model.Pos().Z(); // Convert Z-component from ENU to NED

    // Calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const double alt_msl = alt_home_ - pose_n_z;
    const double temperature_local = kDefaultTemperatureMsl - kDefaultLapseRate * alt_msl;
    const double pressure_ratio = powf(kDefaultTemperatureMsl / temperature_local, 5.256f);
    const double absolute_pressure = kDefaultPressureMsl / pressure_ratio;

    // Add noise
    double absolute_pressure_noisy = absolute_pressure;
    double pressure_altitude_noisy = alt_msl;
    double temperature_local_noisy = temperature_local;
    addNoise(absolute_pressure_noisy, pressure_altitude_noisy, temperature_local_noisy, dt);

    // Fill baro msg
    baro_msg_.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());

    baro_msg_.set_absolute_pressure(absolute_pressure_noisy * 0.01f); // Convert to hPa
    baro_msg_.set_pressure_altitude(pressure_altitude_noisy);
    baro_msg_.set_temperature(temperature_local_noisy + kDefaultAbsoluteZeroC); // Calculate temperature in Celsius

    // Publish baro msg
    pub_baro_.Publish(baro_msg_);

    last_pub_time_ = current_time;
  }
}
