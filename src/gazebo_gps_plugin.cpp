/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
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
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "gazebo_gps_plugin.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    gps_plugin::GpsPlugin,
    ignition::gazebo::System,
    gps_plugin::GpsPlugin::ISystemConfigure,
    gps_plugin::GpsPlugin::ISystemPreUpdate,
    gps_plugin::GpsPlugin::ISystemPostUpdate)

using namespace gps_plugin;

GpsPlugin::GpsPlugin()
{
}

GpsPlugin::~GpsPlugin()
{
}

void GpsPlugin::getSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{
  if (sdf->HasElement("gpsTopic"))
  {
    gps_topic_ = sdf->Get<std::string>("gpsTopic");
  }
  else
  {
    gps_topic_ = kDefaultGpsTopic;
    ignwarn << "[gazebo_gps_plugin] Using default gps topic " << gps_topic_ << "\n";
  }

  // get random walk in XY plane
  if (sdf->HasElement("gpsXYRandomWalk"))
  {
    gps_xy_random_walk_ = sdf->Get<double>("gpsXYRandomWalk");
  }
  else
  {
    gps_xy_random_walk_ = kDefaultGpsXYRandomWalk;
    ignwarn << "[gazebo_gps_plugin] Using default random walk in XY plane: "
            << gps_xy_random_walk_ << "\n";
  }

  // get random walk in Z
  if (sdf->HasElement("gpsZRandomWalk"))
  {
    gps_z_random_walk_ = sdf->Get<double>("gpsZRandomWalk");
  }
  else
  {
    gps_z_random_walk_ = kDefaultGpsZRandomWalk;
    ignwarn << "[gazebo_gps_plugin] Using default random walk in Z: "
            << gps_z_random_walk_ << "\n";
  }

  // get position noise density in XY plane
  if (sdf->HasElement("gpsXYNoiseDensity"))
  {
    gps_xy_noise_density_ = sdf->Get<double>("gpsXYNoiseDensity");
  }
  else
  {
    gps_xy_noise_density_ = kDefaultGpsXYNoiseDensity;
    ignwarn << "[gazebo_gps_plugin] Using default position noise density in XY plane: "
            << gps_xy_noise_density_ << "\n";
  }

  // get position noise density in Z
  if (sdf->HasElement("gpsZNoiseDensity"))
  {
    gps_z_noise_density_ = sdf->Get<double>("gpsZNoiseDensity");
  }
  else
  {
    gps_z_noise_density_ = kDefaultGpsZNoiseDensity;
    ignwarn << "[gazebo_gps_plugin] Using default position noise density in Z: "
            << gps_z_noise_density_ << "\n";
  }

  // get velocity noise density in XY plane
  if (sdf->HasElement("gpsVXYNoiseDensity"))
  {
    gps_vxy_noise_density_ = sdf->Get<double>("gpsVXYNoiseDensity");
  }
  else
  {
    gps_vxy_noise_density_ = kDefaultGpsVXYNoiseDensity;
    ignwarn << "[gazebo_gps_plugin] Using default velocity noise density in XY plane: "
            << gps_vxy_noise_density_ << "\n";
  }

  // get velocity noise density in Z
  if (sdf->HasElement("gpsVZNoiseDensity"))
  {
    gps_vz_noise_density_ = sdf->Get<double>("gpsVZNoiseDensity");
  }
  else
  {
    gps_vz_noise_density_ = kDefaultGpsVZNoiseDensity;
    ignwarn << "[gazebo_gps_plugin] Using default velocity noise density in Z: "
            << gps_vz_noise_density_ << "\n";
  }

  // get update rate
  if (sdf->HasElement("pubRate"))
  {
    pub_rate_ = sdf->Get<unsigned int>("pubRate");
  }
  else
  {
    pub_rate_ = kDefaultPubRate;
    ignwarn << "[gazebo_gps_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  link_name_ = sdf->Get<std::string>("link_name");
}

void GpsPlugin::Configure(const ignition::gazebo::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          ignition::gazebo::EntityComponentManager &_ecm,
                          ignition::gazebo::EventManager &_em)
{

  getSdfParams(_sdf);

  model_ = ignition::gazebo::Model(_entity);
  std::string model_name_ = model_.Name(_ecm);

  // Get link entity
  model_link_ = model_.LinkByName(_ecm, link_name_);

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  pub_gps_ = this->node.Advertise<sensor_msgs::msgs::SITLGps>("/" + model_name_ + gps_topic_);
  node.Subscribe("/" + model_name_ + "/groundtruth", &GpsPlugin::GroundtruthCallback, this);
}

void GpsPlugin::GroundtruthCallback(const sensor_msgs::msgs::Groundtruth &gt_msg)
{
  const std::chrono::steady_clock::duration current_time = std::chrono::microseconds(gt_msg.time_usec());
  if (groundtruth_last_time_ == std::chrono::microseconds(0))
  {
    groundtruth_last_time_ = std::chrono::microseconds(gt_msg.time_usec());
  }
  const double dt = std::chrono::duration<double>(current_time - groundtruth_last_time_).count();

  // update groundtruth pos and vel
  double groundtruth_lat_rad_ = gt_msg.latitude_rad();
  double groundtruth_lon_rad_ = gt_msg.longitude_rad();
  double groundtruth_alt_ = gt_msg.altitude();
  double groundtruth_vel_east_ = gt_msg.velocity_east();
  double groundtruth_vel_north_ = gt_msg.velocity_north();
  double groundtruth_vel_up_ = gt_msg.velocity_up();

  // compute speed without noise
  //auto vel_en = sqrt(groundtruth_vel_east_ * groundtruth_vel_east_ + groundtruth_vel_north_ * groundtruth_vel_north_);

  addNoise(groundtruth_lat_rad_, groundtruth_lon_rad_, groundtruth_alt_, groundtruth_vel_east_, groundtruth_vel_north_, groundtruth_vel_up_, dt);

  // compute speed with noise
  auto vel_en = sqrt(groundtruth_vel_east_ * groundtruth_vel_east_ + groundtruth_vel_north_ * groundtruth_vel_north_);

  // fill SITLGps msg
  sensor_msgs::msgs::SITLGps gps_msg;

  gps_msg.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());
  gps_msg.set_time_utc_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());

  // @note Unfurtonately the Gazebo GpsSensor seems to provide bad readings,
  // starting to drift and leading to global position loss
  // gps_msg.set_latitude_deg(parentSensor_->Latitude().Degree());
  // gps_msg.set_longitude_deg(parentSensor_->Longitude().Degree());
  // gps_msg.set_altitude(parentSensor_->Altitude());
  gps_msg.set_latitude_deg(groundtruth_lat_rad_ * 180.0 / M_PI);
  gps_msg.set_longitude_deg(groundtruth_lon_rad_ * 180.0 / M_PI);
  gps_msg.set_altitude(groundtruth_alt_);

  gps_msg.set_eph(std_xy_);
  gps_msg.set_epv(std_z_);

  gps_msg.set_velocity_east(groundtruth_vel_east_);
  gps_msg.set_velocity(vel_en);
  gps_msg.set_velocity_north(groundtruth_vel_north_);
  gps_msg.set_velocity_up(groundtruth_vel_up_);

  {
    // protect shared variables
    std::lock_guard<std::mutex> lock(data_mutex_);

    // add msg to buffer
    gps_delay_buffer_.push(gps_msg);
  }

  groundtruth_last_time_ = current_time;
}

void GpsPlugin::addNoise(double &lat, double &lon, double &alt, double &vel_east, double &vel_north, double &vel_up, const double dt)
{
  assert(dt > 0.0);

  // update noise parameters if gps_noise_ is set
  if (gps_noise_)
  {
    auto rst = sqrt(dt); // square root of the sampling time
    auto bandwidth_sqrt = sqrt(pub_rate_); // square root of publishing frequency
    pos_noise_gps_.X() = gps_xy_noise_density_ * bandwidth_sqrt * standard_normal_distribution_(random_generator_);
    pos_noise_gps_.Y() = gps_xy_noise_density_ * bandwidth_sqrt * standard_normal_distribution_(random_generator_);
    pos_noise_gps_.Z() = gps_z_noise_density_ * bandwidth_sqrt * standard_normal_distribution_(random_generator_);
    vel_noise_gps_.X() = gps_vxy_noise_density_ * bandwidth_sqrt * standard_normal_distribution_(random_generator_);
    vel_noise_gps_.Y() = gps_vxy_noise_density_ * bandwidth_sqrt * standard_normal_distribution_(random_generator_);
    vel_noise_gps_.Z() = gps_vz_noise_density_ * bandwidth_sqrt * standard_normal_distribution_(random_generator_);
    pos_random_walk_.X() = gps_xy_random_walk_ * rst * standard_normal_distribution_(random_generator_);
    pos_random_walk_.Y() = gps_xy_random_walk_ * rst * standard_normal_distribution_(random_generator_);
    pos_random_walk_.Z() = gps_z_random_walk_ * rst * standard_normal_distribution_(random_generator_);
  }
  else
  {
    pos_noise_gps_.X() = 0.0;
    pos_noise_gps_.Y() = 0.0;
    pos_noise_gps_.Z() = 0.0;
    vel_noise_gps_.X() = 0.0;
    vel_noise_gps_.Y() = 0.0;
    vel_noise_gps_.Z() = 0.0;
    pos_random_walk_.X() = 0.0;
    pos_random_walk_.Y() = 0.0;
    pos_random_walk_.Z() = 0.0;
  }

  // gps bias integration
  // follow the first order Gauss-Markov Process:
  // x_k = (1-dt/T_c)*x_(k-1)+w_k

  gps_bias_.X() += pos_random_walk_.X() - dt * gps_bias_.X() / gps_correlation_time_;
  gps_bias_.Y() += pos_random_walk_.Y() - dt * gps_bias_.Y() / gps_correlation_time_;
  gps_bias_.Z() += pos_random_walk_.Z() - dt * gps_bias_.Z() / gps_correlation_time_;
  // gps_bias_.Z() += pos_random_walk_.Z() * dt - gps_bias_.Z() / gps_correlation_time_;

  auto full_pos_noise = pos_noise_gps_ + gps_bias_;

  auto latlon = reproject(full_pos_noise, lat, lon, alt);

  lat = latlon.first;
  lon = latlon.second;
  alt += gps_bias_.Z() + pos_noise_gps_.Z();

  vel_north += vel_noise_gps_.X();
  vel_east += vel_noise_gps_.Y();
  vel_up += vel_noise_gps_.Z();
}

void GpsPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
}

void GpsPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &_ecm)
{
  // protect shared variables
  std::lock_guard<std::mutex> lock(data_mutex_);

  sensor_msgs::msgs::SITLGps gps_msg;

  const std::chrono::steady_clock::duration current_time = _info.simTime;
  const double dt = std::chrono::duration<double>(current_time - last_pub_time_).count();
  if (dt > 1.0 / pub_rate_)
  {
    last_pub_time_ = current_time;

    // do not sent empty msg
    // abort if buffer is empty
    if (gps_delay_buffer_.empty())
    {
      return;
    }

    while (true)
    {

      if (gps_delay_buffer_.empty())
      {
        // abort if buffer is empty already
        break;
      }

      gps_msg = gps_delay_buffer_.front();
      double gps_current_delay = std::chrono::duration<double>(current_time - std::chrono::microseconds(gps_delay_buffer_.front().time_usec())).count();

      // remove data that is too old or if buffer size is too large
      if (gps_current_delay >= gps_delay_)
      {
        gps_delay_buffer_.pop();
        // remove data if buffer too large
      }
      else if (gps_delay_buffer_.size() > gps_buffer_size_max_)
      {
        gps_delay_buffer_.pop();
      }
      else
      {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at the defined update rate
    pub_gps_.Publish(gps_msg);
  }
}
