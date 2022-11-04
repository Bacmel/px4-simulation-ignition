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
  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  if (env_lat)
  {
    lat_home_ = std::stod(env_lat) * M_PI / 180.0;
    ignmsg << "[gazebo_gps_plugin] Home latitude is set to " << std::stod(env_lat) << ".\n";
  }
  else if (sdf->HasElement("homeLatitude"))
  {
    lat_home_ = sdf->Get<double>("homeLatitude");
    lat_home_ = lat_home_ * M_PI / 180.0;
  }
  else
  {
    lat_home_ = kDefaultHomeLatitude;
    ignwarn << "[gazebo_gps_plugin] Home latitude is set to " << lat_home_ << "\n";
  }

  if (env_lon)
  {
    lon_home_ = std::stod(env_lon) * M_PI / 180.0;
    ignmsg << "[gazebo_gps_plugin] Home longitude is set to " << std::stod(env_lon) << ".\n";
  }
  else if (sdf->HasElement("homeLongitude"))
  {
    lon_home_ = sdf->Get<double>("homeLongitude");
    lon_home_ = lon_home_ * M_PI / 180.0;
  }
  else
  {
    lon_home_ = kDefaultHomeLongitude;
    ignwarn << "[gazebo_gps_plugin] Home longitude is set to " << lon_home_ << "\n";
  }

  if (env_alt)
  {
    alt_home_ = std::stod(env_alt);
    ignmsg << "[gazebo_gps_plugin] Home altitude is set to " << std::stod(env_alt) << ".\n";
  }
  else if (sdf->HasElement("homeAltitude"))
  {
    alt_home_ = sdf->Get<double>("homeAltitude");
    alt_home_ = alt_home_;
  }
  else
  {
    alt_home_ = kDefaultHomeAltitude;
    ignwarn << "[gazebo_gps_plugin] Home altitude is set to " << alt_home_ << "\n";
  }

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
  // Get link entity
  model_link_ = model_.LinkByName(_ecm, link_name_);

  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldPose::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldPose());
  }
  if (!_ecm.EntityHasComponentType(model_link_, ignition::gazebo::components::WorldLinearVelocity::typeId))
  {
    _ecm.CreateComponent(model_link_, ignition::gazebo::components::WorldLinearVelocity());
  }

  pub_gps_ = this->node.Advertise<sensor_msgs::msgs::SITLGps>("/" + model_.Name(_ecm) + "/sensor" + gps_topic_);
}

void GpsPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
}

void GpsPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &_ecm)
{
  const std::chrono::steady_clock::duration current_time = _info.simTime;
  const double dt = std::chrono::duration<double>(current_time - last_pub_time_).count();
  if (dt > 1.0 / pub_rate_)
  {

    const ignition::gazebo::components::WorldPose *pComp = _ecm.Component<ignition::gazebo::components::WorldPose>(model_link_);
    const ignition::math::Pose3d T_W_I = pComp->Data();
    // Use the model world position for GPS
    const ignition::math::Vector3d &pos_W_I = T_W_I.Pos();
    const ignition::math::Quaterniond &att_W_I = T_W_I.Rot();

    // Use the models' world position for GPS velocity.
    const ignition::gazebo::components::WorldLinearVelocity *vComp = _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(model_link_);
    ignition::math::Vector3d velocity_current_W = vComp->Data();
    ; // = model_->WorldLinearVel();

    ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
    velocity_current_W_xy.Z() = 0;

    // update noise parameters if gps_noise_ is set
    if (gps_noise_)
    {
      noise_gps_pos_.X() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
      noise_gps_pos_.Y() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
      noise_gps_pos_.Z() = gps_z_noise_density_ * sqrt(dt) * randn_(rand_);
      noise_gps_vel_.X() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
      noise_gps_vel_.Y() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
      noise_gps_vel_.Z() = gps_vz_noise_density_ * sqrt(dt) * randn_(rand_);
      random_walk_gps_.X() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
      random_walk_gps_.Y() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
      random_walk_gps_.Z() = gps_z_random_walk_ * sqrt(dt) * randn_(rand_);
    }
    else
    {
      noise_gps_pos_.X() = 0.0;
      noise_gps_pos_.Y() = 0.0;
      noise_gps_pos_.Z() = 0.0;
      noise_gps_vel_.X() = 0.0;
      noise_gps_vel_.Y() = 0.0;
      noise_gps_vel_.Z() = 0.0;
      random_walk_gps_.X() = 0.0;
      random_walk_gps_.Y() = 0.0;
      random_walk_gps_.Z() = 0.0;
    }

    // gps bias integration
    gps_bias_.X() += random_walk_gps_.X() * dt - gps_bias_.X() / gps_corellation_time_;
    gps_bias_.Y() += random_walk_gps_.Y() * dt - gps_bias_.Y() / gps_corellation_time_;
    gps_bias_.Z() += random_walk_gps_.Z() * dt - gps_bias_.Z() / gps_corellation_time_;

    // reproject position with noise into geographic coordinates
    auto pos_with_noise = pos_W_I + noise_gps_pos_ + gps_bias_;

    auto latlon = reproject(pos_with_noise, lat_home_, lon_home_, alt_home_);

    // fill SITLGps msg
    sensor_msgs::msgs::SITLGps gps_msg;

    gps_msg.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());
    gps_msg.set_time_utc_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());
    /// TODO: Add start time

    // @note Unfurtonately the Gazebo GpsSensor seems to provide bad readings,
    // starting to drift and leading to global position loss
    // gps_msg.set_latitude_deg(parentSensor_->Latitude().Degree());
    // gps_msg.set_longitude_deg(parentSensor_->Longitude().Degree());
    // gps_msg.set_altitude(parentSensor_->Altitude());
    gps_msg.set_latitude_deg(latlon.first * 180.0 / M_PI);
    gps_msg.set_longitude_deg(latlon.second * 180.0 / M_PI);
    gps_msg.set_altitude(pos_W_I.Z() + alt_home_ - noise_gps_pos_.Z() + gps_bias_.Z());

    std_xy_ = 1.0;
    std_z_ = 1.0;
    gps_msg.set_eph(std_xy_);
    gps_msg.set_epv(std_z_);

    gps_msg.set_velocity_east(velocity_current_W.X() + noise_gps_vel_.Y());
    gps_msg.set_velocity(velocity_current_W_xy.Length());
    gps_msg.set_velocity_north(velocity_current_W.Y() + noise_gps_vel_.X());
    gps_msg.set_velocity_up(velocity_current_W.Z() - noise_gps_vel_.Z());

    pub_gps_.Publish(gps_msg);
    last_pub_time_ = current_time;
  }
}
