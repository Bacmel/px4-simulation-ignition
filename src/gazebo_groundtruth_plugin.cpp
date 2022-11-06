/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @brief Groundtruth Plugin
 *
 * This plugin gets and publishes ground-truth data
 *
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "gazebo_groundtruth_plugin.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    groundtruth_plugin::GroundtruthPlugin,
    ignition::gazebo::System,
    groundtruth_plugin::GroundtruthPlugin::ISystemConfigure,
    groundtruth_plugin::GroundtruthPlugin::ISystemPreUpdate,
    groundtruth_plugin::GroundtruthPlugin::ISystemPostUpdate)

using namespace groundtruth_plugin;

GroundtruthPlugin::GroundtruthPlugin()
{
}

GroundtruthPlugin::~GroundtruthPlugin()
{
}

void GroundtruthPlugin::getSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{
    // Use environment variables if set for home position.
    const char *env_lat = std::getenv("PX4_HOME_LAT");
    const char *env_lon = std::getenv("PX4_HOME_LON");
    const char *env_alt = std::getenv("PX4_HOME_ALT");

    if (env_lat)
    {
        lat_home_ = std::stod(env_lat) * M_PI / 180.0;
        ignmsg << "[gazebo_groundtruth_plugin] Home latitude is set to " << lat_home_ << " m\n";
    }
    else if (sdf->HasElement("homeLatitude"))
    {
        lat_home_ = sdf->Get<double>("homeLatitude");
        lat_home_ = lat_home_ * M_PI / 180.0;
    }
    else
    {
        lat_home_ = kDefaultHomeLatitude;
        ignwarn << "[gazebo_groundtruth_plugin] Using default home latitude of " << lat_home_ << " m\n";
    }

    if (env_lon)
    {
        lon_home_ = std::stod(env_lon) * M_PI / 180.0;
        ignmsg << "[gazebo_groundtruth_plugin] Home longitude is set to " << lon_home_ << " m\n";
    }
    else if (sdf->HasElement("homeLongitude"))
    {
        lon_home_ = sdf->Get<double>("homeLongitude");
        lon_home_ = lon_home_ * M_PI / 180.0;
    }
    else
    {
        lon_home_ = kDefaultHomeLongitude;
        ignwarn << "[gazebo_groundtruth_plugin] Using default home longitude of " << lon_home_ << " m\n";
    }

    if (env_alt)
    {
        alt_home_ = std::stod(env_alt);
        ignmsg << "[gazebo_groundtruth_plugin] Home altitude is set to " << alt_home_ << " m\n";
    }
    else if (sdf->HasElement("homeAltitude"))
    {
        alt_home_ = sdf->Get<double>("homeAltitude");
    }
    else
    {
        alt_home_ = kDefaultHomeAltitude;
        &ignwarn << "[gazebo_groundtruth_plugin] Using default home altitude of " << alt_home_ << " m\n";
    }

    link_name_ = sdf->Get<std::string>("link_name");
}

void GroundtruthPlugin::Configure(const ignition::gazebo::Entity &_entity,
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

    pub_gt_ = this->node.Advertise<sensor_msgs::msgs::Groundtruth>("/" + model_.Name(_ecm) + gt_topic_);
}

void GroundtruthPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm)
{
}

void GroundtruthPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                   const ignition::gazebo::EntityComponentManager &_ecm)
{
    const std::chrono::steady_clock::duration current_time = _info.simTime;

    // Get pose of the model that the plugin is attached to
    const ignition::gazebo::components::WorldPose *pT_W_I = _ecm.Component<ignition::gazebo::components::WorldPose>(model_link_);
    const ignition::math::Pose3d T_W_I = pT_W_I->Data();
    // Use the models world position and attitude for groundtruth
    ignition::math::Vector3d &pos_W_I = T_W_I.Pos();
    ignition::math::Quaterniond &att_W_I = T_W_I.Rot();

    // reproject position into geographic coordinates
    auto latlon_gt = reproject(pos_W_I, lat_home_, lon_home_, alt_home_);

    // Use the models' world position for groundtruth velocity.
    const ignition::gazebo::components::WorldLinearVelocity *pvelocity_current_W = _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(model_link_);
    const ignition::math::Vector3d velocity_current_W = pvelocity_current_W->Data();

    ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
    velocity_current_W_xy.Z() = 0;

    // fill Groundtruth msg
    gt_msg_.set_time_usec(std::chrono::duration_cast<std::chrono::microseconds>(current_time).count());
    gt_msg_.set_latitude_rad(latlon_gt.first);
    gt_msg_.set_longitude_rad(latlon_gt.second);
    gt_msg_.set_altitude(pos_W_I.Z() + alt_home_);
    gt_msg_.set_velocity_east(velocity_current_W.X());
    gt_msg_.set_velocity_north(velocity_current_W.Y());
    gt_msg_.set_velocity_up(velocity_current_W.Z());
    gt_msg_.set_attitude_q_w(att_W_I.W());
    gt_msg_.set_attitude_q_x(att_W_I.X());
    gt_msg_.set_attitude_q_y(att_W_I.Y());
    gt_msg_.set_attitude_q_z(att_W_I.Z());

    // Publish Groundtruth msg
    pub_gt_.Publish(gt_msg_);
}
