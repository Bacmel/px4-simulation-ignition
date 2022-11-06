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

#ifndef GROUNDTRUTH_PLUGIN_HH_
#define GROUNDTRUTH_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>

#include <Groundtruth.pb.h>
#include <common.h>

namespace groundtruth_plugin
{
  class IGNITION_GAZEBO_VISIBLE GroundtruthPlugin :
      // This is class a system.
      public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
  {
  public:
    GroundtruthPlugin();

  public:
    ~GroundtruthPlugin() override;

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

    ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

    std::string link_name_;
    std::string gt_topic_{"/groundtruth"};

    double lat_home_{kDefaultHomeLatitude};
    double lon_home_{kDefaultHomeLongitude};
    double alt_home_{kDefaultHomeAltitude};

    ignition::transport::Node node;
    ignition::transport::Node::Publisher pub_gt_;

    sensor_msgs::msgs::Groundtruth gt_msg_;

  }; // class GroundtruthPlugin
} // namespace groundtruth_plugin

#endif // GROUNDTRUTH_PLUGIN_HH_
