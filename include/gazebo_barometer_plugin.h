
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

#ifndef BAROMETER_PLUGIN_HH_
#define BAROMETER_PLUGIN_HH_

#include <random>

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>

#include <Pressure.pb.h>
#include <common.h>

static constexpr auto kDefaultBarometerTopic = "/baro";
static constexpr auto kDefaultPubRate = 50; // [Hz]. Note: averages the supported Baro device ODR in PX4

// International standard atmosphere (troposphere model - valid up to 11km) see [1]
static constexpr auto kDefaultTemperatureMsl = 288.15; // Temperature at MSL [K] (15 [C])
static constexpr auto kDefaultPressureMsl = 101325.0;  // Pressure at MSL [Pa]
static constexpr auto kDefaultLapseRate = 0.0065;      // Reduction in temperature with altitude for troposphere [K/m]
static constexpr auto kDefaultAirDensityMsl = 1.225;   // Air density at MSL [kg/m^3]
static constexpr auto kDefaultAbsoluteZeroC = -273.15; // [C]

// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr auto kDefaultGravityMagnitude = 9.8068;

// Default values for baro pressure sensor random noise generator
static constexpr auto kDefaultBaroRndY2 = 0.0;
static constexpr auto kDefaultBaroRndUseLast = false;
static constexpr auto kDefaultBaroDriftPaPerSec = 0.0;
static constexpr auto kDefaultBaroDriftPa = 0.0;

namespace barometer_plugin
{
  class IGNITION_GAZEBO_VISIBLE BarometerPlugin :
      // This is class a system.
      public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
  {
  public:
    BarometerPlugin();

  public:
    ~BarometerPlugin() override;

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
    void addNoise(double &absolute_pressure, double &pressure_altitude, double &temperature_local, const double dt);
    void getSdfParams(const std::shared_ptr<const sdf::Element> &sdf);

    ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

    std::chrono::steady_clock::duration last_pub_time_{0};

    std::string link_name_;
    std::string baro_topic_{kDefaultBarometerTopic};
    unsigned int pub_rate_{kDefaultPubRate};

    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;

    ignition::math::Pose3d pose_model_start_;
    double gravity_magnitude_{kDefaultGravityMagnitude};
    double alt_home_{kDefaultHomeAltitude};

    ignition::transport::Node node;
    ignition::transport::Node::Publisher pub_baro_;

    sensor_msgs::msgs::Pressure baro_msg_;

    // State variables for baro pressure sensor random noise generator
    double baro_rnd_y2_{kDefaultBaroRndY2};
    bool baro_rnd_use_last_{kDefaultBaroRndUseLast};
    double baro_drift_pa_per_sec_{kDefaultBaroDriftPaPerSec};
    double baro_drift_pa_{kDefaultBaroDriftPa};
  }; // class BarometerPlugin
} // namespace barometer_plugin

#endif // BAROMETER_PLUGIN_HH_
