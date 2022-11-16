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

#ifndef IMU_PLUGIN_HH_
#define IMU_PLUGIN_HH_

#include <random>

#include <Eigen/Core>

#include <Imu.pb.h>

#include <ignition/transport/Node.hh>
#include <ignition/math.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/System.hh>

#include <common.h>

namespace imu_plugin
{

    static constexpr auto kDefaultLinkName = "imu_link";
    static constexpr auto kDefaultImuTopic = "/imu";
    static constexpr unsigned int kDefaultPubRate = 250.0; // [Hz]. Note: corresponds to most of the mag devices supported in PX4

    // Default values for use with ADIS16448 IMU
    static constexpr double kDefaultAdisGyroscopeNoiseDensity = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
    static constexpr double kDefaultAdisGyroscopeRandomWalk = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
    static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime = 1.0e+3;
    static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma = 0.5 / 180.0 * M_PI;
    static constexpr double kDefaultAdisAccelerometerNoiseDensity = 2.0 * 2.0e-3;
    static constexpr double kDefaultAdisAccelerometerRandomWalk = 2.0 * 3.0e-3;
    static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime = 300.0;
    static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma = 20.0e-3 * 9.8;
    // Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
    static constexpr double kDefaultGravityMagnitude = 9.8068;

    // A description of the parameters:
    // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
    // TODO(burrimi): Should I have a minimalistic description of the params here?
    struct ImuParameters
    {
        /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
        double gyroscope_noise_density;
        /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
        double gyroscope_random_walk;
        /// Gyroscope bias correlation time constant [s]
        double gyroscope_bias_correlation_time;
        /// Gyroscope turn on bias standard deviation [rad/s]
        double gyroscope_turn_on_bias_sigma;
        /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
        double accelerometer_noise_density;
        /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
        double accelerometer_random_walk;
        /// Accelerometer bias correlation time constant [s]
        double accelerometer_bias_correlation_time;
        /// Accelerometer turn on bias standard deviation [m/s^2]
        double accelerometer_turn_on_bias_sigma;
        /// Norm of the gravitational acceleration [m/s^2]
        double gravity_magnitude;
    };

    class IGNITION_GAZEBO_VISIBLE ImuPlugin : public ignition::gazebo::System,
                                              public ignition::gazebo::ISystemConfigure,
                                              public ignition::gazebo::ISystemPreUpdate,
                                              public ignition::gazebo::ISystemPostUpdate
    {
    public:
        ImuPlugin();

    public:
        ~ImuPlugin() override;

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
        void addNoise(
            Eigen::Vector3d &linear_acceleration,
            Eigen::Vector3d &angular_velocity,
            const double dt);

        std::string imu_topic_{kDefaultImuTopic};
        unsigned int pub_rate_{kDefaultPubRate};

        ignition::transport::Node node;
        ignition::transport::Node::Publisher pub_imu_;
        std::string frame_id_;
        std::string link_name_;

        std::default_random_engine random_generator_;
        std::normal_distribution<double> standard_normal_distribution_;

        ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
        ignition::gazebo::Entity model_link_{ignition::gazebo::kNullEntity};

        std::chrono::steady_clock::duration last_pub_time_{0};

        sensor_msgs::msgs::Imu imu_message_;

        ignition::math::Vector3d gravity_W_;

        Eigen::Vector3d gyroscope_bias_;
        Eigen::Vector3d accelerometer_bias_;

        ImuParameters imu_parameters_;

        uint64_t seq_ = 0;
    }; // class ImuPlugin
} // namespace imu_plugin

#endif // IMU_PLUGIN_HH_