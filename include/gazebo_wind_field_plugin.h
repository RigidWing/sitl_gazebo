/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
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


#ifndef GAZEBO_WIND_FIELD_PLUGIN_H
#define GAZEBO_WIND_FIELD_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// KITEPOWER (Xander)
#include "WindField.pb.h"
#include "common.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";

static constexpr double kDefaultWindVelocityMean = 0.0;
static constexpr double kDefaultWindVelocityVariance = 0.0;
static constexpr double kDefaultWindGustVelocityMean = 0.0;
static constexpr double kDefaultWindGustVelocityVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static constexpr double kDefaultWindAzimuth = 0.0;
static constexpr double kDefaultWindGustAzimuth = M_PI/4;

// KITEPOWER (Xander)
static const std::string kDefaultWindFieldPubTopic= "/wind_field";



/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public ModelPlugin {
 public:
  GazeboWindPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_(kDefaultWindFieldPubTopic),
        wind_velocity_mean_(kDefaultWindVelocityMean),
        wind_velocity_variance_(kDefaultWindVelocityVariance),
        wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
        wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
        wind_azimuth_(kDefaultWindAzimuth),
        wind_gust_azimuth_(kDefaultWindGustAzimuth),
        frame_id_(kDefaultFrameId),
        node_handle_(NULL) {}

  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
    /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_pub_topic_;

  double wind_velocity_mean_;
  double wind_velocity_variance_;
  double wind_gust_velocity_mean_;
  double wind_gust_velocity_variance_;

  double wind_azimuth_;
  double wind_gust_azimuth_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;
};
}

#endif // GAZEBO_WIND_FIELD_PLUGIN_H
