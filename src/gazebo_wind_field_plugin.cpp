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


#include "gazebo_wind_field_plugin.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}
;

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(_sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<double>(_sdf, "windAzimuth", wind_azimuth_, wind_azimuth_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(_sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(_sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<double>(_sdf, "windGustAzimuth", wind_gust_azimuth_, wind_gust_azimuth_);

  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));
  wind_pub_ = node_handle_->Advertise<wind_field_msgs::msgs::WindField>(wind_pub_topic_, 1);
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  // Calculate the wind velocity.
  double wind_velocity = wind_velocity_mean_;

  math::Vector3 wind_gust(0, 0, 0);
  // Calculate the wind gust velocity.
  double wind_gust_velocity = 0;
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    wind_gust_velocity = wind_gust_velocity_mean_;
  }
  // Add the wind gust to the default wind
  double wind_vel_x = sin(wind_azimuth_)*wind_velocity + sin(wind_gust_azimuth_)*wind_gust_velocity;
  double wind_vel_y = cos(wind_azimuth_)*wind_velocity + cos(wind_gust_azimuth_)*wind_gust_velocity;

  double wind_total_velocity = sqrt(wind_vel_x*wind_vel_x + wind_vel_y*wind_vel_y);
  double wind_total_azimuth  = atan2(wind_vel_x,wind_vel_y);

  wind_field_msgs::msgs::WindField wind_msg;

  wind_msg.set_frame_id(frame_id_);
  Set(wind_msg.mutable_stamp(), now);
  wind_msg.set_azimuth(wind_total_azimuth);
  wind_msg.set_velocity(wind_total_velocity);

  wind_pub_->Publish(wind_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}
