// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uuv_sensor_ros_plugins/SubseaPressureROSPlugin.hh>

namespace gazebo {

SubseaPressureROSPlugin::SubseaPressureROSPlugin() : ROSBaseModelPlugin() {}
SubseaPressureROSPlugin::~SubseaPressureROSPlugin() {}

void SubseaPressureROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
  ROSBaseModelPlugin::Load(_model, _sdf);

  /* Get the sensor parameters from the SDF file */
  GetSDFParam<double>(_sdf, "saturation", this->saturation, 3000);
  GetSDFParam<bool>(_sdf, "estimate_depth_on", this->estimateDepth, true);
  GetSDFParam<double>(_sdf, "standard_pressure", this->standardPressure, 101.325);
  GetSDFParam<double>(_sdf, "kPa_per_meter", this->kPaPerM, 9.80638);

  /* Advertise on the fluid pressure topic */
  this->rosSensorOutputPub = this->rosNode->advertise<sensor_msgs::FluidPressure>(this->sensorOutputTopic, 1);

  /* Advertise on a measurement position topic (farol stack) */
  std::string depthStackTopic = '/' + this->robotNamespace + "/measurement/position";
  this->farolStackDepthPub = this->rosNode->advertise<dsor_msgs::Measurement>(depthStackTopic, 1);

  /* Initiate the farol measurement message with the noise covariance */
  this->depthMeasurementMsg.noise.push_back(this->noiseAmp);
  this->depthMeasurementMsg.header.frame_id = this->robotNamespace + "_depth";
}


bool SubseaPressureROSPlugin::OnUpdate(const common::UpdateInfo& _info) {
  
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  // Using the world pose wrt Gazebo's ENU reference frame
  ignition::math::Vector3d pos;
  pos = this->link->WorldPose().Pos();

  double depth = std::abs(pos.Z());
  double pressure = this->standardPressure;
  
  // Convert depth to pressure
  if (depth >= 0) pressure += depth * this->kPaPerM;
  pressure += this->GetGaussianNoise(this->noiseAmp);
 
  // Estimate depth, if enabled
  double inferredDepth = 0.0;
  if (this->estimateDepth) {
    inferredDepth = (pressure - this->standardPressure) / this->kPaPerM;
  
    // Publish the estimated depth to the farol stack
    // Get the current ROS time (can be different from simulation time - ROS uses the computer seconds time since unix was created)
    ros::Time currentROSTime = ros::Time().now();
    this->depthMeasurementMsg.header.stamp.sec = currentROSTime.sec;
    this->depthMeasurementMsg.header.stamp.nsec = currentROSTime.nsec;
    this->depthMeasurementMsg.value.clear();
    this->depthMeasurementMsg.value.push_back(inferredDepth);
    this->farolStackDepthPub.publish(this->depthMeasurementMsg);
  }

  // Publish ROS pressure message
  sensor_msgs::FluidPressure rosMsg;
  rosMsg.header.stamp.sec  = _info.simTime.sec;
  rosMsg.header.stamp.nsec = _info.simTime.nsec;
  rosMsg.header.frame_id = this->link->GetName();

  rosMsg.fluid_pressure = pressure;
  rosMsg.variance = this->noiseSigma * this->noiseSigma;

  this->rosSensorOutputPub.publish(rosMsg);

  // Read the current simulation time
  this->lastMeasurementTime = this->world->SimTime();

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(SubseaPressureROSPlugin)
}
