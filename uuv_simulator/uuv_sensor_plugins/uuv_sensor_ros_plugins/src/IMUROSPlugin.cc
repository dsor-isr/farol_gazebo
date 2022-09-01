// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

// This source code is derived from rotors_simulator
//   (https://github.com/ethz-asl/rotors_simulator)
// * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland,
// * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland,mlicensed under the Apache-2.0 license, cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - adhere to Gazebo's coding standards.

#include <uuv_sensor_ros_plugins/IMUROSPlugin.hh>

// Used to convert a quaternion to euler angles
#include <dsor_utils/rotations.hpp>
#include <dsor_utils/frames.hpp>

namespace gazebo {

IMUROSPlugin::IMUROSPlugin() : ROSBaseModelPlugin() {}
IMUROSPlugin::~IMUROSPlugin() {}

void IMUROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
  // Call the base load method
  ROSBaseModelPlugin::Load(_model, _sdf);

  // Only need to load settings specific to this sensor.
  GetSDFParam<double>(_sdf, "gyroscope_noise_density", this->imuParameters.gyroscopeNoiseDensity, this->imuParameters.gyroscopeNoiseDensity);
  GetSDFParam<double>(_sdf, "gyroscope_bias_random_walk", this->imuParameters.gyroscopeRandomWalk, this->imuParameters.gyroscopeRandomWalk);
  GetSDFParam<double>(_sdf, "gyroscope_bias_correlation_time", this->imuParameters.gyroscopeBiasCorrelationTime, this->imuParameters.gyroscopeBiasCorrelationTime);
  GZ_ASSERT(this->imuParameters.gyroscopeBiasCorrelationTime > 0.0, "Gyroscope bias correlation time must be greater than zero");
  
  GetSDFParam<double>(_sdf, "gyroscope_turn_on_bias_sigma", this->imuParameters.gyroscopeTurnOnBiasSigma, this->imuParameters.gyroscopeTurnOnBiasSigma);
  GetSDFParam<double>(_sdf, "accelerometer_noise_density", this->imuParameters.accelerometerNoiseDensity, this->imuParameters.accelerometerNoiseDensity);
  GetSDFParam<double>(_sdf, "accelerometer_random_walk", this->imuParameters.accelerometerRandomWalk, this->imuParameters.accelerometerRandomWalk);
  GetSDFParam<double>(_sdf, "accelerometer_bias_correlation_time", this->imuParameters.accelerometerBiasCorrelationTime, this->imuParameters.accelerometerBiasCorrelationTime);
  GZ_ASSERT(this->imuParameters.accelerometerBiasCorrelationTime > 0.0, "Accelerometer bias correlation time must be greater than zero");
  
  GetSDFParam<double>(_sdf, "accelerometer_turn_on_bias_sigma", this->imuParameters.accelerometerTurnOnBiasSigma, this->imuParameters.accelerometerTurnOnBiasSigma);
  GetSDFParam<double>(_sdf, "orientation_noise", this->imuParameters.orientationNoise, this->imuParameters.orientationNoise);

  // Fill IMU message with static data
  this->imuROSMessage.header.frame_id = this->link->GetName();

  // We assume uncorrelated noise on the 3 channels -> only set diagonal elements. Only the broadband noise component is considered, specified as
  // a continuous-time density (two-sided spectrum); not the true covariance of the measurements.

  // Angular velocity measurement covariance.
  this->AddNoiseModel("gyro_noise_density", this->imuParameters.gyroscopeNoiseDensity);
  double gyroVar = this->imuParameters.gyroscopeNoiseDensity * this->imuParameters.gyroscopeNoiseDensity;
  this->imuROSMessage.angular_velocity_covariance[0] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[4] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[8] = gyroVar;

  // Linear acceleration measurement covariance.
  this->AddNoiseModel("acc_noise_density", this->imuParameters.accelerometerNoiseDensity);
  double accelVar = this->imuParameters.accelerometerNoiseDensity * this->imuParameters.accelerometerNoiseDensity;
  this->imuROSMessage.linear_acceleration_covariance[0] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[4] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[8] = accelVar;

  // Orientation estimate covariance
  this->AddNoiseModel("orientation_noise_density", this->imuParameters.orientationNoise);
  double orientationVar = this->imuParameters.orientationNoise * this->imuParameters.orientationNoise;
  this->imuROSMessage.orientation_covariance[0] = orientationVar;
  this->imuROSMessage.orientation_covariance[4] = orientationVar;
  this->imuROSMessage.orientation_covariance[8] = orientationVar;

  // Add the noise covariance to the Measurement message (used by the farol stack)
  std::vector<double> noise_msg;
  for(int i = 0; i < 3; i++) noise_msg.push_back(orientationVar);
  for(int i = 0; i < 3; i++) noise_msg.push_back(gyroVar);
  this->imuMeasurementMsg.noise = noise_msg;
  this->imuMeasurementMsg.header.frame_id = this->robotNamespace + "_ahrs";

  // Store the acc. gravity vector
  this->gravityWorld = this->world->Gravity();

  double sigmaBonG = this->imuParameters.gyroscopeTurnOnBiasSigma;
  double sigmaBonA = this->imuParameters.accelerometerTurnOnBiasSigma;

  this->AddNoiseModel("gyro_turn_on_bias", sigmaBonG);
  this->AddNoiseModel("acc_turn_on_bias", sigmaBonA);

  // FIXME Add the noise amplitude input for gyroscope
  this->gyroscopeTurnOnBias = ignition::math::Vector3d(
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp));
  
  // FIXME Add the noise amplitude input for accelerometer
  this->accelerometerTurnOnBias = ignition::math::Vector3d(
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp));

  // FIXME incorporate steady-state covariance of bias process
  this->gyroscopeBias = ignition::math::Vector3d::Zero;
  this->accelerometerBias = ignition::math::Vector3d::Zero;

  // Advertise an IMU topic
  this->rosSensorOutputPub = this->rosNode->advertise<sensor_msgs::Imu>(this->sensorOutputTopic, 1);

  // Advertise an orientation topic to be used with the (farol stack)
  std::string imuStackTopic = '/' + this->robotNamespace + "/measurement/orientation";
  this->farolStackImuPub = this->rosNode->advertise<dsor_msgs::Measurement>(imuStackTopic, 1);
}


bool IMUROSPlugin::OnUpdate(const common::UpdateInfo& _info) {
  
  // Publish the sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info)) return false;

  // Read the current simulation time
  common::Time curTime = this->world->SimTime();
  double dt = curTime.Double() - this->lastMeasurementTime.Double();

  // Initialize the vectors that will store all the data
  ignition::math::Pose3d worldLinkPose;
  ignition::math::Vector3d bodyAngVel;
  ignition::math::Vector3d bodyLinAcc, worldLinAcc;
  ignition::math::Vector3d refGravityWorld;

  // Read sensor link's current pose and velocity
  bodyAngVel = this->link->RelativeAngularVel();
  bodyLinAcc = this->link->RelativeLinearAccel();
  worldLinkPose = this->link->WorldPose();

  this->UpdateReferenceFramePose();
  if (this->referenceLink) this->referenceFrame = this->referenceLink->WorldPose();

  // Transform pose and velocity vectors to be represented wrt the reference link provided
  worldLinkPose.Pos() = worldLinkPose.Pos() - this->referenceFrame.Pos();
  worldLinkPose.Pos() = this->referenceFrame.Rot().RotateVectorReverse(worldLinkPose.Pos());
  worldLinkPose.Rot() *= this->referenceFrame.Rot().Inverse();

  ignition::math::Vector3d gravityBody = worldLinkPose.Rot().RotateVectorReverse(this->gravityWorld);

  // Compute the simulated measurements wrt the default world ENU frame
  this->measOrientation = worldLinkPose.Rot();
  this->measAngularVel = bodyAngVel;
  this->measLinearAcc = bodyLinAcc - gravityBody;

  // Add noise and bias to the simulated data
  this->AddNoise(this->measLinearAcc, this->measAngularVel, this->measOrientation, dt);

  // Compute the Orientation in (inertial NED) - for that we must convert both body reference frame and inertial frame to NED
  Eigen::Quaterniond orientation_ned;
  Eigen::Quaterniond orientation_enu;
  orientation_enu.x() = this->measOrientation.X();
  orientation_enu.y() = this->measOrientation.Y();
  orientation_enu.z() = this->measOrientation.Z();
  orientation_enu.w() = this->measOrientation.W();
  orientation_ned = DSOR::rot_body_to_inertial(orientation_enu);

  // Compute the body angular velocity in (body NED)
  Eigen::Vector3d angular_velocity_ned;
  Eigen::Vector3d angular_velocity_enu(measAngularVel.X(), measAngularVel.Y(), measAngularVel.Z());
  angular_velocity_ned = DSOR::transform_vect_body_enu_ned(angular_velocity_enu);

  // Compute the body linear acceleration in (body NED)
  Eigen::Vector3d linear_acceleration_ned;
  Eigen::Vector3d linear_acceleration_enu(measLinearAcc.X(), measLinearAcc.Y(), measLinearAcc.Z());
  linear_acceleration_ned = DSOR::transform_vect_body_enu_ned(linear_acceleration_enu);

  // Get the current ROS time (can be different from simulation time - ROS uses the computer seconds time since unix was created)
  ros::Time currentROSTime = ros::Time().now();

  // Fill the ROS IMU message (using NED standard)
  this->imuROSMessage.header.stamp.sec = currentROSTime.sec;
  this->imuROSMessage.header.stamp.nsec = currentROSTime.nsec;

  this->imuROSMessage.orientation.x = orientation_ned.x();
  this->imuROSMessage.orientation.y = orientation_ned.y();
  this->imuROSMessage.orientation.z = orientation_ned.z();
  this->imuROSMessage.orientation.w = orientation_ned.w();

  this->imuROSMessage.angular_velocity.x = angular_velocity_ned.x();
  this->imuROSMessage.angular_velocity.y = angular_velocity_ned.y();
  this->imuROSMessage.angular_velocity.z = angular_velocity_ned.z();

  this->imuROSMessage.linear_acceleration.x = linear_acceleration_ned.x();
  this->imuROSMessage.linear_acceleration.y = linear_acceleration_ned.y();
  this->imuROSMessage.linear_acceleration.z = linear_acceleration_ned.z();

  this->rosSensorOutputPub.publish(this->imuROSMessage); 

  // Convert the quaternion to euler angles according to Z-Y-X convention
  Eigen::Vector3d euler_angles;
  euler_angles = DSOR::quaternion_to_euler(orientation_ned);

  // Fill the ROS Measurement message (using NED standard) to be used by the Farol stack
  this->imuMeasurementMsg.header.stamp.sec = currentROSTime.sec;
  this->imuMeasurementMsg.header.stamp.nsec = currentROSTime.nsec;
  this->imuMeasurementMsg.value.clear();
  this->imuMeasurementMsg.value.push_back(euler_angles.x());
  this->imuMeasurementMsg.value.push_back(euler_angles.y());
  this->imuMeasurementMsg.value.push_back(euler_angles.z());
  this->imuMeasurementMsg.value.push_back(angular_velocity_ned.x());
  this->imuMeasurementMsg.value.push_back(angular_velocity_ned.y());
  this->imuMeasurementMsg.value.push_back(angular_velocity_ned.z());

  this->farolStackImuPub.publish(this->imuMeasurementMsg);

  this->lastMeasurementTime = curTime;
  return true;
}


void IMUROSPlugin::AddNoise(ignition::math::Vector3d& _linAcc, ignition::math::Vector3d& _angVel, ignition::math::Quaterniond& _orientation, double _dt) {
  
  GZ_ASSERT(_dt > 0.0, "Invalid time step");

  /// Gyroscope
  double tauG = this->imuParameters.gyroscopeBiasCorrelationTime;

  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigmaGD = 1 / sqrt(_dt) * this->imuParameters.gyroscopeNoiseDensity;
  double sigmaBG = this->imuParameters.gyroscopeRandomWalk;
  
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigmaBGD = sqrt(- sigmaBG * sigmaBG * tauG / 2.0 * (exp(-2.0 * _dt / tauG) - 1.0));
  
  // Compute state-transition.
  double phiGD = exp(-1.0 / tauG * _dt);

  // FIXME Add the noise amplitude input for BGD
  this->AddNoiseModel("bgd", sigmaBGD);
  
  // FIXME Add the noise amplitude input for GD
  this->AddNoiseModel("gd", sigmaGD);

  // Simulate gyroscope noise processes and add them to the true angular rate.
  this->gyroscopeBias = phiGD * this->gyroscopeBias + 
    ignition::math::Vector3d(
      this->GetGaussianNoise("bgd", this->noiseAmp),
      this->GetGaussianNoise("bgd", this->noiseAmp),
      this->GetGaussianNoise("bgd", this->noiseAmp));
  _angVel = _angVel + this->gyroscopeBias + this->gyroscopeTurnOnBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("gd", this->noiseAmp),
      this->GetGaussianNoise("gd", this->noiseAmp),
      this->GetGaussianNoise("gd", this->noiseAmp));

  /// Accelerometer
  double tauA = this->imuParameters.accelerometerBiasCorrelationTime;
  
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigmaAD = 1. / sqrt(_dt) * this->imuParameters.accelerometerNoiseDensity;
  double sigmaBA = this->imuParameters.accelerometerRandomWalk;
  
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigmaBAD = sqrt(- sigmaBA * sigmaBA * tauA / 2.0 * (exp(-2.0 * _dt / tauA) - 1.0));
  
  // Compute state-transition.
  double phiAD = exp(-1.0 / tauA * _dt);

  // FIXME Add the noise amplitude input for BAD
  this->AddNoiseModel("bad", sigmaBAD);
  
  // FIXME Add the noise amplitude input for BAD
  this->AddNoiseModel("ad", sigmaAD);

  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  this->accelerometerBias = phiAD * this->accelerometerBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("bad", this->noiseAmp),
      this->GetGaussianNoise("bad", this->noiseAmp),
      this->GetGaussianNoise("bad", this->noiseAmp));
  _linAcc = _linAcc + this->accelerometerBias + this->accelerometerTurnOnBias +
    ignition::math::Vector3d(
      this->GetGaussianNoise("ad", this->noiseAmp),
      this->GetGaussianNoise("ad", this->noiseAmp),
      this->GetGaussianNoise("ad", this->noiseAmp));

  /// Orientation
  // Construct error quaterion using small-angle approximation.
  double scale = 0.5 * this->imuParameters.orientationNoise;

  // Attention: w-xyz
  ignition::math::Quaterniond error(1.0,
    this->GetGaussianNoise("orientation_noise_density", scale),
    this->GetGaussianNoise("orientation_noise_density", scale),
    this->GetGaussianNoise("orientation_noise_density", scale));

  error.Normalize();
  _orientation = _orientation * error;
}

GZ_REGISTER_MODEL_PLUGIN(IMUROSPlugin)
}
