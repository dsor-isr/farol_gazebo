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

// This source code is derived from gazebo_ros_pkgs
//   (https://github.com/ros-simulation/gazebo_ros_pkgs)
// * Copyright 2012 Open Source Robotics Foundation,
// licensed under the Apache-2.0 license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// The original code was modified to:
// - be more consistent with other sensor plugins within uuv_simulator,
// - adhere to Gazebo's coding standards.

#include <uuv_sensor_ros_plugins/PoseGTROSPlugin.hh>
#include <dsor_utils/frames.hpp>
#include <Eigen/Dense>

namespace gazebo {


PoseGTROSPlugin::PoseGTROSPlugin() : ROSBaseModelPlugin() {
  
  this->offset.Pos() = ignition::math::Vector3d::Zero;
  this->offset.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));

  // Initialize the reference's velocity and acceleration vectors
  this->refLinAcc = ignition::math::Vector3d::Zero;
  this->refAngAcc = ignition::math::Vector3d::Zero;

  this->nedTransform = ignition::math::Pose3d::Zero;
  this->nedTransformIsInit = true;
}


PoseGTROSPlugin::~PoseGTROSPlugin() {}


void PoseGTROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  ROSBaseModelPlugin::Load(_model, _sdf);

  ignition::math::Vector3d vec;
  
  GetSDFParam<ignition::math::Vector3d>(_sdf, "position_offset", vec, ignition::math::Vector3d::Zero);
  this->offset.Pos() = vec;
  
  GetSDFParam<ignition::math::Vector3d>(_sdf, "orientation_offset", vec, ignition::math::Vector3d::Zero);
  this->offset.Rot() = ignition::math::Quaterniond(vec);

  GetSDFParam<bool>(_sdf, "publish_ned_odom", this->publishNEDOdom, false);

  // Initialize the reference's velocity and acceleration vectors
  if (!this->referenceLink) {
    this->lastRefLinVel = ignition::math::Vector3d::Zero;
    this->lastRefAngVel = ignition::math::Vector3d::Zero;
  } else {
    this->lastRefLinVel = this->referenceLink->WorldLinearVel();
    this->lastRefAngVel = this->referenceLink->WorldAngularVel();
  }

  this->tfListener.reset(new tf2_ros::TransformListener(this->tfBuffer));
  this->rosSensorOutputPub = this->rosNode->advertise<nav_msgs::Odometry>(this->sensorOutputTopic, 1);
}


bool PoseGTROSPlugin::OnUpdate(const common::UpdateInfo& _info) {
  
  if (!this->EnableMeasurement(_info)) return false;

  // Read the current simulation time
  common::Time curTime = this->world->SimTime();
  double dt = curTime.Double() - this->lastMeasurementTime.Double();

  if (dt <= 0) return false;

  ignition::math::Pose3d linkPose, refLinkPose;
  ignition::math::Vector3d refLinVel, refAngVel;
  ignition::math::Vector3d linkLinVel, linkAngVel;

  this->UpdateNEDTransform();

  // Read sensor link's current pose and velocity
  linkLinVel = this->link->WorldLinearVel();
  linkAngVel = this->link->WorldAngularVel();
  linkPose = this->link->WorldPose();

  this->UpdateReferenceFramePose();

  // Update the reference frame in case it is given as a Gazebo link and
  // read the reference link's linear and angular velocity vectors
  if (this->referenceLink) {
    refLinVel = this->referenceLink->WorldLinearVel();
    refAngVel = this->referenceLink->WorldAngularVel();

    this->referenceFrame = this->referenceLink->WorldPose();
  } else {
    
    // If no Gazebo link is given as a reference, the linear and angular
    // velocity vectors are set to zero
    refLinVel = ignition::math::Vector3d::Zero;
    refAngVel = ignition::math::Vector3d::Zero;
  }

  // Transform pose and velocity vectors to be represented wrt the
  // reference link provided
  linkLinVel -= refLinVel;
  linkAngVel -= refAngVel;

  // Add noise to the link's linear velocity
  linkLinVel += ignition::math::Vector3d(
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp));

  // Add noise to the link's angular velocity
  linkAngVel += ignition::math::Vector3d(
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp));

  // Publish the odometry message of the base_link wrt Gazebo's ENU
  // inertial reference frame
  this->PublishOdomMessage(curTime, linkPose, linkLinVel, linkAngVel);

  // Store the time stamp for this measurement
  this->lastMeasurementTime = curTime;
  return true;
}

void PoseGTROSPlugin::PublishOdomMessage(common::Time _time, ignition::math::Pose3d _pose, ignition::math::Vector3d _linVel, ignition::math::Vector3d _angVel) {
  
  // Generates an Odometry message with the 
  nav_msgs::Odometry odomMsg;

  // Initialize header of the odometry message
  odomMsg.header.frame_id = "world";

  // Get the current time from ros
  ros::Time currentTime = ros::Time().now();

  odomMsg.header.stamp.sec = currentTime.sec;
  odomMsg.header.stamp.nsec = currentTime.nsec;
  odomMsg.child_frame_id = this->link->GetName();

  // Apply pose offset
  _pose += this->offset;

  // Get the position in ENU (inertial frame) and convert to NED (inertial frame)
  Eigen::Vector3d position_enu(_pose.Pos().X(), _pose.Pos().Y(), _pose.Pos().Z());
  Eigen::Vector3d position_ned;
  position_ned = DSOR::transform_vect_inertial_enu_ned(position_enu);

  // Get the orientation in ENU and convert to NED
  Eigen::Quaterniond orientation_enu;
  orientation_enu.x() = _pose.Rot().X();
  orientation_enu.y() = _pose.Rot().Y();
  orientation_enu.z() = _pose.Rot().Z();
  orientation_enu.w() = _pose.Rot().W();
  Eigen::Quaterniond orientation_ned;
  orientation_ned = DSOR::rot_body_to_inertial(orientation_enu);

  // Rotate the velocity from vehicle_ENU (aka body ENU) to vehicle_NED (aka body NED)
  Eigen::Vector3d velocity_vehicle_enu(_linVel.X(), _linVel.Y(), _linVel.Z());
  Eigen::Vector3d velocity_vehicle_ned;
  velocity_vehicle_ned = DSOR::transform_vect_body_enu_ned(velocity_vehicle_enu);

  // Compute the body angular velocity in (body NED)
  Eigen::Vector3d angular_velocity_enu(_angVel.X(), _angVel.Y(), _angVel.Z());
  Eigen::Vector3d angular_velocity_ned;
  angular_velocity_ned = DSOR::transform_vect_body_enu_ned(angular_velocity_enu);

  // Fill out the messages
  odomMsg.pose.pose.position.x = position_ned.x();
  odomMsg.pose.pose.position.y = position_ned.y();
  odomMsg.pose.pose.position.z = position_ned.z();

  odomMsg.pose.pose.orientation.x = orientation_ned.x();
  odomMsg.pose.pose.orientation.y = orientation_ned.y();
  odomMsg.pose.pose.orientation.z = orientation_ned.z();
  odomMsg.pose.pose.orientation.w = orientation_ned.w();

  odomMsg.twist.twist.linear.x = velocity_vehicle_ned.x();
  odomMsg.twist.twist.linear.y = velocity_vehicle_ned.y();
  odomMsg.twist.twist.linear.z = velocity_vehicle_ned.z();

  odomMsg.twist.twist.angular.x = angular_velocity_ned.x();
  odomMsg.twist.twist.angular.y = angular_velocity_ned.y();
  odomMsg.twist.twist.angular.z = angular_velocity_ned.z();

  // Fill in the covariance matrix
  double gn2 = this->noiseSigma * this->noiseSigma;
  odomMsg.pose.covariance[0] = gn2;
  odomMsg.pose.covariance[7] = gn2;
  odomMsg.pose.covariance[14] = gn2;
  odomMsg.pose.covariance[21] = gn2;
  odomMsg.pose.covariance[28] = gn2;
  odomMsg.pose.covariance[35] = gn2;

  odomMsg.twist.covariance[0] = gn2;
  odomMsg.twist.covariance[7] = gn2;
  odomMsg.twist.covariance[14] = gn2;
  odomMsg.twist.covariance[21] = gn2;
  odomMsg.twist.covariance[28] = gn2;
  odomMsg.twist.covariance[35] = gn2;

  this->rosSensorOutputPub.publish(odomMsg);
}


void PoseGTROSPlugin::UpdateNEDTransform() {
  
  if (!this->publishNEDOdom) return;
  if (this->nedTransformIsInit) return;

  geometry_msgs::TransformStamped childTransform;
  std::string targetFrame = this->nedFrameID;
  std::string sourceFrame = this->link->GetName();
  
  try {
    childTransform = this->tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
  } catch(tf2::TransformException &ex) {
    gzmsg << "Transform between " << targetFrame << " and " << sourceFrame << std::endl;
    gzmsg << ex.what() << std::endl;
    return;
  }

  this->nedTransform.Pos() = ignition::math::Vector3d(
    childTransform.transform.translation.x,
    childTransform.transform.translation.y,
    childTransform.transform.translation.z);
  this->nedTransform.Rot() = ignition::math::Quaterniond(
    childTransform.transform.rotation.w,
    childTransform.transform.rotation.x,
    childTransform.transform.rotation.y,
    childTransform.transform.rotation.z);

  this->nedTransformIsInit = true;
}

GZ_REGISTER_MODEL_PLUGIN(PoseGTROSPlugin)

}
