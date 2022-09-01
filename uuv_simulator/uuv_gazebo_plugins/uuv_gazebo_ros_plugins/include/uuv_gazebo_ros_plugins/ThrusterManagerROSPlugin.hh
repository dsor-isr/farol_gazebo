/**
 * Authors:
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: Andre Potes (andre.potes@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: ThrusterManager.hpp 
 * Brief: Defines a thruster manager ROS plugin that translates the desired forces for the vehicle thrusters, from the 
 *        DSOR stack to the UUV simulator
 */
#pragma once

#include <boost/scoped_ptr.hpp>

#include <map>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <dsor_msgs/Thruster.h>

#include <sdf/sdf.hh>

namespace gazebo {

class ThrusterManagerROSPlugin : public ModelPlugin {

  /// \brief Constrcutor.
  public: ThrusterManagerROSPlugin();

  /// \brief Destructor.
  public: ~ThrusterManagerROSPlugin();

  /// \brief Load module and read parameters from SDF.
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update the simulation state
  /// \param[in] _info Information used in the update event.
  public: void Update(const common::UpdateInfo &_info);

    /// \brief Callback for the input topic subscriber
  protected: void SetThrustReference(const dsor_msgs::Thruster &_msg);

  /// \brief Pointer to this ROS node's handle.
  protected: boost::scoped_ptr<ros::NodeHandle> rosNode;

  /// \brief Gazebo node
  protected: transport::NodePtr node;

  /// \brief Gazebo update event
  protected: event::ConnectionPtr updateConnection;

  /// \brief Subscriber to the reference signal topic from DSOR stack.
  protected: ros::Subscriber thrusterStackSubscriber;

  /// \brief Vector of publishers to the output thrust topic for each thruster (ros)
  protected: std::vector<ros::Publisher> thrustInputPublisher;

  /// \brief: Number of thrusters in vehicle.
  protected: int thrustersNumber;

  /// \brief: Thrusters topic from DSOR stack.
  protected: std::string dsorTopicThrusters;

  /// \brief: Vector of thrusters received from DSOR stack
  protected: std::vector<double> thrustVector;
};
}