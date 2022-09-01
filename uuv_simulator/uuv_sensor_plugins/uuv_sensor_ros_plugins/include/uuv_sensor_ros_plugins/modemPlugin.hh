#ifndef GAZEBO_MODEM_PLUGIN_HPP_
#define GAZEBO_MODEM_PLUGIN_HPP_

#include <thread>
#include <random>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include "uuv_sensor_ros_plugins_msgs/modemLocation.h"
#include "dmac/mUSBLFix.h"
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
  class modemPlugin : public ModelPlugin
  {
    public:
      modemPlugin();
      ~modemPlugin();

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void iisRosCallback(uuv_sensor_ros_plugins_msgs::modemLocationConstPtr modem_position);
      void cisRosCallback(uuv_sensor_ros_plugins_msgs::modemLocationConstPtr modem_position);
      void payloadToTransmitCallback(const std_msgs::StringConstPtr &payload);
      void queueThread();

      void publishPosition(double &bearing, double &range, double &elevation);
      void calcuateRelativePose(ignition::math::Vector3d position, double &bearing, double &range, double &elevation);

    private:
      std::string m_namespace;
      std::string m_modemDevice;
      std::string m_modemID;
      std::string m_usblDevice;
      std::string m_usblID;
      std::string m_modemAttachedObject;
      std::string m_usblAttachedObject;
      std::string m_delayUntilAnswer;
      
      std::string aux_usbl_id;
      std::string aux_usbl_frame_id;

      // environment variables
      double m_temperature;
      double m_soundSpeed;
      double m_noiseMu;
      double m_noiseSigma;
      bool m_hasUSBL;

      // Gazebo nodes, publishers, and subscribers
      physics::ModelPtr m_model;

      // ROS nodes, publishers and subscibers
      ros::Publisher m_commandResponsePub;
      ros::Publisher m_trigger_serialization;
      ros::Publisher m_payload_to_deserialize;
      ros::Publisher m_globalPosPub2;
      ros::Subscriber m_iisSub;
      ros::Subscriber m_cisSub;
      ros::Subscriber m_commandSub;
      ros::Subscriber m_payload_to_transmit;
      ros::CallbackQueue m_rosQueue;

      ros::Publisher m_usbl_fix;

      std::unique_ptr<ros::NodeHandle> m_rosNode;
      std::thread m_rosQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(modemPlugin)
}
#endif
