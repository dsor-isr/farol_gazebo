#ifndef GAZEBO_USBL_PLUGIN_HPP_
#define GAZEBO_USBL_PLUGIN_HPP_

#include <thread>
#include <math.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <unordered_map>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/common/StringUtils.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

#include "dmac/mUSBLFix.h"
#include "uuv_sensor_ros_plugins_msgs/modemLocation.h"

namespace gazebo
{
  class usblPlugin : public ModelPlugin
  {
    public:
      usblPlugin();
      ~usblPlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void sendPing(const ros::TimerEvent&);
      void channelSwitchCallback(const std_msgs::StringConstPtr &msg);
      void interrogationModeRosCallback(const std_msgs::StringConstPtr &msg);
      void receiveModemPoseCallback(uuv_sensor_ros_plugins_msgs::modemLocationConstPtr modem_position);
      void publishPosition(double &bearing, double &range, double &elevation);
      void calcuateRelativePose(ignition::math::Vector3d position, double &bearing, double &range, double &elevation);
      void payloadToTransmitCallback(const std_msgs::StringConstPtr &payload);
      void enablePingerCallback(const std_msgs::BoolConstPtr &msg);
      void queueThread();
      int getIndex(std::string elem);

    public:
      // This entity's attributes
      std::string m_namespace;
      std::string m_usblDevice;
      std::string m_usblID;
      std::string m_modemAttachedObject;
      std::string m_usblAttachedObject;
      std::string m_channel = "1";
      std::string m_interrogationMode;
      std::string m_pingerScheduler = "1";

    private:
      std::string m_modemDevice;
      double m_temperature;
      double m_soundSpeed;
      std::vector<std::string> m_deployedmodems;
      std::vector<std::string> m_deployedmodems_frame_ID;
      std::string aux_modem_id;
      std::string aux_modem_frame_id;
      double m_noiseMu;
      double m_noiseSigma;

      // Gazebo nodes, publishers, and subscribers
      ros::Timer m_timer;
      physics::ModelPtr m_model;

      // ROS nodes, publishers and subscibers
      std::unique_ptr<ros::NodeHandle> m_rosNode;
      //ros::Publisher m_publishmodemRelPos;
      //ros::Publisher m_publishmodemRelPosCartesion;
      ros::Publisher m_usbl_fix;
      ros::Publisher m_trigger_serialization;
      ros::Publisher m_payload_to_deserialize;
      ros::Publisher m_cisPinger;
      std::unordered_map<std::string, ros::Publisher> m_iisPinger;
      ros::Subscriber m_interrogationModeSub;
      ros::Subscriber m_channelSwitchSub;
      ros::Subscriber m_payload_to_transmit;
      ros::Subscriber m_start_ping;
      std::vector<ros::Subscriber> m_sub_modem_pos;
      ros::CallbackQueue m_rosQueue;

      std::thread m_rosQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(usblPlugin)
}
#endif
