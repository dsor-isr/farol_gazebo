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

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <limits>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>
#include <dsor_msgs/Thruster.h>

#include <math.h>

#include <uuv_gazebo_ros_plugins/ThrusterManagerROSPlugin.hh>

namespace gazebo {

    ThrusterManagerROSPlugin::ThrusterManagerROSPlugin() : thrustersNumber(0)  {}

    ThrusterManagerROSPlugin::~ThrusterManagerROSPlugin() {
        if (this->updateConnection) {
            this->updateConnection.reset();
        }
        this->rosNode->shutdown();
    }


    void ThrusterManagerROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        
        GZ_ASSERT(_model != NULL, "Invalid model pointer");

        if (!ros::isInitialized()) {
            gzerr << "Not loading plugin since ROS has not been "
                << "properly initialized.  Try starting gazebo with ros plugin:\n"
                << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }

        this->rosNode.reset(new ros::NodeHandle(""));

        // Initializing the transport node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(_model->GetWorld()->Name());

        // Reading number of thrusters
        GZ_ASSERT(_sdf->HasElement("thrustersNumber"), "Number of thrusters was not provided");
        this->thrustersNumber = _sdf->Get<int>("thrustersNumber");
        GZ_ASSERT(this->thrustersNumber > 0, "Number of thrusters must be greater than zero.");

        // Reading thrusters topic from DSOR stack 
        GZ_ASSERT(_sdf->HasElement("dsorTopicThrusters"), "Topic of thrusters was not provided");
        this->dsorTopicThrusters = _sdf->Get<std::string>("dsorTopicThrusters");

        this->thrusterStackSubscriber = this->rosNode->subscribe(this->dsorTopicThrusters, 10, &ThrusterManagerROSPlugin::SetThrustReference, this);

        // Root string for topics
        std::string topicPrefix;

        for (int i = 0; i < this->thrustersNumber; i++) {

            // Get the topic prefix for each thruster topic to publish
            topicPrefix =  "/" + _model->GetName() + "/thrusters/" + std::to_string(i) + "/input";

            // Advertise the thrust topic
            this->thrustInputPublisher.push_back(this->rosNode->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(topicPrefix, 1));
        }
    }


    void ThrusterManagerROSPlugin::SetThrustReference(const dsor_msgs::Thruster &msg) {

        // Safety check the number of thrusters in the control message
        if (msg.value.size() > this->thrustersNumber) {
            ROS_WARN("Number of thruster from DSOR stack is not equal to the predefined number of thruster in vehicle (gazebo).");
            return;
        }

        // Get the current time from ros
        ros::Time curTime = ros::Time().now();
        
        // Publish individual thrust in msg (DSOR stack) to each respective topic (gazebo)
        for (int i = 0; i < this->thrustersNumber; i++) {

            // Create the FloatStamped ros msg
            uuv_gazebo_ros_plugins_msgs::FloatStamped thrust_msg;
            thrust_msg.header.stamp.sec = curTime.sec;
            thrust_msg.header.stamp.nsec = curTime.nsec;
            thrust_msg.data = msg.value[i];

            // Publish the desired thrust to each individual thruster
            this->thrustInputPublisher[i].publish(thrust_msg);
        }
    }

    void ThrusterManagerROSPlugin::Update(const common::UpdateInfo &_info) {}

GZ_REGISTER_MODEL_PLUGIN(ThrusterManagerROSPlugin)
}