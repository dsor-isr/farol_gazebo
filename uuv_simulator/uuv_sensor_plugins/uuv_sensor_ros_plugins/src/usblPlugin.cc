#include <string>
#include <vector>
#include <uuv_sensor_ros_plugins/usblPlugin.hh>
#include <iostream>
#include <sstream>

using namespace gazebo;

// available interrogation modes
std::vector<std::string> im = {"common", "individual"};

// default temperature = 10 degrees Celsius
usblPlugin::usblPlugin(): m_temperature(10.0), m_soundSpeed(1500), m_noiseMu(0), m_noiseSigma(1) {}

usblPlugin::~usblPlugin() {}

void usblPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    /** SDF PARAMETERS*****************/

    // Ensure ROS is initialized for publishers and subscribers
    if (!ros::isInitialized())
    {
        gzerr << "ROS has not been initialized\n";
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "USBL_usbl",
                  ros::init_options::NoSigintHandler);
        return;
    }

    /*------------------------------------------------------------------------*/
    // Grab namespace from SDF
    if (!_sdf->HasElement("namespace"))
    {
        gzerr << "Missing required parameter <namespace>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_namespace = _sdf->Get<std::string>("namespace");

    /*------------------------------------------------------------------------*/
    // Obtain usbl device name from SDF
    if (!_sdf->HasElement("usbl_device"))
    {
        gzerr << "Missing required parameter <usbl_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    this->m_usblDevice = _sdf->Get<std::string>("usbl_device");
    gzmsg << "Entity: " << this->m_usblDevice << std::endl;

    /*------------------------------------------------------------------------*/
    // get usbl device id
    if (!_sdf->HasElement("usbl_ID"))
    {
        gzerr << "Missing required parameter <usbl_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    this->m_usblID = _sdf->Get<std::string>("usbl_ID");

    /*------------------------------------------------------------------------*/
    // get modem device name
    if (!_sdf->HasElement("modem_device"))
    {
        gzerr << "Missing required parameter <modem_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemDevice = _sdf->Get<std::string>("modem_device");
    gzmsg << "modem device: " << this->m_modemDevice << std::endl;

    /*------------------------------------------------------------------------*/
    // get commanding modems
    if (!_sdf->HasElement("modem_ID"))
    {
        gzerr << "Missing required parameter <modem_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    auto modems = ignition::common::Split(_sdf->Get<std::string>(
        "modem_ID"), ',');
    gzmsg << "Current deployed modems are: \n";

    for (auto &modem : modems)
    {
        gzmsg << modem << std::endl;
        this->m_deployedmodems.push_back(modem);
    }

    /*------------------------------------------------------------------------*/
    // get commanding modems frame ID
    if (!_sdf->HasElement("modem_frame_ID"))
    {
        gzerr << "Missing required parameter <modem_frame_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    auto modems_frame_ID = ignition::common::Split(_sdf->Get<std::string>(
        "modem_frame_ID"), ',');
    gzmsg << "Current deployed modems_frame_ID are: \n";

    for (auto &modem_frame_ID : modems_frame_ID)
    {
        gzmsg << modem_frame_ID << std::endl;
        this->m_deployedmodems_frame_ID.push_back(modem_frame_ID);
    }

    /*------------------------------------------------------------------------*/
    // enable automation of sending pings to modem
    if (!_sdf->HasElement("ping_scheduler"))
    {
        gzerr << "Missing required parameter <enable_ping_scheduler>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    this->m_pingerScheduler = _sdf->Get<std::string>("ping_scheduler");
    gzmsg << "ping time: " << this->m_pingerScheduler << std::endl;

    /*------------------------------------------------------------------------*/
    // Get object that modem attached to
    if (!_sdf->HasElement("modem_attached_object"))
    {
        gzerr << "Missing required parameter <modem_attached_object>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemAttachedObject = _sdf->Get<std::string>(
        "modem_attached_object");

    /*------------------------------------------------------------------------*/
    // Get object that usbl attached to
    if (!_sdf->HasElement("usbl_attached_object"))
    {
        gzerr << "Missing required parameter <usbl_attached_object>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_usblAttachedObject = _sdf->Get<std::string>(
        "usbl_attached_object");

    /*------------------------------------------------------------------------*/
    /*  interrogation mode - 2 options
        *  II (individual interrogation) <----->  CRS (common response signal)
        *  CI (common interrogation)     <----->  IRS (individual response
        *                                         signal) from modem_01
        *                                    ͱ->  IRS from modem_02
        *                                    ͱ->  IRS from modem_03
        *                                            ⋮
        */
    if (_sdf->HasElement("interrogation_mode"))
    {
        std::string interrogation_mode = _sdf->Get<std::string>(
            "interrogation_mode");
        if (std::find(im.begin(), im.end(), interrogation_mode) != im.end())
        {
            gzmsg << interrogation_mode << " interrogation mode is used"
                  << std::endl;
            this->m_interrogationMode = interrogation_mode;
        }
        else
        {
            gzmsg << "Specified interrogation mode is unavailable, "
                  << "Common mode is used" << std::endl;
            this->m_interrogationMode = "common";
        }
    }
    else
    {
        gzmsg << "Interrogation mode is not specified, Common mode is used"
              << std::endl;
        this->m_interrogationMode = "common";
    }

    /*------------------------------------------------------------------------*/
    // get the mean of normal distribution for the noise model
    if (_sdf->HasElement("mu"))
    {
        this->m_noiseMu = _sdf->Get<double>("mu");
    }

    /*------------------------------------------------------------------------*/
    // get the standard deviation of normal distribution for the noise model
    if (_sdf->HasElement("sigma"))
    {
        this->m_noiseSigma = _sdf->Get<double>("sigma");
    }

    /************************************************************************/

    // store this entity model
    this->m_model = _model;

    /******************  ROS PUBLISHERS ************************/

    // ROS node initialization
    this->m_rosNode.reset(new ros::NodeHandle(this->m_usblDevice));

    // ROS publisher for broadcasting modem's relative location
    // to the usbl
    //std::string modem_location_topic = "/" + this->m_namespace
        //+ "/" + this->m_usblDevice + "_" + this->m_usblID
        //+ "/modem_location";
    //this->m_publishmodemRelPos = this->m_rosNode->advertise<
        //geometry_msgs::Vector3>(modem_location_topic, 1);

    // ROS publisher for common interrogation signal ping
    std::string cis_pinger_topic = "/" + m_namespace
        + "/common_interrogation_ping";
    this->m_cisPinger = this->m_rosNode->advertise<uuv_sensor_ros_plugins_msgs::modemLocation>(
        cis_pinger_topic, 1);

    // ROS publisher for individual signal ping and command for each modem
    for (auto& modem : this->m_deployedmodems)
    {
        std::string ping_topic("/" + this->m_namespace + "/"
            + this->m_modemDevice + "_" + modem
            + "/individual_interrogation_ping");
        this->m_iisPinger[modem] = this->m_rosNode->advertise<
            uuv_sensor_ros_plugins_msgs::modemLocation>(ping_topic, 1);
    }

    //std::string modem_location_cartesion_topic = "/"
        //+ this->m_namespace + "/" + this->m_usblDevice + "_"
        //+ this->m_usblID + "/modem_location_cartesion";
    //this->m_publishmodemRelPosCartesion = this->m_rosNode->advertise<
        //geometry_msgs::Vector3>(modem_location_cartesion_topic, 1);

    this->m_usbl_fix = this->m_rosNode->advertise<dmac::mUSBLFix>("/"+this->m_usblAttachedObject+"/acomms/modem/measurement/usbl_fix", 1);
    
    this->m_trigger_serialization = this->m_rosNode->advertise<std_msgs::Empty>("/"+this->m_usblAttachedObject+"/acomms/scheme/trigger_serialization", 1);
    
    this->m_payload_to_deserialize = this->m_rosNode->advertise<std_msgs::String>("/"+this->m_usblAttachedObject+"/acomms/scheme/payload_to_deserialize", 1);
    
    /************************************************************************/

    /******************  ROS SUBSCRIBERS ***********************/

    // create ROS subscriber
    this->m_rosNode.reset(new ros::NodeHandle(m_usblDevice));

    // subscriber for setting interrogation mode
    ros::SubscribeOptions interrogation_mode_sub =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/" + this->m_usblDevice
            + "_" + this->m_usblID + "/interrogation_mode",
            1,
            boost::bind(&usblPlugin::interrogationModeRosCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_interrogationModeSub = this->m_rosNode->subscribe(
        interrogation_mode_sub);

    // subscriber for testing command response
    ros::SubscribeOptions channel_switch =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/" + this->m_usblDevice
            + "_" + this->m_usblID + "/channel_switch",
            1,
            boost::bind(&usblPlugin::channelSwitchCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_channelSwitchSub = this->m_rosNode->subscribe(channel_switch);

    ros::SubscribeOptions payload_to_transmit = ros::SubscribeOptions::create<std_msgs::String>(
            "/"+this->m_usblAttachedObject+"/acomms/serializer/payload_to_transmit",
            1,
            boost::bind(&usblPlugin::payloadToTransmitCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);
    
    this->m_payload_to_transmit = this->m_rosNode->subscribe(payload_to_transmit); 

    ros::SubscribeOptions start_ping = ros::SubscribeOptions::create<std_msgs::Bool>(
            "/"+this->m_namespace+"/enable_pinger",
            1,
            boost::bind(&usblPlugin::enablePingerCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);
    
    this->m_start_ping = this->m_rosNode->subscribe(start_ping);


    for (auto& modem : this->m_deployedmodems)
    {
        ros::SubscribeOptions sub_modem_pos = ros::SubscribeOptions::create<uuv_sensor_ros_plugins_msgs::modemLocation>(
            "/" + this->m_namespace + "/" + this->m_usblDevice + "_" + modem + "/global_position",
            1,
            boost::bind(&usblPlugin::receiveModemPoseCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);
    
        this->m_sub_modem_pos.push_back(this->m_rosNode->subscribe(sub_modem_pos));
    }

    /************************************************************************/

    /******************  ROS MISC  *****************************/
    // timer to send ping Command
    this->m_timer = this->m_rosNode->createTimer(ros::Duration(std::stoi(this->m_pingerScheduler)),
            &usblPlugin::sendPing, this);
    this->m_timer.stop();
        

    this->m_rosQueueThread = std::thread(std::bind(
        &usblPlugin::queueThread, this));
}

// publish ROS ping topic to get response from modem depending
// on the interrogation modes
void usblPlugin::sendPing(const ros::TimerEvent&)
{
    std_msgs::Empty aux;
    this->m_trigger_serialization.publish(aux);
}

void usblPlugin::payloadToTransmitCallback(const std_msgs::StringConstPtr &payload)
{
    
  // need to fake the transmission by applying distance based delay
    physics::ModelPtr tranponder = this->m_model->GetWorld()->ModelByName(
        this->m_modemAttachedObject);
    double dist = (this->m_model->WorldPose().Pos()
        - tranponder->WorldPose().Pos()).Length();
    // gzmsg << "distance to tranponder: " << dist << " m\n";
    sleep(dist/this->m_soundSpeed);

    // randomly generate from normal distribution for noise
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d(this->m_noiseMu, this->m_noiseSigma);

    // Gazebo publishing modem's position with noise and delay
    auto curr_pose = this->m_model->WorldPose();
    ignition::math::Vector3<double> position = curr_pose.Pos();

    uuv_sensor_ros_plugins_msgs::modemLocation aux_modem_pos;

    aux_modem_pos.x = position.X() + d(gen);
    aux_modem_pos.y = position.Y() + d(gen);
    aux_modem_pos.z = position.Z() + d(gen); 
    aux_modem_pos.modem_ID = this->m_usblID;
    aux_modem_pos.data = payload->data;

    if (this->m_interrogationMode.compare("common") == 0)
    {
        this->m_cisPinger.publish(aux_modem_pos);
    }
    else if (this->m_interrogationMode.compare("individual") == 0)
    {
        this->m_iisPinger[this->m_channel].publish(aux_modem_pos);
    }
    else
    {
        gzmsg << "Interrogation mode not recognized\n";
    }

}

void usblPlugin::enablePingerCallback(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data == true){
    this->m_timer.start();
    ROS_WARN("Pinger activated\n");
  }
  if(msg->data == false){
    this->m_timer.stop();
    ROS_WARN("Pinger deactivated");
  }
}

// switch channel to ping another modem or all modems
void usblPlugin::channelSwitchCallback(
        const std_msgs::StringConstPtr &msg)
{
    gzmsg << "Switching to modem_" << msg->data << " channel\n";
    this->m_channel = msg->data;
}

// callback for interrogation mode switch
void usblPlugin::interrogationModeRosCallback(
         const std_msgs::StringConstPtr &msg)
{
    std::string mode = msg->data;
    if (std::find(im.begin(), im.end(), mode) != im.end())
    {
        this->m_interrogationMode = mode;
    }
    else
    {
        gzmsg << "The input mode is not available\n";
    }
}

// Gazebo callback for receiving modem position, simulating
// usbl's positioning calculation
void usblPlugin::receiveModemPoseCallback(uuv_sensor_ros_plugins_msgs::modemLocationConstPtr modem_position)
{
    std_msgs::String aux_msg;
    aux_msg.data = modem_position->data + ":vehicle" + this->m_usblID; 
    //deserialization
    this->m_payload_to_deserialize.publish(aux_msg);
    //gzmsg << "\n\nData_Recv_In_USBL: " << aux_msg.data << std::endl;

    ignition::math::Vector3d modem_position_ign
        = ignition::math::Vector3d(modem_position->x,
        modem_position->y, modem_position->z);

    this->aux_modem_id = modem_position->modem_ID;
    this->aux_modem_frame_id = this->m_deployedmodems_frame_ID[this->getIndex(modem_position->modem_ID)];

    double bearing = 0, range = 0, elevation = 0;
    calcuateRelativePose(modem_position_ign, bearing, range, elevation);

    publishPosition(bearing, range, elevation);
}

// publish modem's relative position in spherical
// coordinate(range, bearing, elevation)
void usblPlugin::publishPosition(double &bearing, double &range,
                                        double &elevation)
{
    geometry_msgs::Vector3 location;
    location.x = bearing;
    location.y = range;
    location.z = elevation;

    geometry_msgs::Vector3 location_cartesion;
    location_cartesion.x = range * cos(elevation * M_PI/180)
                           * cos(bearing * M_PI/180);
    location_cartesion.y = range * cos(elevation * M_PI/180)
                           * sin(bearing * M_PI/180);
    location_cartesion.z = range * sin(elevation * M_PI/180);


    gzmsg << "\n\nIN: " << this->m_usblAttachedObject  << "\nSource_id: " << this->aux_modem_id << 
      "\nSpherical Coordinate: \n\tBearing: " << location.x
          << " degree(s)\n\tRange: " << location.y
          << " m\n\tElevation: " << location.z << " degree(s)\n" << "\n\n";
    //gzmsg << "Cartesion Coordinate: \n\tX: " << location_cartesion.x
          //<< " m\n\tY: " << location_cartesion.y
          //<< " m\n\tZ: " << location_cartesion.z << " m\n\n";
  
    //this->m_publishmodemRelPos.publish(location);
    //this->m_publishmodemRelPosCartesion.publish(location_cartesion);

    dmac::mUSBLFix fix_msg;
    fix_msg.header.stamp = ros::Time::now();
    fix_msg.header.frame_id = this->aux_modem_frame_id;
    fix_msg.type = fix_msg.FULL_FIX;
    fix_msg.source_id = std::stoi(this->aux_modem_id);
    fix_msg.source_name = this->m_modemDevice + this->aux_modem_id;
    fix_msg.bearing_raw = bearing * M_PI/180;
    fix_msg.elevation_raw = elevation * M_PI/180;
    fix_msg.range = range;
    fix_msg.sound_speed = this->m_soundSpeed;

    // to garante that the position sended via acoustic is received frist than angles in USBL2POS
    ros::Duration(0.1).sleep();
    this->m_usbl_fix.publish(fix_msg);

}

void usblPlugin::calcuateRelativePose(ignition::math::Vector3d position,
        double &bearing, double &range, double &elevation)
{
    auto my_pos = this->m_model->WorldPose();
    auto direction = position - my_pos.Pos();

    bearing = (atan2(direction.X(), direction.Y()) + M_PI)* 180 / M_PI;
    range = sqrt(direction.X()*direction.X() + direction.Y()*direction.Y()
          + direction.Z()*direction.Z());
    elevation = asin(-direction.Z()/direction.Length()) * 180 / M_PI;
}

// use threads to execute callback associated with each subscriber
void usblPlugin::queueThread()
{
    static const double timeout = 0.01;
    while (this->m_rosNode->ok())
    {
        this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

int usblPlugin::getIndex(std::string elem)
{
  auto it = find(this->m_deployedmodems.begin(), this->m_deployedmodems.end(), elem);

  // If element was found - is the only choise
  if (it != this->m_deployedmodems.end())
  {
    // calculate the index of elem
    int index = it - this->m_deployedmodems.begin();
    return index;
  }
  else
  {
    std::cout << "Something was wrong in usbl configuration" << std::endl;
    return 0;
  }
}

