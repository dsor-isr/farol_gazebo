#include <string>
#include <uuv_sensor_ros_plugins/modemPlugin.hh>

using namespace gazebo;


modemPlugin::modemPlugin(): m_temperature(10.0), m_noiseMu(0),
                                        m_noiseSigma(1), m_soundSpeed(1500), m_hasUSBL(false) {}
modemPlugin::~modemPlugin() {}
void modemPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    /*****************************  SDF PARAMETERS ****************************/

    // Ensure ROS is initialized for publishers and subscribers
    if (!ros::isInitialized())
    {
        gzerr << "ROS has not been inintialized\n";
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "modem",
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
    // Obtain modem device name from SDF
    if (!_sdf->HasElement("modem_device"))
    {
        gzerr << "Missing required parameter <modem_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemDevice = _sdf->Get<std::string>("modem_device");
    gzmsg << "modem device: " << this->m_modemDevice << std::endl;

    /*------------------------------------------------------------------------*/
    // get modem ID
    if (!_sdf->HasElement("modem_ID"))
    {
        gzerr << "Missing required parameter <modem_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemID = _sdf->Get<std::string>("modem_ID");

    /*------------------------------------------------------------------------*/
    // get usbl ID
    if (!_sdf->HasElement("usbl_device"))
    {
        gzerr << "Missing required parameter <usbl_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_usblDevice = _sdf->Get<std::string>("usbl_device");

    /*------------------------------------------------------------------------*/
    // get usbl ID
    if (!_sdf->HasElement("usbl_ID"))
    {
        gzerr << "Missing required parameter <usbl_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_usblID = _sdf->Get<std::string>("usbl_ID");

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

    // store this entity model
    this->m_model = _model;

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
    // Get time to wait
    if (!_sdf->HasElement("delay_until_answer"))
    {
        gzerr << "Missing required parameter <delay_until_answer>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_delayUntilAnswer = _sdf->Get<std::string>(
        "delay_until_answer");

		/*------------------------------------------------------------------------*/
    // Get if the modem have or not USBL
    if (_sdf->HasElement("hasUSBL"))
    {
				this->m_hasUSBL = _sdf->Get<bool>(
        "hasUSBL");
    }

    /********************************************************************/

    /******************  ROS PUBLISHERS ************************/
    this->m_rosNode.reset(new ros::NodeHandle(this->m_modemDevice));

    std::string commandResponseTopic("/" + this->m_namespace + "/"
        + this->m_usblDevice + "_" + this->m_usblID
        + "/command_response");
    this->m_commandResponsePub = this->m_rosNode->advertise<std_msgs::String>(commandResponseTopic, 1);

    this->m_trigger_serialization = this->m_rosNode->advertise<std_msgs::Empty>("/"+this->m_modemAttachedObject+"/acomms/scheme/trigger_serialization", 1);
    
    this->m_payload_to_deserialize = this->m_rosNode->advertise<std_msgs::String>("/"+this->m_modemAttachedObject+"/acomms/scheme/payload_to_deserialize", 1);
    
    this->m_globalPosPub2 = this->m_rosNode->advertise<uuv_sensor_ros_plugins_msgs::modemLocation>("/" + this->m_namespace + "/" + this->m_usblDevice + "_" + this->m_modemID + "/global_position", 1);
    
    this->m_usbl_fix = this->m_rosNode->advertise<dmac::mUSBLFix>("/"+this->m_modemAttachedObject+"/acomms/modem/measurement/usbl_fix", 1);

    /*********************************************************************/

    /******************  ROS SUBSCRIBERS ********************/

    this->m_rosNode.reset(new ros::NodeHandle(this->m_modemDevice));

    ros::SubscribeOptions iis_ping =
        ros::SubscribeOptions::create<uuv_sensor_ros_plugins_msgs::modemLocation>(
            "/" + this->m_namespace + "/" + this->m_modemDevice
            + "_" + this->m_modemID + "/individual_interrogation_ping",
            1,
            boost::bind(&modemPlugin::iisRosCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_iisSub = this->m_rosNode->subscribe(iis_ping);

    ros::SubscribeOptions cis_ping =
        ros::SubscribeOptions::create<uuv_sensor_ros_plugins_msgs::modemLocation>(
            "/" + this->m_namespace + "/common_interrogation_ping",
            1,
            boost::bind(&modemPlugin::cisRosCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_cisSub = this->m_rosNode->subscribe(cis_ping);

    ros::SubscribeOptions payload_to_transmit = ros::SubscribeOptions::create<std_msgs::String>(
            "/"+this->m_modemAttachedObject+"/acomms/serializer/payload_to_transmit",
            1,
            boost::bind(&modemPlugin::payloadToTransmitCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);
    
    this->m_payload_to_transmit = this->m_rosNode->subscribe(payload_to_transmit);
 
    /********************************************************************/

    /******************  ROS MISC ******************************/

    this->m_rosQueueThread = std::thread(std::bind(
        &modemPlugin::queueThread, this));
    gzmsg << "modem plugin loaded\n";
}

// receives ping from modem and call Send()
void modemPlugin::iisRosCallback(uuv_sensor_ros_plugins_msgs::modemLocationConstPtr modem_position)
{
    gzmsg << this->m_modemDevice+ "_" + this->m_modemID
        + ": Received iis_ping, responding\n";

   std_msgs::String aux_msg;
    aux_msg.data = modem_position->data + ":vehicle" + this->m_usblID; 
    //deserialization
    this->m_payload_to_deserialize.publish(aux_msg);
  
    //when a modem has a usbl too
    if(this->m_hasUSBL)
    {
      ignition::math::Vector3d modem_position_ign
        = ignition::math::Vector3d(modem_position->x,
        modem_position->y, modem_position->z);

      this->aux_usbl_id = modem_position->modem_ID;
      this->aux_usbl_frame_id = this->m_usblAttachedObject + "_usbl";

      double bearing = 0, range = 0, elevation = 0;
      calcuateRelativePose(modem_position_ign, bearing, range, elevation);
    
      publishPosition(bearing, range, elevation);
    }

    //pub triggerSerialization
    std_msgs::Empty aux;
    this->m_trigger_serialization.publish(aux); 
}

// receives ping from modem and call Send()
void modemPlugin::cisRosCallback(uuv_sensor_ros_plugins_msgs::modemLocationConstPtr modem_position)
{
    std_msgs::String aux_msg;
    aux_msg.data = modem_position->data + ":vehicle" + this->m_usblID; 
    //deserialization
    this->m_payload_to_deserialize.publish(aux_msg);
  
    //when a modem has a usbl too
    if(this->m_hasUSBL)
    {
      ignition::math::Vector3d modem_position_ign
        = ignition::math::Vector3d(modem_position->x,
        modem_position->y, modem_position->z);

      this->aux_usbl_id = modem_position->modem_ID;
      this->aux_usbl_frame_id = this->m_modemAttachedObject + "_usbl";

      double bearing = 0, range = 0, elevation = 0;
      calcuateRelativePose(modem_position_ign, bearing, range, elevation);
    
      publishPosition(bearing, range, elevation);
    }

    //pub triggerSerialization
    std_msgs::Empty aux;
    this->m_trigger_serialization.publish(aux);
}

void modemPlugin::payloadToTransmitCallback(const std_msgs::StringConstPtr &payload)
{
    auto box = this->m_model->GetWorld()->ModelByName(this->m_usblAttachedObject);
    double dist = (this->m_model->WorldPose().Pos()
                   - box->WorldPose().Pos()).Length();
    // delay the answer acording a parameter to garantee that the acoustic channel is not with messages from 2
    // different modems
    ros::Duration(std::stoi(this->m_delayUntilAnswer)).sleep();
    // simulate the delay of the acoustic channel
    ros::Duration(dist / this->m_soundSpeed).sleep();
    
    // randomly generate from normal distribution for noise
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d(this->m_noiseMu, this->m_noiseSigma);

    // Gazebo publishing modem's position with noise and delay
    auto curr_pose = this->m_model->WorldPose();
    ignition::math::Vector3<double> position = curr_pose.Pos();
    //auto pub_mPub->Publish(pub_msg);

    uuv_sensor_ros_plugins_msgs::modemLocation aux_modem_pos;

    aux_modem_pos.x = position.X() + d(gen);
    aux_modem_pos.y = position.Y() + d(gen);
    aux_modem_pos.z = position.Z() + d(gen);
    aux_modem_pos.modem_ID = this->m_modemID;
    aux_modem_pos.data = payload->data;

    this->m_globalPosPub2.publish(aux_modem_pos);
}

void modemPlugin::queueThread()
{
    static const double timeout = 0.01;
    while (this->m_rosNode->ok())
    {
        this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

void modemPlugin::calcuateRelativePose(ignition::math::Vector3d position,
        double &bearing, double &range, double &elevation)
{
    auto my_pos = this->m_model->WorldPose();
    auto direction = -position + my_pos.Pos();

    bearing = (atan2(direction.X(), direction.Y()) + M_PI)* 180 / M_PI;
    range = sqrt(direction.X()*direction.X() + direction.Y()*direction.Y()
          + direction.Z()*direction.Z());
    elevation = asin(-direction.Z()/direction.Length()) * 180 / M_PI;
}

// publish modem's relative position in spherical
// coordinate(range, bearing, elevation)
void modemPlugin::publishPosition(double &bearing, double &range,
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


    gzmsg << "\n\nIN: " << this->m_modemAttachedObject  << "\nSource_id: " << 
      this->aux_usbl_id << "\nSpherical Coordinate: \n\tBearing: " << location.x
          << " degree(s)\n\tRange: " << location.y
          << " m\n\tElevation: " << location.z << " degree(s)\n" << "\n\n";
    //gzmsg << "Cartesion Coordinate: \n\tX: " << location_cartesion.x
          //<< " m\n\tY: " << location_cartesion.y
          //<< " m\n\tZ: " << location_cartesion.z << " m\n\n";
  
    //this->m_publishmodemRelPos.publish(location);
    //this->m_publishmodemRelPosCartesion.publish(location_cartesion);

    dmac::mUSBLFix fix_msg;
    fix_msg.header.stamp = ros::Time::now();
    fix_msg.header.frame_id = this->aux_usbl_frame_id;
    fix_msg.type = fix_msg.FULL_FIX;
    fix_msg.source_id = std::stoi(this->aux_usbl_id);
    fix_msg.source_name = this->m_usblDevice + this->aux_usbl_id;
    fix_msg.bearing_raw = bearing * M_PI/180;
    fix_msg.elevation_raw = elevation * M_PI/180;
    fix_msg.range = range;
    fix_msg.sound_speed = this->m_soundSpeed;

    // to garante that the position sended via acoustic is received frist than angles in USBL2POS
    ros::Duration(0.1).sleep();
    this->m_usbl_fix.publish(fix_msg);

}

