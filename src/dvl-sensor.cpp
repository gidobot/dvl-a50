/**
 * @file   dvl-sensor.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 */

#include "dvl_a50/dvl-sensor.hpp"

namespace dvl_sensor {


DVL_A50::DVL_A50():
Node("dvl_a50_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);

    timer_receive = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&DVL_A50::handle_receive, this));

    //Publishers
    // "dvl/velocity", not "dvl/data". Consumers expecting a DVL velocity look
    // for the former -- it is what the simulator publishes and what a
    // marine_acoustic_msgs consumer defaults to -- so a driver publishing
    // "dvl/data" agreed with nothing and failed by silence.
    dvl_pub_report = this->create_publisher<marine_acoustic_msgs::msg::Dvl>("dvl/velocity", qos);
    dvl_pub_pos = this->create_publisher<nav_msgs::msg::Odometry>("dvl/position", qos);
    dvl_pub_config_status = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("dvl/config/status", qos);
    dvl_pub_command_response = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("dvl/command/response", qos);
    dvl_sub_config_command = this->create_subscription<std_msgs::msg::String>("dvl/config/command", qos, std::bind(&DVL_A50::command_subscriber, this, _1));

    this->declare_parameter<std::string>("dvl_address", "192.168.2.95");
    this->declare_parameter<std::string>("velocity_frame_id", "dvl_A50/velocity_link");
    this->declare_parameter<std::string>("position_frame_id", "dvl_A50/position_link");
    
    velocity_frame_id = this->get_parameter("velocity_frame_id").as_string();
    position_frame_id = this->get_parameter("position_frame_id").as_string();
    ip_address = this->get_parameter("dvl_address").as_string();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());

    //--- TCP/IP SOCKET ---- 
    tcpSocket = new TCPSocket((char*)ip_address.c_str() , 16171);
    
    if(tcpSocket->Create() < 0)
        RCLCPP_ERROR(get_logger(), "Error creating the socket");
   
    tcpSocket->SetRcvTimeout(400);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    first_time = std::chrono::steady_clock::now();
    first_time_error = first_time;
    while(fault != 0)
    {
        fault = tcpSocket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            RCLCPP_WARN(get_logger(), "Is the sensor on? error_code: %d", error_code);
            usleep(2000000);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
    	    double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
    	    if(dt >= 78.5) //Max time to set up
    	    {
    	        fault = -10;
    	        break;
    	    }
        }
        else if(error_code == 103)
        {
            RCLCPP_WARN(get_logger(), "No route to host, DVL might be booting?: error_code: %d", error_code);
            usleep(2000000);
        }
    }  
    
    if(fault == -10)
    {
        tcpSocket->Close();
        RCLCPP_ERROR(get_logger(), "Turn the sensor on and try again!");
    }
    else
        RCLCPP_INFO(get_logger(), "DVL-A50 connected!");
    
    /*
     * Disable transducer operation to limit sensor heating out of water.
     */
    this->set_json_parameter("acoustic_enabled", "false");
    usleep(2000);

}

DVL_A50::~DVL_A50() {
    tcpSocket->Close();
    delete tcpSocket;
}


void DVL_A50::handle_receive()
{
    char *tempBuffer = new char[1];

    //tcpSocket->Receive(&tempBuffer[0]);
    std::string str; 
    
    if(fault == 0)
    {
        while(tempBuffer[0] != '\n')
        {
            if(tcpSocket->Receive(tempBuffer) !=0)
                str = str + tempBuffer[0];
        }
		
        try
        {
            json_data = json::parse(str);

            if (json_data.contains("altitude")) {
                this->publish_vel_trans_report();
            }
            else if (json_data.contains("pitch")) {
                this->publish_dead_reckoning_report();
            }
            else if (json_data.contains("response_to"))
            {
                if(json_data["response_to"] == "set_config"
                || json_data["response_to"] == "calibrate_gyro"
                || json_data["response_to"] == "reset_dead_reckoning")
                    this->publish_command_response();
                else if(json_data["response_to"] == "get_config")
                    this->publish_config_status();
            }
        }
        catch(std::exception& e)
        {
            std::cout << "Exception: " << e.what() << std::endl;
        }

    }
}

/*
 * Publish velocity and transductors report
 */
void DVL_A50::publish_vel_trans_report()
{
    marine_acoustic_msgs::msg::Dvl dvl;

    dvl.header.stamp = Node::now();
    dvl.header.frame_id = velocity_frame_id;

    // The A50 is a four-beam Janus piston array reporting bottom track.
    dvl.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
    dvl.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;

    dvl.velocity.x = double(json_data["vx"]);
    dvl.velocity.y = double(json_data["vy"]);
    dvl.velocity.z = double(json_data["vz"]);
    dvl.altitude = double(json_data["altitude"]);
    dvl.beam_velocities_valid = json_data["velocity_valid"];

    // Covariance arrives as a 3x3 nested array and the field is a flat 9.
    // Left at zero when the device omits it rather than filled with a plausible
    // constant: a consumer weights this velocity by its covariance, and an
    // invented one is a lie it cannot detect.
    if (json_data.contains("covariance") && json_data["covariance"].is_array()) {
        size_t k = 0;
        for (const auto& row : json_data["covariance"]) {
            if (!row.is_array()) continue;
            for (const auto& value : row) {
                if (k < 9) dvl.velocity_covar[k++] = value.get<double>();
            }
        }
    }

    // PER-BEAM, AND num_good_beams IS THE FIELD THAT MATTERS.
    //
    // A four-beam DVL reporting two valid beams is under-determined, and this
    // one has been observed reporting 2.04 m/s with the vehicle stationary and
    // ground truth reading 0.0000. Consumers reject on beam count, so the count
    // has to be real: it is derived from each transducer's own beam_valid flag
    // rather than from the overall velocity_valid, which stays true in exactly
    // the degraded case worth rejecting.
    uint8_t good = 0;
    bool all_ranges = true;
    for (size_t i = 0; i < 4 && i < json_data["transducers"].size(); ++i) {
        const auto& t = json_data["transducers"][i];
        const bool valid = t["beam_valid"];
        dvl.beam_velocity[i] = static_cast<float>(double(t["velocity"]));
        dvl.range[i] = double(t["distance"]);
        dvl.beam_quality[i] = static_cast<float>(double(t["rssi"]));
        if (valid) { ++good; } else { all_ranges = false; }
    }
    dvl.num_good_beams = good;
    dvl.beam_ranges_valid = all_ranges;

    // beam_unit_vec is LEFT ZERO. The device does not report its beam geometry
    // and writing the datasheet angles here would put fabricated geometry
    // somewhere very hard to notice. A consumer needing it should take it from
    // the URDF, which is where this vehicle's sensor geometry already lives.

    dvl_pub_report->publish(dvl);
}

/*
 * Publish dead reckoning
 */
void DVL_A50::publish_dead_reckoning_report()
{
    // nav_msgs/Odometry rather than a private DVLDR: dead reckoning is a pose
    // with a covariance, which is what Odometry is, and every ROS tool that
    // plots or records a track already understands it.
    nav_msgs::msg::Odometry dr;
    dr.header.stamp = Node::now();
    dr.header.frame_id = position_frame_id;
    dr.child_frame_id = velocity_frame_id;
    dr.pose.pose.position.x = double(json_data["x"]);
    dr.pose.pose.position.y = double(json_data["y"]);
    dr.pose.pose.position.z = double(json_data["z"]);

    const double roll  = double(json_data["roll"])  * M_PI / 180.0;
    const double pitch = double(json_data["pitch"]) * M_PI / 180.0;
    const double yaw   = double(json_data["yaw"])   * M_PI / 180.0;
    const double cr = cos(roll * 0.5),  sr = sin(roll * 0.5);
    const double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    const double cy = cos(yaw * 0.5),   sy = sin(yaw * 0.5);
    dr.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    dr.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    dr.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    dr.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    // The device reports one standard deviation for position, so it goes on the
    // three translational diagonals and nowhere else. The rotational block is
    // left zero because the device says nothing about it.
    const double var = double(json_data["std"]) * double(json_data["std"]);
    dr.pose.covariance[0] = var;
    dr.pose.covariance[7] = var;
    dr.pose.covariance[14] = var;

    dvl_pub_pos->publish(dr);
}

/*
 * Publish the command response
 */
namespace {

/// One key/value pair, for the diagnostic payloads below.
diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value)
{
    diagnostic_msgs::msg::KeyValue p;
    p.key = key;
    p.value = value;
    return p;
}

}  // namespace

// DEVICE ACKS AND CONFIGURATION AS DiagnosticStatus.
//
// Both were private types whose only job was to carry a success flag, an error
// string and a handful of device settings -- which is what DiagnosticStatus is
// for, and every ROS tool already renders it. The level field means a consumer
// can tell a failed command from a successful one without knowing anything
// about this device.
void DVL_A50::publish_command_response()
{
    diagnostic_msgs::msg::DiagnosticStatus msg;
    const bool ok = json_data["success"];
    msg.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                   : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg.name = "dvl_a50/command";
    msg.hardware_id = ip_address;
    msg.message = ok ? std::string("ok")
                     : std::string(json_data["error_message"]);
    msg.values.push_back(kv("response_to", json_data["response_to"]));
    msg.values.push_back(kv("type", json_data["type"]));
    dvl_pub_command_response->publish(msg);
}

/*
 * Publish the sensor current configuration
 */
void DVL_A50::publish_config_status()
{
    diagnostic_msgs::msg::DiagnosticStatus msg;
    const bool ok = json_data["success"];
    msg.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                   : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg.name = "dvl_a50/config";
    msg.hardware_id = ip_address;
    msg.message = ok ? std::string("ok")
                     : std::string(json_data["error_message"]);

    const auto& r = json_data["result"];
    msg.values.push_back(kv("speed_of_sound", std::to_string(int(r["speed_of_sound"]))));
    msg.values.push_back(kv("acoustic_enabled", bool(r["acoustic_enabled"]) ? "true" : "false"));
    msg.values.push_back(kv("dark_mode_enabled", bool(r["dark_mode_enabled"]) ? "true" : "false"));
    msg.values.push_back(kv("mounting_rotation_offset",
                            std::to_string(double(r["mounting_rotation_offset"]))));
    msg.values.push_back(kv("range_mode", r["range_mode"]));
    dvl_pub_config_status->publish(msg);
}


// JSON IN, JSON OUT. The device's command interface is already JSON and this
// driver only forwards it, so a message carrying that JSON invents nothing. The
// previous three-string type had to be understood by every operator tool and
// was defined nowhere but here.
void DVL_A50::command_subscriber(const std_msgs::msg::String::SharedPtr in)
{
    json msg;
    try {
        msg = json::parse(in->data);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "dvl/config/command is not valid JSON: %s", e.what());
        return;
    }
    if (!msg.contains("command")) {
        RCLCPP_ERROR(get_logger(), "dvl/config/command has no \"command\" field");
        return;
    }
    const std::string command = msg["command"];

    if(command == "set_config")
        this->set_json_parameter(msg.value("parameter_name", ""), msg.value("parameter_value", ""));
    else if(command == "get_config")
    {
        json command = {
            {"command", "get_config"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(command == "calibrate_gyro")
    {
        json command = {
            {"command", "calibrate_gyro"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(command == "reset_dead_reckoning")
    {
        json command = {
            {"command", "reset_dead_reckoning"}
        };
        this->send_parameter_to_sensor(command);
    }

}

DVL_Parameters DVL_A50::resolveParameter(std::string param)
{
	if(param == "speed_of_sound")
	    return speed_of_sound;
	else if(param == "acoustic_enabled")
	    return acoustic_enabled;
	else if(param == "dark_mode_enabled")
	    return dark_mode_enabled;
	else if(param == "mountig_rotation_offset")
	    return mountig_rotation_offset;
	else if(param == "range_mode")
	    return range_mode;

	return invalid_param;
}

/*
 * Create JSON command message to set parameter in the DVL sensor
 */
void DVL_A50::set_json_parameter(const std::string name, const std::string value)
{
    json message;
    message["command"] = "set_config";

    switch (resolveParameter(name))
    {
        case speed_of_sound:
            try
            {
                message["parameters"]["speed_of_sound"] = (int)std::stoi(value);
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case acoustic_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["acoustic_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case dark_mode_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["dark_mode_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case mountig_rotation_offset:
            try
            {
                message["parameters"]["mountig_rotation_offset"] = (double)std::stod(value);
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case range_mode:
            try
            {
                message["parameters"]["range_mode"] = value;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Invalid parameter!");
            break;
    }

}

/*
 * Send parameter to the sensor using the JSON command
 */
void DVL_A50::send_parameter_to_sensor(const json &message)
{
    std::string str = message.dump();
    char* c = &*str.begin();
    tcpSocket->Send(c);
}

}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dvl_sensor::DVL_A50>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
