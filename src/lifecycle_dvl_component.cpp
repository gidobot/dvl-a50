/**
 * @file   lifecycle_dvl_component.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 *
 * Contact: pgutierrez@marum.de
 */

#include "dvl_a50/lifecycle_dvl_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;
using nlohmann::json;

namespace composition
{



LifecycleDVL::LifecycleDVL(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("dvl_a50_node", options),
current_altitude(0.0),
old_altitude(0.0)
{
    this->declare_parameter<std::string>("dvl_ip_address", "192.168.194.95");   
    ip_address = this->get_parameter("dvl_ip_address").as_string();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());
}

LifecycleDVL::~LifecycleDVL() {
  tcpSocket->Close();
  delete tcpSocket;
}

/// Transition callback for state configuring
/// TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
/// TRANSITION_CALLBACK_FAILURE transitions to "inactive"
/// TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LifecycleDVL::on_timer, this));
    dvl_pub_report = this->create_publisher<marine_acoustic_msgs::msg::Dvl>("dvl/velocity", 10);
    dvl_pub_pos = this->create_publisher<nav_msgs::msg::Odometry>("dvl/position", 10);
    

    //--- TCP/IP SOCKET ---- 
    tcpSocket = new TCPSocket((char*)ip_address.c_str() , 16171);
    
    if(tcpSocket->Create() < 0)
    {
        RCLCPP_INFO(get_logger(), "Socket creation error");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
   
    tcpSocket->SetRcvTimeout(300);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    first_time_error = std::chrono::steady_clock::now();
    while(fault != 0)
    {
        fault = tcpSocket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            RCLCPP_INFO(get_logger(), "TCP Error: [%s]", error.c_str());
            RCLCPP_INFO(get_logger(), "Is the sensor on?");
            //usleep(2000000);
            std::this_thread::sleep_for(2s);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
    	    double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
    	    if(dt >= 78.5) //Max time to set up
    	    {
    	        RCLCPP_INFO(get_logger(), "Error time: [%6.2f]", dt);
    	        fault = -10;
    	        break;
    	    }
        }
        else if(error_code == 103)
        {
            RCLCPP_INFO(get_logger(), "TCP Error: [%s]", error.c_str());
            RCLCPP_INFO(get_logger(), "No route to host, DVL might be booting?");
            //usleep(2000000);
            std::this_thread::sleep_for(2s);
        }
    }  
    
    //std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    //double dt = std::chrono::duration<double>(current_time - first_time).count();
    //first_time = current_time;
    //std::cout << "time: " << dt << std::endl;
    
    if(fault == -10)
    {
        tcpSocket->Close();
        RCLCPP_INFO(get_logger(), "Turn the sensor on and try again!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
        RCLCPP_INFO(get_logger(), "DVL-A50 connected!");

    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_activate(const rclcpp_lifecycle::State &)
{
    dvl_pub_report->on_activate();
    dvl_pub_pos->on_activate();
    first_time_loss = std::chrono::steady_clock::now();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state deactivating
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
LifecycleDVL::on_deactivate(const rclcpp_lifecycle::State &)
{

    dvl_pub_report->on_deactivate();
    dvl_pub_pos->on_deactivate();
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleDVL::on_cleanup(const rclcpp_lifecycle::State &)
{
    fault = 1;
    tcpSocket->Close();
 
    timer_.reset();
    dvl_pub_report.reset();
    dvl_pub_pos.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;  
}

/// Transition callback for state shutting down
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleDVL::on_shutdown(const rclcpp_lifecycle::State & state)
{
    timer_.reset();
    dvl_pub_report.reset();
    dvl_pub_pos.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());
      
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}



void LifecycleDVL::on_timer()
{

  if(!dvl_pub_report->is_activated() || !dvl_pub_pos->is_activated())
  {
      //RCLCPP_INFO(get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
  }else {
        std::flush(std::cout);
        char *tempBuffer = new char[1];

        //tcpSocket->Receive(&tempBuffer[0]);
        std::string str; 
    
        //std::chrono::steady_clock::time_point current_time;
	while(tempBuffer[0] != '\n')
	{
	    //current_time = std::chrono::steady_clock::now();
	    //double dt = std::chrono::duration<double>(current_time - first_time_loss).count();
            tcpSocket->Receive(tempBuffer);
            str = str + tempBuffer[0];
            //std::cout << "dt: " << dt << std::endl;         
	}
	
	//first_time_loss = current_time;
		
	try
	{
            json_data = json::parse(str);
		
            if (json_data.contains("altitude")) {
		
		dvl.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
		dvl.header.frame_id = "dvl_A50_report_link";
		dvl.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
		dvl.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;

		dvl.velocity.x = double(json_data["vx"]);
		dvl.velocity.y = double(json_data["vy"]);
		dvl.velocity.z = double(json_data["vz"]);
		dvl.beam_velocities_valid = json_data["velocity_valid"];
		dvl.altitude = double(json_data["altitude"]);

		// NO old-altitude fallback. This component used to hold the previous
		// altitude when the current one was invalid; the maintained driver removed
		// that because it does not report what the device actually said, and a
		// consumer cannot tell a held value from a fresh one.

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

		dvl_pub_report->publish(dvl);
		
            }
            else //if (json_data.contains("pitch")) 
            {
		//std::cout << std::setw(4) << json_data << std::endl;
		DVLDeadReckoning.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
		DVLDeadReckoning.header.frame_id = "dvl_A50_position_link";
		DVLDeadReckoning.child_frame_id = "dvl_A50_report_link";
		DVLDeadReckoning.pose.pose.position.x = double(json_data["x"]);
		DVLDeadReckoning.pose.pose.position.y = double(json_data["y"]);
		DVLDeadReckoning.pose.pose.position.z = double(json_data["z"]);
		{
			const double roll  = double(json_data["roll"])  * M_PI / 180.0;
			const double pitch = double(json_data["pitch"]) * M_PI / 180.0;
			const double yaw   = double(json_data["yaw"])   * M_PI / 180.0;
			const double cr = cos(roll*0.5),  sr = sin(roll*0.5);
			const double cp = cos(pitch*0.5), sp = sin(pitch*0.5);
			const double cy = cos(yaw*0.5),   sy = sin(yaw*0.5);
			DVLDeadReckoning.pose.pose.orientation.w = cr*cp*cy + sr*sp*sy;
			DVLDeadReckoning.pose.pose.orientation.x = sr*cp*cy - cr*sp*sy;
			DVLDeadReckoning.pose.pose.orientation.y = cr*sp*cy + sr*cp*sy;
			DVLDeadReckoning.pose.pose.orientation.z = cr*cp*sy - sr*sp*cy;
			const double var = double(json_data["std"]) * double(json_data["std"]);
			DVLDeadReckoning.pose.covariance[0]  = var;
			DVLDeadReckoning.pose.covariance[7]  = var;
			DVLDeadReckoning.pose.covariance[14] = var;
		}
		dvl_pub_pos->publish(DVLDeadReckoning);
	    }
    	
	}
	catch(std::exception& e)
	{
	     UNUSED(e);
            //std::cout << "Exception: " << e.what() << std::endl;
	} 
	 	    
    } 
}




}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::LifecycleDVL)
