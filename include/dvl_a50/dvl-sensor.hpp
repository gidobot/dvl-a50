#ifndef DVL_A50_HPP
#define DVL_A50_HPP

/**
 * dvl-sensor.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       24/11/2021
 */

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "dvl_a50/tcpsocket.hpp"

#include <string>
// STANDARD MESSAGES, NOT A PRIVATE PACKAGE.
//
// The velocity report was dvl_msgs/DVL, a type defined by this driver and
// understood by nothing else. Downstream consumers therefore had to depend on
// the driver's message package to read a velocity -- and in practice a
// consumer written against marine_acoustic_msgs/Dvl, which is what the
// simulator publishes, simply never received anything. Same quantity, unrelated
// type, and a subscription that silently never fires.
//
// marine_acoustic_msgs/Dvl carries everything dvl_msgs/DVL did that a consumer
// needs, including per-beam ranges, velocities and quality, and adds
// num_good_beams -- which matters here, because a two-beam solution is
// under-determined and has been observed reporting 2.04 m/s against a true
// 0.0000 m/s.
#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;
using nlohmann::json;


namespace dvl_sensor {

enum DVL_Parameters {
    speed_of_sound,
    acoustic_enabled,
    dark_mode_enabled,
    mountig_rotation_offset,
    range_mode,
    invalid_param
};

class DVL_A50: public rclcpp::Node
{
public:

    uint8_t ready = 0;
    uint8_t error = 0;

    DVL_A50();
    ~DVL_A50();

private:
    int fault = 1; 
    string delimiter = ",";
    std::string ip_address;
    std::string velocity_frame_id;
    std::string position_frame_id;
    TCPSocket *tcpSocket;
    json json_data;

    DVL_Parameters resolveParameter(std::string param);
    
    std::chrono::steady_clock::time_point first_time;
    std::chrono::steady_clock::time_point first_time_error;
    
    rclcpp::TimerBase::SharedPtr timer_receive;
    rclcpp::TimerBase::SharedPtr timer_send;
    rclcpp::Publisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr dvl_pub_report;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub_pos;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr dvl_pub_command_response;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr dvl_pub_config_status;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dvl_sub_config_command;


    void handle_receive();
    //Publish velocity and transducer report
    void publish_vel_trans_report();
    void publish_dead_reckoning_report();
    void publish_config_status();
    void publish_command_response();

    void command_subscriber(const std_msgs::msg::String::SharedPtr msg);
    void set_json_parameter(const std::string name, const std::string value);
    void send_parameter_to_sensor(const json &message);

};

} // namespace
#endif //DVL_A50_HPP
