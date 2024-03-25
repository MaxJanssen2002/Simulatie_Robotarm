#ifndef STATE_PUBLISHER_HPP
#define STATE_PUBLISHER_HPP


#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <vector>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"


#include "joint.hpp"
#include "parser.hpp"


using namespace std::chrono_literals;



class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher();
    ~StatePublisher();

private:

    void createJoints();

    void transform(const std::string& header_id, const std::string& child_id, const JointState& jointState);

    void timer_callback();

    void PWM_command_callback(const std_msgs::msg::String & msg);

    const double PI = 3.141592653589793238463;
    const double degree = PI / 180.0;

    rclcpp::Time now;
    double time;

    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSubsription;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<Joint> joints;

    Parser commandParser;
};



#endif