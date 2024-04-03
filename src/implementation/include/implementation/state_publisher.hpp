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
#include "std_msgs/msg/string.hpp"


#include "joint.hpp"
#include "parser.hpp"


#define TIME_INTERVAL 33


class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher();
    ~StatePublisher();

private:

    /// @brief Places the robot arm on its intial position (based on the configuration)
    void initializePosition();

    /// @brief Creates the joints for the robot arm as objects
    void createJoints();

    /// @brief Broadcasts the transform (a.k.a. the position translation and rotation) of a certain joint
    /// @param header_id The name of the first segment of the joint
    /// @param child_id The name of the second segment of the joint
    /// @param jointState The current position translation and rotation of the joint
    void transform(const std::string& header_id, const std::string& child_id, const JointState& jointState);

    /// @brief Loops with an interval that is determined by TIME_INTERVAL
    void timer_callback();

    /// @brief Gets called if there is an incoming message from /arm_command, which should be a SSC-32U formatted command
    /// @param msg The incoming message
    void PWM_command_callback(const std_msgs::msg::String & msg);

    rclcpp::Time now;
    double time;

    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSubscription;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<Joint> joints;

    Parser commandParser;
};



#endif