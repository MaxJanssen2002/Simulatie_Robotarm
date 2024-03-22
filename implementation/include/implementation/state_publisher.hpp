#ifndef STATE_PUBLISHER_HPP
#define STATE_PUBLISHER_HPP


#include <chrono>
#include <functional>
#include <memory>
#include <array>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"


#include "joint.hpp"


using namespace std::chrono_literals;


class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher();
    ~StatePublisher();

private:

    const double PI = 3.141592653589793238463;
    const double degree = PI / 180.0;

    rclcpp::Time now;
    double time;

    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;

    void createJoints();

    void transform(const std::string& header_id, const std::string& child_id, const JointState& jointState);

    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<Joint> joints;
};



#endif