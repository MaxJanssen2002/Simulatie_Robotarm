#ifndef STATE_PUBLISHER_HPP
#define STATE_PUBLISHER_HPP


#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"


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

    void transform(std::string header_id, std::string child_id, double x, double y, double z, double roll, double pitch, double yaw);

    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;
};



#endif