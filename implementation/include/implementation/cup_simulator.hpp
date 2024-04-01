#ifndef CUP_SIMULATOR_HPP
#define CUP_SIMULATOR_HPP


#include <chrono>
#include <functional>
#include <memory>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"



using namespace std::chrono_literals;


#define TIME_INTERVAL 33


struct CupState
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};



class CupSimulator : public rclcpp::Node
{
public:
    CupSimulator();
    ~CupSimulator();

private:

    void jointStateCallback(const sensor_msgs::msg::JointState & msg);

    void timer_callback();

    void transform();

    void gravityUpdate();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::TimerBase::SharedPtr timer;


    const double PI = 3.141592653589793238463;
    const double degree = PI / 180.0;

    const double gravitationalAcceleration = 9.81;
    double fallingSpeed;

    rclcpp::Time now;
    double time;

    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::TransformStamped received_t;
    tf2::Quaternion q;

    CupState state;
};



#endif