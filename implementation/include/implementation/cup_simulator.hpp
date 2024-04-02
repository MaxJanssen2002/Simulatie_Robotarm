#ifndef CUP_SIMULATOR_HPP
#define CUP_SIMULATOR_HPP


#include <chrono>
#include <functional>
#include <memory>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"


#define TIME_INTERVAL 33
#define ALTITUDE_OF_GROUND 0.02
#define MAXIMUM_DISTANCE_TO_GRIPPER 0.040
#define MAXIMUM_DISTANCE_BETWEEN_GRIPPERS 0.053


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

    void timer_callback();

    void transform();

    void gravityUpdate();

    void checkHeldByGripper();

    double pythagoreanTheorem(double a, double b, double c);

    void updateFromGripper();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::TimerBase::SharedPtr timer;


    const double PI = 3.141592653589793238463;

    const double gravitationalAcceleration = 9.81;
    double fallingSpeed;

    rclcpp::Time now;
    double time;

    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;

    CupState state;
    bool heldByGripper;
};



#endif