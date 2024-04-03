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
#define GRAVITATIONAL_ACCELERATION 9.81
#define ALTITUDE_OF_GROUND 0.020
#define MAXIMUM_DISTANCE_TO_GRIPPER 0.033
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

    void initializePosition();

    void timer_callback();

    void transform();

    void gravityUpdate();

    void checkHeldByGripper();

    void updateFromGripper();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::TimerBase::SharedPtr timer;

    double fallingSpeed;

    rclcpp::Time now;
    double time;

    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q;

    CupState state;
    bool heldByGripper;
};



#endif