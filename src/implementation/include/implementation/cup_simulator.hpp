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

    /// @brief Places the cup on its intial position (based on the configuration)
    void initializePosition();

    /// @brief Loops with an interval that is determined by TIME_INTERVAL
    void timer_callback();

    /// @brief Broadcasts the transform (a.k.a. the position and rotation) of the cup
    void transform();

    /// @brief Makes the cup fall if it is in the air
    void gravityUpdate();

    /// @brief Checks if the cup is held by the gripper of the robot arm
    void checkHeldByGripper();

    /// @brief Adjusts the position of the cup based on the position of the robot arm (only if it is held by the robot arm)
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