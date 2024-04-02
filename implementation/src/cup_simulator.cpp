#include "implementation/cup_simulator.hpp"

#include <iostream>


CupSimulator::CupSimulator()
: Node("cup_simulator"), fallingSpeed(0.0), heldByGripper(false)
{
    joint_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CupSimulator::jointStateCallback, this, std::placeholders::_1));
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_wall_timer(std::chrono::milliseconds(TIME_INTERVAL), std::bind(&CupSimulator::timer_callback, this));

    state = CupState{0.3, 0.3, 5.0, 0.0, 0.0, 0.0};

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}


CupSimulator::~CupSimulator() {}


void CupSimulator::jointStateCallback(const sensor_msgs::msg::JointState & msg)
{

}


void CupSimulator::timer_callback()
{
    gravityUpdate();
    transform();
    checkHeldByGripper();

   
}


void CupSimulator::transform()
{
    now = this->get_clock()->now();

    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "cup_base_link";

    t.transform.translation.x = state.x;
    t.transform.translation.y = state.y;
    t.transform.translation.z = state.z;

    q.setRPY(state.roll, state.pitch, state.yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(t);
}


void CupSimulator::gravityUpdate()
{
    if (state.z > 0.02)
    {
        fallingSpeed += gravitationalAcceleration / TIME_INTERVAL;
        state.z -= fallingSpeed / TIME_INTERVAL;

        if (state.z <= 0.02)
        {
            state.z = 0.02;
            fallingSpeed = 0.0;
        }
    }
}


void CupSimulator::checkHeldByGripper()
{
    geometry_msgs::msg::TransformStamped left_gripper_position;
    geometry_msgs::msg::TransformStamped right_gripper_position;
    left_gripper_position = tf_buffer->lookupTransform("odom", "gripper_left", tf2::TimePointZero);
    right_gripper_position = tf_buffer->lookupTransform("odom", "gripper_right", tf2::TimePointZero);

    double distanceToLeftGripper = pythagoreanTheorem(left_gripper_position.transform.translation.x - state.x,
                                                      left_gripper_position.transform.translation.y - state.y,
                                                      left_gripper_position.transform.translation.z - state.z);
    double distanceToRightGripper = pythagoreanTheorem(right_gripper_position.transform.translation.x - state.x,
                                                       right_gripper_position.transform.translation.y - state.y,
                                                       right_gripper_position.transform.translation.z - state.z);
    if (distanceToLeftGripper <= 0.04 && 
        distanceToLeftGripper >= 0.035 && 
        distanceToRightGripper <= 0.04 &&
        distanceToRightGripper >= 0.035)
    {
        heldByGripper = true;
    }
    else
    {
        heldByGripper = false;
    }
}


double CupSimulator::pythagoreanTheorem(double a, double b)
{
    return sqrt(a * a + b * b);
}


double CupSimulator::pythagoreanTheorem(double a, double b, double c)
{
    return sqrt(a * a + b * b + c * c);
}