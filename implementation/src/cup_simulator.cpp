#include "implementation/cup_simulator.hpp"

#include <iostream>


CupSimulator::CupSimulator()
: Node("cup_simulator"), fallingSpeed(0.0)
{
    joint_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CupSimulator::jointStateCallback, this, std::placeholders::_1));
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer = this->create_wall_timer(std::chrono::milliseconds(TIME_INTERVAL), std::bind(&CupSimulator::timer_callback, this));

    state = CupState{0.2, 0.2, 5.0, 0.0, 0.0, 0.0};
}


CupSimulator::~CupSimulator() {}


void CupSimulator::jointStateCallback(const sensor_msgs::msg::JointState & msg)
{

}


void CupSimulator::timer_callback()
{
    gravityUpdate();
    transform();

    rclcpp::Time now = this->get_clock()->now();

    received_t = tf_buffer->lookupTransform("base_link", "turret", now);

    RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(received_t.transform.translation.x));
}


void CupSimulator::transform()
{
    now = this->get_clock()->now();
    time = now.seconds() * PI;

    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "cup_base_link";

    t.transform.translation.x = 0.2 * cos(time);
    //t.transform.translation.x = state.x;
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
    if (state.z > 0.0)
    {
        fallingSpeed += gravitationalAcceleration / TIME_INTERVAL;
        state.z -= fallingSpeed / TIME_INTERVAL;

        if (state.z <= 0.0)
        {
            state.z = 0.0;
            fallingSpeed = 0.0;
        }
    }
}