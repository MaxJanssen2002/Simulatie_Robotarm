#include "implementation/state_publisher.hpp"


StatePublisher::StatePublisher()
: Node("state_publisher")
{
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer = this->create_wall_timer(100ms, std::bind(&StatePublisher::timer_callback, this));
}


StatePublisher::~StatePublisher() {}


void StatePublisher::transform(std::string header_id, std::string child_id, double x, double y, double z, double roll, double pitch, double yaw)
{
    now = this->get_clock()->now();
    
    t.header.stamp = now;
    t.header.frame_id = header_id;
    t.child_frame_id = child_id;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(t);
}


void StatePublisher::timer_callback()
{
    now = this->get_clock()->now();
    time = now.seconds() * PI;

    transform("base_link", "turret", 0, 0, 0.045, 0, 0, 0);
    transform("turret", "upperarm", 0, 0, 0.02, 0, 0, 0);
    transform("upperarm", "forearm", 0, 0, 0.18, 0, PI / 2, 0);
    transform("forearm", "wrist", 0, 0, 0.20, 0, 0, 0);
    transform("wrist", "hand", 0, 0, 0.06, 0, 0, 0);
    transform("hand", "gripper_left", 0, 0.025, 0.025, 0, 0, 0);
    transform("hand", "gripper_right", 0, -0.025, 0.025, 0, 0, 0);

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
    joint_state.position = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
    joint_pub->publish(joint_state);
}