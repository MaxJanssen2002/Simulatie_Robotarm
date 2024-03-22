#include "implementation/state_publisher.hpp"


StatePublisher::StatePublisher()
: Node("state_publisher")
{
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer = this->create_wall_timer(100ms, std::bind(&StatePublisher::timer_callback, this));

    createJoints();
}


StatePublisher::~StatePublisher() {}


void StatePublisher::createJoints()
{
    joints.emplace_back(0, "base_link", "turret", JointState{0.0, 0.0, 0.045, 0.0, 0.0, 0.0}, YAW, PI / -2, PI / 2);
    joints.emplace_back(1, "turret", "upperarm", JointState{0.0, 0.0, 0.02, 0.0, 0.0, 0.0}, PITCH, PI / -6, PI / 2);
    joints.emplace_back(2, "upperarm", "forearm", JointState{0.0, 0.0, 0.18, 0.0, PI / 2, 0.0}, PITCH, 0, PI * 0.75);
    joints.emplace_back(3, "forearm", "wrist", JointState{0.0, 0.0, 0.20, 0.0, 0.0, 0.0}, PITCH, PI / -2, PI / 2);
    joints.emplace_back(4, "wrist", "hand", JointState{0.0, 0.0, 0.06, 0.0, 0.0, 0.0}, YAW, PI / -2, PI / 2);
    joints.emplace_back(5, "hand", "gripper_left", JointState{0.0, 0.025, 0.025, 0.0, 0.0, 0.0}, Y, 0.33, 0.01);
    joints.emplace_back(5, "hand", "gripper_right", JointState{0.0, -0.025, 0.025, 0.0, 0.0, 0.0}, Y, -0.33, -0.01);
}


void StatePublisher::transform(const std::string& header_id, const std::string& child_id, const JointState& jointState)
{
    now = this->get_clock()->now();
    
    t.header.stamp = now;
    t.header.frame_id = header_id;
    t.child_frame_id = child_id;

    t.transform.translation.x = jointState.x;
    t.transform.translation.y = jointState.y;
    t.transform.translation.z = jointState.z;

    q.setRPY(jointState.roll, jointState.pitch, jointState.yaw);
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

    joints.at(0).moveJoint(500 * cos(time) + 1500);

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
    joint_state.position = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
    joint_pub->publish(joint_state);
}