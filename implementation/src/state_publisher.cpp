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
    rclcpp::Time now = this->get_clock()->now();
    
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = header_id;
    odom_trans.child_frame_id = child_id;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(odom_trans);
}


void StatePublisher::timer_callback()
{
    rclcpp::Time now = this->get_clock()->now();
    double time = now.seconds() * PI;

    transform("base_link", "turret", 0, 0, 0.045, 0, 0, PI / 2);
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