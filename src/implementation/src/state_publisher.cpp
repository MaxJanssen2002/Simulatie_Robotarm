#include "implementation/state_publisher.hpp"
#include "implementation/math_utils.hpp"

#include <sstream>
#include <algorithm>


StatePublisher::StatePublisher()
: Node("state_publisher")
{
    commandSubscription = this->create_subscription<std_msgs::msg::String>("/arm_command", 10, std::bind(&StatePublisher::PWM_command_callback, this, std::placeholders::_1));
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer = this->create_wall_timer(std::chrono::milliseconds(TIME_INTERVAL), std::bind(&StatePublisher::timer_callback, this));

    initializePosition();
    createJoints();
}


StatePublisher::~StatePublisher() {}


void StatePublisher::initializePosition()
{
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z", 0.0);
    this->declare_parameter<double>("roll", 0.0);
    this->declare_parameter<double>("pitch", 0.0);
    this->declare_parameter<double>("yaw", 0.0);

    double initialX = this->get_parameter("x").get_parameter_value().get<double>();
    double initialY = this->get_parameter("y").get_parameter_value().get<double>();
    double initialZ = this->get_parameter("z").get_parameter_value().get<double>();
    double initialRoll = this->get_parameter("roll").get_parameter_value().get<double>();
    double initialPitch = this->get_parameter("pitch").get_parameter_value().get<double>();
    double initialYaw = this->get_parameter("yaw").get_parameter_value().get<double>();

    joints.emplace_back(-1, "odom", "base_link", JointState{initialX, initialY, initialZ, initialRoll, initialPitch, initialYaw}, PITCH, initialYaw, initialYaw, TIME_INTERVAL);
    
    for (Joint& joint : joints)
    {
        transform(joint.getHeader_id(), joint.getChild_id(), joint.getJointState());
    }
}


void StatePublisher::createJoints()
{
    joints.emplace_back(0,  "base_link", "turret",        JointState{0.0, 0.0, 0.045, 0.0, 0.0, 0.0},    YAW,   PI / -2, PI / 2,    TIME_INTERVAL);
    joints.emplace_back(1,  "turret",    "upperarm",      JointState{0.0, 0.0, 0.02, 0.0, 0.0, 0.0},     PITCH, PI / -3, PI / 2,    TIME_INTERVAL);
    joints.emplace_back(2,  "upperarm",  "forearm",       JointState{0.0, 0.0, 0.18, 0.0, PI / 2, 0.0},  PITCH, 0,       PI * 0.75, TIME_INTERVAL);
    joints.emplace_back(3,  "forearm",   "wrist",         JointState{0.0, 0.0, 0.20, 0.0, 0.0, 0.0},     PITCH, PI / -2, PI / 2,    TIME_INTERVAL);
    joints.emplace_back(4,  "hand",      "gripper_left",  JointState{0.0, 0.025, 0.025, 0.0, 0.0, 0.0},  Y,     0.033,   0.01,      TIME_INTERVAL);
    joints.emplace_back(4,  "hand",      "gripper_right", JointState{0.0, -0.025, 0.025, 0.0, 0.0, 0.0}, Y,     -0.033,  -0.01,     TIME_INTERVAL);
    joints.emplace_back(5,  "wrist",     "hand",          JointState{0.0, 0.0, 0.06, 0.0, 0.0, 0.0},     YAW,   PI / -2, PI / 2,    TIME_INTERVAL);
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
    for (Joint& joint : joints)
    {
        joint.move();
        transform(joint.getHeader_id(), joint.getChild_id(), joint.getJointState());
    }
}


void StatePublisher::PWM_command_callback(const std_msgs::msg::String & msg)
{
    std::string command = msg.data;

    if (command == "stop" || command == "Stop" || command == "STOP")
    {
        for (auto& joint : joints)
        {
            joint.stop();
        }
        return;
    }

    if (!commandParser.parseCommand(command))
    {
        return;
    }

    fullcommand movementsAndTime = commandParser.getMovementsAndTime();

    for (const auto& movement : movementsAndTime.first)
    {
        for (auto& joint : joints)
        {
            joint.adjustGoal(movement.first, movement.second, movementsAndTime.second);
        }
    }
}


