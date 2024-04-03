#include "implementation/cup_simulator.hpp"
#include "implementation/math_utils.hpp"


CupSimulator::CupSimulator()
: Node("cup_simulator"), fallingSpeed(0.0), heldByGripper(false)
{
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_wall_timer(std::chrono::milliseconds(TIME_INTERVAL), std::bind(&CupSimulator::timer_callback, this));

    initializePosition();

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for robotmodel before using lookupTransform
}


CupSimulator::~CupSimulator() {}


void CupSimulator::initializePosition()
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

    state = CupState{initialX, initialY, initialZ, initialRoll, initialPitch, initialYaw};
    transform();
}


void CupSimulator::timer_callback()
{
    updateFromGripper();
    gravityUpdate();
    checkHeldByGripper();
    transform();
}


void CupSimulator::transform()
{
    now = this->get_clock()->now();
    t.header.stamp = now;

    if (!heldByGripper)
    {
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
    else
    {
        t.header.frame_id = "hand";
        t.child_frame_id = "cup_base_link";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.04;

        q.setRPY(0.0, -0.5 * PI, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(t);
    }
}


void CupSimulator::gravityUpdate()
{
    if (state.z > ALTITUDE_OF_GROUND && !heldByGripper)
    {
        fallingSpeed += GRAVITATIONAL_ACCELERATION / TIME_INTERVAL;
        state.z -= fallingSpeed / TIME_INTERVAL;
    }

    if (state.z <= ALTITUDE_OF_GROUND)
    {
        state.z = ALTITUDE_OF_GROUND;
        fallingSpeed = 0.0;

        state.roll = 0.0;
        state.pitch = 0.0;
        state.yaw = 0.0;
    }
}


void CupSimulator::checkHeldByGripper()
{
    geometry_msgs::msg::TransformStamped left_gripper_position;
    geometry_msgs::msg::TransformStamped right_gripper_position;
    left_gripper_position = tf_buffer->lookupTransform("odom", "gripper_left", tf2::TimePointZero);
    right_gripper_position = tf_buffer->lookupTransform("odom", "gripper_right", tf2::TimePointZero);

    double distanceToLeftGripper = MathUtils::pythagoreanTheorem(left_gripper_position.transform.translation.x - state.x,
                                                                 left_gripper_position.transform.translation.y - state.y,
                                                                 left_gripper_position.transform.translation.z - state.z);
    double distanceToRightGripper = MathUtils::pythagoreanTheorem(right_gripper_position.transform.translation.x - state.x,
                                                                  right_gripper_position.transform.translation.y - state.y,
                                                                  right_gripper_position.transform.translation.z - state.z);
    double distanceBetweenGrippers = MathUtils::pythagoreanTheorem(left_gripper_position.transform.translation.x - right_gripper_position.transform.translation.x,
                                                                   left_gripper_position.transform.translation.y - right_gripper_position.transform.translation.y,
                                                                   left_gripper_position.transform.translation.z - right_gripper_position.transform.translation.z);

    if (distanceToLeftGripper <= MAXIMUM_DISTANCE_TO_GRIPPER && 
        distanceToRightGripper <= MAXIMUM_DISTANCE_TO_GRIPPER && 
        distanceBetweenGrippers <= MAXIMUM_DISTANCE_BETWEEN_GRIPPERS)
    {
        heldByGripper = true;
    }
    else if (distanceBetweenGrippers > MAXIMUM_DISTANCE_BETWEEN_GRIPPERS)
    {
        heldByGripper = false;
    }
}


void CupSimulator::updateFromGripper()
{
    if (heldByGripper)
    {
        geometry_msgs::msg::TransformStamped cup_position;
        cup_position = tf_buffer->lookupTransform("odom", "cup_base_link", tf2::TimePointZero);

        state.x = cup_position.transform.translation.x;
        state.y = cup_position.transform.translation.y;
        state.z = cup_position.transform.translation.z;
        state.roll = cup_position.transform.rotation.x;
        state.pitch = cup_position.transform.rotation.y;
        state.yaw = cup_position.transform.rotation.z;
    }
}