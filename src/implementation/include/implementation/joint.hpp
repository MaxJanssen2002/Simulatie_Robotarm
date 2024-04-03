#ifndef JOINT_HPP
#define JOINT_HPP


#include <string>
#include "tf2/LinearMath/Quaternion.h"


#define MINIMUM_PWM 500
#define MAXIMUM_PWM 2500


struct JointState
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};


enum Movable
{
    X, Y, Z, ROLL, PITCH, YAW
};


class Joint
{
public:

    /// @brief Constructor of the joint
    /// @param a_index The index of the joint (based on the SSC-32U format)
    /// @param a_header_id The name of the first segment of the joint
    /// @param a_child_id The name of the second segment of the joint
    /// @param a_jointState The translation and the rotation of the joint
    /// @param a_variable The part of the joint that can be changed (x, y, z, roll, pitch or yaw)
    /// @param a_minimum The minimum value of the joint
    /// @param a_maximum The maximum value of the joint
    /// @param a_timeInterval The time interval of the loop
    Joint(const signed char a_index,
          const std::string& a_header_id, 
          const std::string& a_child_id, 
          const JointState& a_jointState,
          const Movable a_variable,
          const double a_minimum,
          const double a_maximum,
          const double a_timeInterval);
    ~Joint();

    /// @brief Changes the goal position of the joint (if it gives the same index and the value is valid)
    /// @param a_index The index of the joint
    /// @param newValue The inserted goal position of the joint
    /// @param duration The duration of the movement to the goal position
    void adjustGoal(const signed char a_index, const double newValue, const double duration);

    /// @brief Moves the joint for one frame
    void move();

    /// @brief Stops the joint from changing
    void stop();

    const JointState& getJointState() const;
    const std::string& getHeader_id() const;
    const std::string& getChild_id() const;

private:

    void refreshJointState();

    void printError(const double& newValue);

    const signed char index;

    const std::string header_id;
    const std::string child_id;

    JointState jointState;

    const Movable variable;

    const double minimum;
    const double maximum;

    double startPWM;
    double goalPWM;
    double movementDuration;
    double variablePosition;
    double timeInterval;

    bool moving;

};




#endif