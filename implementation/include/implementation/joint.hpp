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

    Joint(const signed char a_index,
          const std::string& a_header_id, 
          const std::string& a_child_id, 
          const JointState& a_jointState,
          const Movable a_variable,
          const double a_minimum,
          const double a_maximum,
          const double a_timeInterval);
    ~Joint();

    void adjustGoal(const signed char a_index, const double newValue, const double duration);

    void move();

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

    /// @brief Which part of the joint state can be changed
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