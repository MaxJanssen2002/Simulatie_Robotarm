#include "implementation/joint.hpp"

#include <iostream>


Joint::Joint(const unsigned char a_index,
             const std::string& a_header_id, 
             const std::string& a_child_id, 
             const JointState& a_jointState,
             const Movable a_variable,
             const double a_minimum,
             const double a_maximum)
: index(a_index),
  header_id(a_header_id), 
  child_id(a_child_id),
  jointState(a_jointState),
  variable(a_variable),
  minimum(a_minimum),
  maximum(a_maximum)
{
}


Joint::~Joint() {}


void Joint::moveJoint(const double newValue)
{
    if (newValue < MINIMUM_PWM || newValue > MAXIMUM_PWM)
    {
        printError(newValue);
        return;
    }

    double mappedValue = map(newValue, MINIMUM_PWM, MAXIMUM_PWM, minimum, maximum);

    switch (variable)
    {
        case X:
            jointState.x = mappedValue;
            break;
        case Y:
            jointState.y = mappedValue;
            break;
        case Z:
            jointState.z = mappedValue;
            break;
        case ROLL:
            jointState.roll = mappedValue;
            break;
        case PITCH:
            jointState.pitch = mappedValue;
            break;
        case YAW:
            jointState.yaw = mappedValue;
            break;
        default:
            // Nothing
            break;
    }
}


const JointState& Joint::getJointState() const
{
    return jointState;
}


const std::string& Joint::getHeader_id() const
{
    return header_id;
}


const std::string& Joint::getChild_id() const
{
    return child_id;
}


void Joint::printError(const double& newValue)
{  
    std::cout << "The pwm for joint number " << std::to_string(index) << " (" << header_id << "2" << child_id << ")";
    std::cout << " is out of range. It has to be between " << MINIMUM_PWM << " and " << MAXIMUM_PWM;
    std::cout << ", so " << newValue << " is invalid." << std::endl;
}


double Joint::map(const double originalNumber, const double in_min, const double in_max, const double out_min, const double out_max)
{
    return (originalNumber - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}