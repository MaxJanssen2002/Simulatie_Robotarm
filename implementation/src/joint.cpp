#include "implementation/joint.hpp"

#include <iostream>


Joint::Joint(const unsigned char a_index,
             const std::string& a_header_id, 
             const std::string& a_child_id, 
             const JointState& a_jointState,
             const Movable a_variable,
             const double a_minimum,
             const double a_maximum,
             const double a_timeInterval)
: index(a_index),
  header_id(a_header_id), 
  child_id(a_child_id),
  jointState(a_jointState),
  variable(a_variable),
  minimum(a_minimum),
  maximum(a_maximum),
  startPWM(0),
  goalPWM(0),
  movementDuration(0),
  variablePosition(0),
  timeInterval(a_timeInterval),
  moving(false)
{
    switch (variable)
    {
        case X:
            variablePosition = jointState.x;
            break;
        case Y:
            variablePosition = jointState.y;
            break;
        case Z:
            variablePosition = jointState.z;
            break;
        case ROLL:
            variablePosition = jointState.roll;
            break;
        case PITCH:
            variablePosition = jointState.pitch;
            break;
        case YAW:
            variablePosition = jointState.yaw;
            break;
        default:
            // Nothing
            break;
    }
    startPWM = variablePosition;
}


Joint::~Joint() {}


void Joint::adjustGoal(const unsigned char a_index, const double newValue, const double duration)
{
    if (a_index != index)
    {
        return;
    }

    if (newValue < MINIMUM_PWM || newValue > MAXIMUM_PWM)
    {
        printError(newValue);
        return;
    }

    goalPWM = map(newValue, MINIMUM_PWM, MAXIMUM_PWM, minimum, maximum);
    movementDuration = duration;
    moving = true;
}


void Joint::move()
{
    if (!moving)
    {
        return;
    }

    if (variablePosition > goalPWM)
    {
        variablePosition -= (startPWM - goalPWM) / movementDuration * timeInterval;
        if (variablePosition < goalPWM)
        {
            variablePosition = goalPWM;
        }
    }
    else if (variablePosition < goalPWM)
    {
        variablePosition += (goalPWM - startPWM) / movementDuration * timeInterval;
        if (variablePosition > goalPWM)
        {
            variablePosition = goalPWM;
        }
    }

    if (variablePosition == goalPWM)
    {
        startPWM = variablePosition;
        moving = false;
    }

    refreshJointState();
}


void Joint::stop()
{
    goalPWM = variablePosition;
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


void Joint::refreshJointState()
{
    switch (variable)
    {
        case X:
            jointState.x = variablePosition;
            break;
        case Y:
            jointState.y = variablePosition;
            break;
        case Z:
            jointState.z = variablePosition;
            break;
        case ROLL:
            jointState.roll = variablePosition;
            break;
        case PITCH:
            jointState.pitch = variablePosition;
            break;
        case YAW:
            jointState.yaw = variablePosition;
            break;
        default:
            // Nothing
            break;
    }
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