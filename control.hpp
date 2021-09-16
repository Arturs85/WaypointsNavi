#ifndef CONTROL_HPP
#define CONTROL_HPP



#include <string>
#include "motorcontrol.h"
#include "uiparser.hpp"
enum States {INIT_GPS,INIT_PLATFORM,INIT_GYRO,WAYPOINTS_DRIVE,WAYPOINT_WAITING,OBSTACLE_WAITING,MANUAL,AUTO};


class Control{
public:
MotorControl motorControl=MotorControl(10,10);
   void control();
   States state = States::INIT_PLATFORM;

private:
   std::string TAG = "[control] ";


};

#endif // CONTROL_HPP
