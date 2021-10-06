#ifndef CONTROL_HPP
#define CONTROL_HPP



#include <string>
#include "motorcontrol.h"
#include "uiparser.hpp"
#include "particlefilter.h"
#include "uarttest.hpp"
#include "gyroreader.hpp"
enum States {INIT_GPS,INIT_PLATFORM,INIT_GYRO,WAYPOINTS_DRIVE,WAYPOINT_WAITING,OBSTACLE_WAITING,MANUAL,AUTO,IDLE};


class Control{
public:
static ParticleFilter particleFilter;
static UartTest uartTest;
static GyroReader gyroReader;
    MotorControl motorControl=MotorControl(10,10);
   void control();
   States state = States::INIT_GPS;//skip init-platform for tests
   bool enterAutoMode();
   bool enterManualMode();
private:
   std::string TAG = "[control] ";


};

#endif // CONTROL_HPP
