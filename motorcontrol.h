#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include <string>
//#include "uartRoomba.h"
class Subscriber;
class Odometry;
//class RoombaController;
class MotorControl
{
public:
    MotorControl (double track, double wheelRadius);
    MotorControl (double track, double wheelRadius, Subscriber* subscriber);
    //  RoombaController* rc;
    void setSpeed(double speed, double radius);

    double getRightWheelSpeed();//for consumers like topic advertisers
    double getLefttWheelSpeed();

    void setTargetPoint(double x, double y);//set target point wo yaw, Monitoring of reaching it will be done in tick()
    Odometry* odometryFromControl;

    void setWheelSpeedsCenter(double speed, double radius);
    void setWheelSpeedsFromAngVel(double linVel, double angVel);
private:
    Subscriber* subscriber;
    double leftWheelSpeed;
    double rightWheelSpeed;
    double leftWheelSpeedPrevious=0;
    double rightWheelSpeedPrevious=0;
    double maxAllowedWheelSpeedDeltaRadSec = 1;//todo - value from dt and wheel dia
    double track;
    double wheelRadius;
    void calcWheelSpeeds();
    void sendWheelSpeeds();
    std::string TAG = "[motorControl] ";
    // UartRoomba uartRoomba;
};

#endif // MOTORCONTROL_H
