#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include <string>
class Subscriber;
class Odometry;

class MotorControl
{
public:
    MotorControl (double track, double wheelRadius);
    MotorControl (double track, double wheelRadius, Subscriber* subscriber);

    void setSpeed(double speed, double radius);

double getRightWheelSpeed();//for consumers like topic advertisers
double getLefttWheelSpeed();

void setTargetPoint(double x, double y);//set target point wo yaw, Monitoring of reaching it will be done in tick()
Odometry* odometryFromControl;

void setWheelSpeedsCenter(double speed, double radius);
private:
Subscriber* subscriber;
double leftWheelSpeed;
    double rightWheelSpeed;
    double track;
    double wheelRadius;
    void calcWheelSpeeds();
    void sendWheelSpeeds();
    std::string TAG = "[motorControl] ";

};

#endif // MOTORCONTROL_H
