#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "trajectoryexecutor.hpp"//for Pose, replace with pose later- move pose to separate file
//#include "publisherbase.h"

class Odometry
{
public:

    double WHEEL_RADI = 0.1;
    double WHEEL_BASE = 0.26;

    Odometry();
    double angleLeftCumulative =0;
    double angleRightCumulative =0;
    double angleLeftLastRead =0;
    double angleRightLastRead =0;
    double dt =0;
    double linearVelocity =0;
    Position2D pose;
Position2D deltaPose;

    void updateAngles(double left, double right);
    void updateAnglesFromSpeed(double leftSpeed, double rightSpeed);
    double getLinearVelocity();

    void updatePose();


private:
    double getDeltaLeftAngleSinceLastRead();
    double getDeltaRightAngleSinceLastRead();

    double prevSpeedUpdateTime=0; //initialized in constructor with valid time



};

#endif // ODOMETRY_H
