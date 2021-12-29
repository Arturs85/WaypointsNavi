#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "trajectoryexecutor.hpp"//for Pose, replace with pose later- move pose to separate file
//#include "publisherbase.h"

class Odometry
{
public:

    static double constexpr WHEEL_RADI = 0.19;//0.036;//0.19;
    static double constexpr WHEELS_TRACK = 0.82;//0.232;//82;

    Odometry();
    double angleLeftCumulative =0;
    double angleRightCumulative =0;
    double angleLeftLastRead =0;
    double angleRightLastRead =0;
    double dt =0;
    double linearVelocity =0;
    double angVel =0; // for Trajectory solver

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
