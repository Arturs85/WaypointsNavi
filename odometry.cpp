#include "odometry.h"
#include <cmath>
#include "trajectoryexecutor.h"

Odometry::Odometry()
{

    prevSpeedUpdateTime = TrajectoryExecutor::getSystemTimeSec();// initialize time, so that first dt calculation would be valid


}
//angles in radians
void Odometry::updateAngles(double left, double right)
{
    double time = TrajectoryExecutor::getSystemTimeSec();
    dt = time - prevSpeedUpdateTime;
    this->angleLeftCumulative=left;
    this->angleRightCumulative=right;
    prevSpeedUpdateTime = time;
    updatePose();

}

void Odometry::updateAnglesFromSpeed(double leftSpeed, double rightSpeed)
{
    double time = TrajectoryExecutor::getSystemTimeSec();
    dt = time - prevSpeedUpdateTime;

    angleLeftCumulative+=(leftSpeed*dt);
    angleRightCumulative+=(rightSpeed*dt);

    prevSpeedUpdateTime = time;
    updatePose();

}

double Odometry::getLinearVelocity()
{
    return linearVelocity;
}

double Odometry::getDeltaLeftAngleSinceLastRead()
{
    double delta  = angleLeftCumulative - angleLeftLastRead;
    angleLeftLastRead = angleLeftCumulative;
    return delta;
}

double Odometry::getDeltaRightAngleSinceLastRead()//call only from one point in code
{
    double delta  = angleRightCumulative - angleRightLastRead;
    angleRightLastRead = angleRightCumulative;
    return delta;
}

void Odometry::updatePose()
{
    double dAngleRight = getDeltaRightAngleSinceLastRead();
    double dAngleLeft = getDeltaLeftAngleSinceLastRead();

    double travelRight = dAngleRight*WHEEL_RADI;
    double travelLeft = dAngleLeft*WHEEL_RADI;
    double travel = (travelRight+travelLeft)/2;

    double deltaYaw=(travelRight-travelLeft)/WHEEL_BASE;



    double dx = travel * std::cos(pose.yaw+deltaYaw/2);
    double dy = travel * std::sin(pose.yaw+deltaYaw/2);
    pose.x+=dx;
    pose.y+=dy;
    pose.yaw+=deltaYaw;
    pose.yaw = std::remainder(pose.yaw,2*M_PI);
    linearVelocity = travel/dt;
  deltaPose.x =dx;
  deltaPose.y = dy;
  deltaPose.yaw = deltaYaw;

}

