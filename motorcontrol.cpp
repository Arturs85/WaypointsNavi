#include "motorcontrol.h"
//#include "subscriber.h"
#include "odometry.h"
#include <iostream>
MotorControl::MotorControl(double track, double wheelRadius)
{
    this->track = track;
    this->wheelRadius = wheelRadius;
    odometryFromControl = new Odometry();

}

MotorControl::MotorControl(double track, double wheelRadius, Subscriber* subscriber)
{
    this->track = track;
    this->wheelRadius = wheelRadius;
    this->subscriber = subscriber;
    odometryFromControl = new Odometry();
}

void MotorControl::setSpeed(double speed, double radius)
{
  //  std::cout<<"setSpeed motorcontrol\n";
    //set given speed for outer wheel, calculate for other
    double track05 = track/2;
    if(radius<0){
        leftWheelSpeed=speed/wheelRadius; // converting from m/s to rad/s
        rightWheelSpeed = leftWheelSpeed*(radius+track05)/(radius-track05);
    }else
        if(radius>0){
            rightWheelSpeed=speed/wheelRadius;
            leftWheelSpeed = rightWheelSpeed*(radius-track05)/(radius+track05);

        }else{//todo - how to pass direction when turning about center
            rightWheelSpeed =0;
            leftWheelSpeed = 0;
        }
    sendWheelSpeeds();
    odometryFromControl->updateAnglesFromSpeed(leftWheelSpeed,rightWheelSpeed);

}






void MotorControl::calcWheelSpeeds()
{

}

void MotorControl::sendWheelSpeeds()
{
  std::cout<<TAG<<"left: "<<leftWheelSpeed<<" right: "<<rightWheelSpeed<<std::endl;
    // Subscriber::sendWheelSpeeds(leftWheelSpeed,rightWheelSpeed);
}
