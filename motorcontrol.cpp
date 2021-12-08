#include "motorcontrol.h"
//#include "subscriber.h"
#include "odometry.h"
#include <iostream>
#include "udpcommunication.hpp"
#include "control.hpp"
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
Control::particleFilter.onOdometry(odometryFromControl->deltaPose);
}

void MotorControl::setWheelSpeedsCenter(double speed, double radius)
{
    //  std::cout<<"setSpeed motorcontrol\n";
    //set given speed to center of plarform wheel, calculate wheel speeds
    double wb05 = Odometry::WHEELS_TRACK/2;
    //if(radius<0){
    leftWheelSpeed=speed*(radius-wb05)/radius/Odometry::WHEEL_RADI; // converting from m/s to rad/s
    rightWheelSpeed=speed*(radius+wb05)/radius/Odometry::WHEEL_RADI;
    //    }else
    //        if(radius>0){
    //            rightWheelSpeed=speed/Odometry::WHEEL_RADI;
    //            leftWheelSpeed = rightWheelSpeed*(radius-wb05)/(radius+wb05);

    if(std::abs(radius)<0.0001){//todo - how to pass direction when turning about center
        rightWheelSpeed =0;
        leftWheelSpeed = 0;
    }
    sendWheelSpeeds();
    odometryFromControl->updateAnglesFromSpeed(leftWheelSpeed,rightWheelSpeed);
    Control::particleFilter.onOdometry(odometryFromControl->deltaPose);


}






void MotorControl::calcWheelSpeeds()
{

}

void MotorControl::sendWheelSpeeds()
{
//  std::cout<<TAG<<"left: "<<leftWheelSpeed<<" right: "<<rightWheelSpeed<<std::endl;
UdpCommunication::platformMsgparser.sendMotorControl((int)rightWheelSpeed,(int)leftWheelSpeed);
  // Subscriber::sendWheelSpeeds(leftWheelSpeed,rightWheelSpeed);
}
