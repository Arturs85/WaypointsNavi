#include "motorcontrol.h"
//#include "subscriber.h"
#include "odometry.h"
#include <iostream>
#include "udpcommunication.hpp"
#include "control.hpp"
#include "uartRoomba.h"
#include "roombaController.hpp"
MotorControl::MotorControl(double track, double wheelRadius)
{
    this->track = track;
    this->wheelRadius = wheelRadius;
    odometryFromControl = new Odometry();


    uartRoomba.initialize();
    uartRoomba.startReceiveing();
    rc = new RoombaController(&uartRoomba);
    rc->startFull();
    uint16_t ca = rc->readBattCapacity();
    uint16_t ch = rc->readBattCharge();
    std::cout<<"batt ca: "<<ca<< ", ch: "<<ch<<" left: "<<(100*ch/++ca)<<" %\n";

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

void MotorControl::setWheelSpeedsCenter(double speed, double radius)// called from manual mode
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
    if(std::abs(leftWheelSpeed)> 0.0001 || std::abs(rightWheelSpeed)> 0.0001){// do not update particles, if we are in manual mode, but standing still. This is needed to reduce entries in log

        Control::particleFilter.onOdometryWGps(leftWheelSpeed,rightWheelSpeed);}
    //Control::particleFilter.onOdometry(odometryFromControl->deltaPose);
    rc->drive((int16_t)(speed*1000),(int16_t)(radius*1000));


}

void MotorControl::setWheelSpeedsFromAngVel(double linVel, double angVel)//called from auto mode
{
    //  std::cout<<"setSpeed motorcontrol\n";
    //set given speed to center of plarform wheel, calculate wheel speeds
    double wb05 = Odometry::WHEELS_TRACK/2;
    leftWheelSpeed=(linVel-angVel*wb05)/Odometry::WHEEL_RADI;
    rightWheelSpeed=(linVel+angVel*wb05)/Odometry::WHEEL_RADI;


    sendWheelSpeeds();
    odometryFromControl->updateAnglesFromSpeed(leftWheelSpeed,rightWheelSpeed);
    Control::particleFilter.onOdometryWGps(leftWheelSpeed,rightWheelSpeed);
    rc->drive((int16_t)(linVel*1000),(int16_t)(linVel/angVel*1000));


}




void MotorControl::calcWheelSpeeds()
{

}

void MotorControl::sendWheelSpeeds()
{
    //leftWheelSpeed /=20; rightWheelSpeed /=20;
    std::cout<<TAG<<"left: "<<leftWheelSpeed<<" right: "<<rightWheelSpeed<<std::endl;
    UdpCommunication::platformMsgparser.sendMotorControl((int)rightWheelSpeed*10,(int)leftWheelSpeed*10);
    // Subscriber::sendWheelSpeeds(leftWheelSpeed,rightWheelSpeed);
}
