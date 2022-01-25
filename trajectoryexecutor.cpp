#include "trajectoryexecutor.hpp"
#include <sys/time.h>
#include "odometry.h"
#include "motorcontrol.h"
#include <iostream>
#include "control.hpp"
#include <iomanip>      // std::setprecision
#include "uiudp.hpp"
TrajectoryExecutor::TrajectoryExecutor()
{

    motorControl = new MotorControl(Odometry::WHEELS_TRACK,Odometry::WHEEL_RADI);
    odometry = motorControl->odometryFromControl;
    pidLinVel.pc =0.02;
    pidLinVel.ic =0.02;
    pidLinVel.dc =0.02;

}

void TrajectoryExecutor::setTarget(double desiredRadius, double desiredSpeed, double endX, double endY)
{
    targetPos.x=endX;
    targetPos.y=endY;
    this->desiredSpeed = desiredSpeed;
    //  this->desiredRadius = desiredRadius;
    lastUpdateDistance =targetPos.distance(odometry->pose);

    motorControl->setSpeed(desiredSpeed,desiredRadius);
}

void TrajectoryExecutor::setTarget(double desiredSpeed, double endX, double endY)
{
    targetPos.x=endX;
    targetPos.y=endY;
    this->desiredSpeed = desiredSpeed;
    lastUpdateDistance =targetPos.distance(odometry->pose);
    linVel = desiredSpeed;
    // double targetYaw = targetPos.calcGlobalYawOfPoint();
    //variRadiMotion = new VaribleRadiusMotion(odometry->pose.yaw,targetYaw,motorControl->odometryFromControl);
    //  drivingState=DrivingState::TO_TARGET;
    previousTime = TrajectoryExecutor::getSystemTimeSec();
}

void TrajectoryExecutor::pause()
{
    drivingState = DrivingStateTe::PAUSED;
    motorControl->setWheelSpeedsCenter(0,0);
    angVel =0;
    linVel =0;
    pidAngVel.reset();
    pidLinVel.reset();
    std::cout<<"ang vel = 0, te.pause() "<<std::endl;
}

void TrajectoryExecutor::resume(){

    drivingState = DrivingStateTe::TO_TARGET;
    angVel =0;
    linVel = 0;
}

void TrajectoryExecutor::setTarget(Position2D targetPose){ //to arrive in point with orientation
    linVel =0;// todo
    targetPos = targetPose;
    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.avgParticle.direction);

    distAvg =    0.1 + targetPos.distance(curPose)*ParticleFilter::radiOfEarthForDegr; // dist in meters, adding small value, to avoid unexpected arrive condition, if next position would be further from target, as avg filter is not jet initialised

    std::cout<<std::setprecision(9);

    std::cout<<"[TE] target.x "<<targetPose.x<<" target.y "<<targetPose.y<<std::endl;

}
bool TrajectoryExecutor::trajectoryStep(){
    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousTime;
    if(dt>0.3){previousTime = time;return false;}// to avoid large dt after waiting

    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.avgParticle.direction);
    double dist =targetPos.distance(curPose);
    if(dist < arrivedDistTreshold){motorControl->setWheelSpeedsCenter(0,0); return true;}
    //linear vel;
    double linVelMax = std::abs(minRadius*angVelMax);//?
    double linVel =Control::particleFilter.avgParticle.linearVel; // odometry->linearVelocity;// read actual(from control) lin vel from odometry

    double accSign = (linVelMax-linVel)/std::abs((linVelMax-linVel)); //-1 or +1
    double linVelDelta = dt*acc*accSign;
    if(std::abs(linVelMax-linVel)>1.1* dt*acc ) linVel+=linVelDelta; // change l=speed only if it is not close to target speed

    //direction
    double deltaYaw;

    deltaYaw =curPose.calcYawPoseToPoint(targetPos);
    deltaYaw = std::remainder(deltaYaw,2*M_PI); // normalize to -pi;pi
    // if(std::abs(deltaYaw)< 0.005)return true;
    //determine the sign of angular acceleration
    double angVelActual = std::abs(Control::particleFilter.avgParticle.angVel);//odometry->angVel);
    double angAccToZero = angVelActual*angVelActual/(2*std::abs(deltaYaw));
    double angAccSign =1;
    if(angAccToZero > angAccel){// negative angular acceleration to increase turning radius
        angAccSign = -1;
    }
    double   angVelDelta = dt*angAccel*angAccSign;
    if(std::abs(angVel)<0.0001)angVel = 0.0001; // avoid possible div/zero
    double radius = 1000;// for first step when there is no movement in odometry yet
    double angVelSet = pidAngVel.calcControlValue(angVel + angVelDelta-angVelActual);

    if(std::abs(Control::particleFilter.avgParticle.linearVel)>0.0001)  radius = linVel/(angVel+angVelDelta);

    if(std::abs(radius)<minRadius) radius = minRadius;// clamp to min radius according to physical properties of platform
    else{
        angVel+=angVelDelta; // updatea ang vel only if we are actually changing it
        if(std::abs(angVel)>=angVelMax) angVel -=angVelDelta; //remove accel, if ang vel become to large (angular speed limitation to angVelMax)

    }
    if(deltaYaw<0)
        radius*=-1;
    motorControl->setWheelSpeedsCenter(linVel,radius);
    //odo->updateAnglesFromSpeedSimTime(leftWheelSpeed,rightWheelSpeed);

    std::cout<<"dist: "<<dist<< " odo x: "<<odometry->pose.x<<" odo y: "<<odometry->pose.y<<" dir: "<<odometry->pose.yaw<<" dYaw: "<<deltaYaw*180/M_PI<<" radi: "<<radius<<" angVelLocal: "<<angVel<<" avDelta: "<<angVelDelta<<" avset: "<< angVelSet<<std::endl;

    previousTime = time;
    lastUpdateDistance = dist; // ist his needed, just copied from tick()?
    return false;


}
bool TrajectoryExecutor::trajectoryStepPid(){


    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousTime;
    if(dt>0.3){previousTime = time;return false;}// to avoid large dt after waiting
    double localAngAcc = angAccel;
    //    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.avgParticle.direction);
    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.dirComplRad);//avgParticle.direction);

    double dist = targetPos.distance(curPose)*ParticleFilter::radiOfEarthForDegr; // dist in meters
    distAvg = distAvg*distAvgLpfRatio+dist*(1-distAvgLpfRatio);
    if(distAvg<approachingDist && distAvg>lastUpdateDistance){motorControl->setWheelSpeedsCenter(0,0); UiUdp::uiParser.sendText("reached pt at dist:  "+std::to_string(distAvg));return true;}
    // if(dist < arrivedDistTreshold){motorControl->setWheelSpeedsCenter(0,0); return true;}

    //double linVel =Control::particleFilter.avgParticle.linearVel; // odometry->linearVelocity;// read actual(from control) lin vel from odometry

    //   double accSign = (linVelMax-linVel)/std::abs((linVelMax-linVel)); //-1 or +1
    //   double linVelDelta = dt*acc*accSign;
    //   if(std::abs(linVelMax-linVel)>1.1* dt*acc ) linVel+=linVelDelta; // change l=speed only if it is not close to target speed
    //linVel = 0.1;

    //direction
    double deltaYaw;

    deltaYaw =curPose.calcYawPoseToPoint(targetPos);
    deltaYaw = std::remainder(deltaYaw,2*M_PI); // normalize to -pi;pi
    //if(std::abs(deltaYaw)<0.1) localAngAcc = angAccel/2;
    //determine the sign of angular acceleration
    // if(deltaYaw<0)localAngAcc= -1*std::abs(localAngAcc);//use negative ang acc to acheieve turning to other direction wo jump of wheel speeds
    // double angVelActual = std::abs(Control::particleFilter.avgParticle.angVel);//odometry->angVel);
    double angVelActual = std::abs(Control::particleFilter.lastGyroAngVelRad);
    //calc desired angVel at this deltaYaw
    double targetAngVel;
    //if(std::abs(deltaYaw)<deltaYawRadForLo)
    targetAngVel =std::sqrt(2*angAccel*std::abs(deltaYaw)); //use only lo ang acc
    //else //  using lo ang accc for whole low end and rest of angle with normal ang acc
    //  targetAngVel = std::sqrt(2*angAccelLo*std::abs(deltaYawRadForLo))+ std::sqrt(2*angAccel*(std::abs(deltaYaw)-deltaYawRadForLo));// this targetAngVel is used only when dyaw is small, so we can alter its sign based on dyaw, and do not worry about jump in  wheel speeds

    if(targetAngVel>angVelMax){//we need to decrease abs value of ang vel,because we are close to target direction
        targetAngVel = angVelMax;
    }
    if(deltaYaw<0)targetAngVel *=-1;
    double angAccSign = std::abs(targetAngVel-angVel)/(targetAngVel-angVel);
    localAngAcc *= angAccSign;
    if(std::abs(angVel+dt*localAngAcc)<angVelMax) angVel+=dt*localAngAcc; // updatea ang vel only if we are actually changing it

    //  if(std::abs(angVel)>=angVelMax) angVel -= dt*localAngAcc;// angVel was exceeded by applying localAngAcc, so remove it to stay within bounds

    // if(deltaYaw<0) targetAngVel*=-1;
    if(std::abs(targetAngVel)<std::abs(angVel))angVel = targetAngVel;// use smallest value
    double angVelSet = pidAngVel.calcControlValue(angVel-Control::particleFilter.lastGyroAngVelRad);
    double delinAv = delin.delin(angVel);
    targetAngVel= 1.3*delinAv+2*pidRatioAngVel*angVelSet;
    //if(std::abs(targetAngVel< 0.15 )targetAngVel = 0; // clamp to 0 near 0, todo test

    //linear vel;
    double linVelMaxCur = linVelMax*std::abs(deltaYaw)/M_PI;//?
    linVel +=dt*acc;
    if(linVel>linVelMaxCur) linVel = linVelMax;
    double linVelDecc = std::sqrt(2*acc*(dist-arrivedDistTreshold));
    if(linVelDecc<linVel) linVel = linVelDecc;

    // linVelPid
    //double linVelActual = std::abs(Control::particleFilter.avgParticle.linearVel);
    double linVelActual = std::abs(Control::particleFilter.linVelGpsLpf);
    double linVelPid = pidLinVel.calcControlValue(linVel-linVelActual);
    double linVelContr = linVel*1.0+ linVelPid;// linVelSet*0.3+linVel; // adding pid to model
    // if(std::abs(targetAngVel)<0.0001)targetAngVel = 0.0001; // avoid possible div/zero
    //  double radius = 1000;// for first step when there is no movement in odometry yet

    //   if(std::abs(Control::particleFilter.avgParticle.linearVel)>0.0001)  radius = linVel/targetAngVel;//angVelSet;//(angVel+angVelDelta);

    // if(std::abs(radius)<minRadius) radius = minRadius;// clamp to min radius according to physical properties of platform

    //motorControl->setWheelSpeedsCenter(linVel,radius);
    motorControl->setWheelSpeedsFromAngVel(linVelContr,targetAngVel);
    //odo->updateAnglesFromSpeedSimTime(leftWheelSpeed,rightWheelSpeed);

    // std::cout<<"dist: "<<dist<<" dYaw: "<<deltaYaw*180/M_PI<<" radi: "<<radius<<" targAV: "<<targetAngVel<<" linVelFinal: "<<linVelSet<<" avset: "<< angVelSet<<" locAngAcc: "<<localAngAcc<<std::endl;
    std::cout<<"dist: "<<dist<<" dYaw: "<<deltaYaw*180/M_PI<<" actAV: "<<Control::particleFilter.lastGyroAngVelRad<<" targAV: "<<angVel<<" tarAVdelin: "<<delinAv<<" linVelTarg: "<<linVel<<" linVelAct: "<<linVelActual<<" linVelPid: "<<linVelPid<<" avset: "<< angVelSet<<" locAngAcc: "<<localAngAcc<<std::endl;

    previousTime = time;
    lastUpdateDistance = distAvg; // ist his needed, just copied from tick()?
    return false;


}

bool TrajectoryExecutor::adjustDirectionStepPid(){


    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousTime;
    if(dt>0.3){previousTime = time;return false;}// to avoid large dt after waiting
    double localAngAcc = angAccel;
    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.dirComplRad);//avgParticle.direction);


    //  if(distAvg<approachingDist && distAvg>lastUpdateDistance){motorControl->setWheelSpeedsCenter(0,0); UiUdp::uiParser.sendText("reached pt at dist:  "+std::to_string(distAvg));return true;}

    //direction
    double deltaYaw;

    deltaYaw = curPose.calcDeltaYaw(targetPos);
    if(std::abs(deltaYaw)<deltaYawArrived){motorControl->setWheelSpeedsCenter(0,0); UiUdp::uiParser.sendText("reached angle:  "+std::to_string(deltaYaw));return true;}

    double angVelActual = std::abs(Control::particleFilter.lastGyroAngVelRad);
    //calc desired angVel at this deltaYaw
    double targetAngVel;
    targetAngVel =std::sqrt(2*angAccel*std::abs(deltaYaw)); //use only lo ang acc

    if(targetAngVel>angVelMax){
        targetAngVel = angVelMax;
    }
    if(deltaYaw<0)targetAngVel *=-1;
    double angAccSign = std::abs(targetAngVel-angVel)/(targetAngVel-angVel);
    localAngAcc *= angAccSign;
    if(std::abs(angVel+dt*localAngAcc)<angVelMax) angVel+=dt*localAngAcc; // updatea ang vel only if we are actually changing it

    if(std::abs(targetAngVel)<std::abs(angVel))angVel = targetAngVel;// use smallest value
    double angVelSet = pidAngVel.calcControlValue(angVel-Control::particleFilter.lastGyroAngVelRad);

    targetAngVel= 1.3*angVel+2*pidRatioAngVel*angVelSet;
    //if(targetAngVel< 0.15 )targetAngVel = 0; // clamp to 0 near 0, todo test

    motorControl->setWheelSpeedsFromAngVel(0,targetAngVel);
    std::cout<<"dYaw: "<<deltaYaw*180/M_PI<<" actAV: "<<Control::particleFilter.lastGyroAngVelRad<<" targAV: "<<angVel<<" avset: "<< angVelSet<<std::endl;

    previousTime = time;
    return false;


}

bool TrajectoryExecutor::tick() // return true if dest point reached
{
    //std::cout<<"trajectory executor tick"<<std::endl;
    //  double time = TrajectoryExecutor::getSystemTimeSec();
    //  double dt = time - previousTime;
    switch (drivingState) {//do we need switch here
    case DrivingStateTe::TO_TARGET:{

        return trajectoryStepPid();
        //        //  double deltaYaw = motorControl->odometryFromControl->pose.calcYawToPoint(targetPos);
        //        double deltaYaw = odometry->pose.calcYawPoseToPoint(targetPos);
        //        deltaYaw = std::remainder(deltaYaw,2*M_PI); // normalize to -pi;pi
        //        //determine the sign of angular acceleration
        //        double angAccToZero = angVel*angVel/(2*std::abs(deltaYaw));
        //        double angAccSign =1;
        //        if(angAccToZero > angAccel){// negative angular acceleration to increase turning radius
        //            angAccSign = -1;
        //        }
        //        double   angVelDelta = dt*angAccel*angAccSign;
        //        if(std::abs(angVel)<0.0001)angVel = 0.0001; // avoid possible div/zero
        //        radius = odometry->getLinearVelocity()/(angVel+angVelDelta);

        //        if(radius<minRadius)
        //            radius = minRadius;// clamp to min radius according to physical properties of platform
        //        else angVel+=angVelDelta; // updatea ang vel only if we are actually changing it

        //        if(deltaYaw<0)
        //            radius*=-1;
        //        //compare actual distance to target with estimated(from odometry)
        //        //  std::cout<<"trajectory executor tick \n";
        //        //  motorControl->setSpeed(desiredSpeed,desiredRadius);
        //        counter++;
        //        double dist =targetPos.distance(odometry->pose);
        //        if(counter%1==0){
        //              std::cout<<"trajexec dist to target = "<<dist<<" angVel: "<<angVel<<" radi: "<<radius<<" dYaw: "<<deltaYaw<<" angacctoz : "<<angAccToZero<<" odyaw: "<<odometry->pose.yaw<<" linVelOdo: "<<odometry->getLinearVelocity()<<" \n";

        //        }
        //        if(dist < arrivedDistTreshold){//stop movement
        //            //  motorControl->setSpeed(0,0);
        //            //  targetPos.y*=-1;
        //            drivingState = DrivingStateTe::ARRIVED;
        //            std::cout<<"trajectory executor set to stop motors \n";
        //        }else{

        //            motorControl->setSpeed(linVel,radius);

        //        }

        //       lastUpdateDistance = dist;
    }
        break;
    case DrivingStateTe::ARRIVED:{
        //targetPos.y*=-1;
        //drivingState = DrivingState::TO_TARGET;

        return true;
    }
        break;
    case DrivingStateTe::PAUSED:{
        //do nothing?

    }
    default:
        break;
    }

    //previousTime = time;
    //
    return false;

}

Position2D TrajectoryExecutor::calcDeltaEstimatedPosition()
{

}

double TrajectoryExecutor::calcDistanceToTarget(Position2D actualPos)
{
    return actualPos.distance(targetPos);
}

//bool TrajectoryExecutor::varibleRadiMotionControl()
//{
//    double radi = variRadiMotion->tick();

//    if(std::abs(radi)<0.1) return true;
//    motorControl->setSpeed(desiredSpeed,radi);
//    return false;
//}
double TrajectoryExecutor::getSystemTimeSec(void){
    struct timeval start_time;
    double milli_time;
    gettimeofday(&start_time, NULL);

    milli_time = ((start_time.tv_usec) / 1000000.0 + start_time.tv_sec);
    return milli_time;
}
