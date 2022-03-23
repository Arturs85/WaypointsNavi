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
    timeStart = TrajectoryExecutor::getSystemTimeSec();
    motorControl = new MotorControl(Odometry::WHEELS_TRACK,Odometry::WHEEL_RADI);
    odometry = motorControl->odometryFromControl;
    pidLinVel.pc =0.5;
    pidLinVel.ic =0.1;
    pidLinVel.dc =0.2;
    pidAngVelStatic.pc =0.1;
    pidAngVelStatic.ic =0.08;
    pidAngVelStatic.dc =0.2;
    pidAngVelStatic.maxI= 2.5; //todo right value

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
    pidAngVelStatic.reset();
    std::cout<<"ang vel = 0, te.pause() "<<std::endl;
}

void TrajectoryExecutor::resume(){

    drivingState = DrivingStateTe::TO_TARGET;
    angVel =0;
    linVel = 0;
}

void TrajectoryExecutor::setTarget(Position2D targetPose, double endLinVel){ //to arrive in point with orientation
    // linVel =0;// todo
    targetPos = targetPose;
    targetEndLinVel = endLinVel; // for intermediate points
    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.avgParticle.direction);

    distAvg =    0.1 + targetPos.distance(curPose)*ParticleFilter::radiOfEarthForDegr; // dist in meters, adding small value, to avoid unexpected arrive condition, if next position would be further from target, as avg filter is not jet initialised

    std::cout<<std::setprecision(9);

    std::cout<<"[TE] target.x "<<targetPose.x<<" target.y "<<targetPose.y<<" endLinVel: "<<targetEndLinVel<<std::endl;

}
double TrajectoryExecutor::getLinVelControl(double targetLinVel,double dt){
    // linVelPid
    double linVelActual = std::abs(Control::particleFilter.linVelGpsLpf);
    double linVelPid = pidLinVel.calcControlValue(linVel-linVelActual,dt);
    double linVelContr = linVel*1.0+ linVelPid;// linVelSet*0.3+linVel; // adding pid to model

    return linVelContr;
}
double TrajectoryExecutor::getAngVelControl(double targetAngVel,double dt){
    double angVelActual = Control::particleFilter.lastGyroAngVelRad;

    //ang vel I proportional to linear vel
    double linVelRatio = std::abs((linVelMax-linVel)*1.5/linVelMax);
    double angVelRatio = std::abs(0.25/angVelActual);
    if(angVelRatio>3)angVelRatio =3;
    double icAvLocal = pidAngVel.ic+pidAngVel.ic*linVelRatio;//*angVelRatio;
    double angVelSet = pidAngVel.calcControlValue(angVel-angVelActual,icAvLocal,dt);
    targetAngVel= 1.3*angVel+2*pidRatioAngVel*angVelSet;
    return targetAngVel;

}

bool TrajectoryExecutor::trajStepBrakeToZero(){
    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousTime;
    if(dt>0.3){previousTime = time;return false;}
    bool linVelZero = false;
    bool angVelZero = false;

    if(linVel - dt*decc>0 )   linVel -=dt*decc;
    else{
        linVel =0;
        linVelZero = true;
    }

    if(angVel<0){
        if(angVel+dt*angAccel<0)
            angVel+= dt*angAccel;
        else{
            angVel =0;
            angVelZero = true;
        }
    }
    else if(angVel>0){
        if(angVel-dt*angAccel>0)
            angVel-= dt*angAccel;
        else {angVel =0;
            angVelZero = true;
        }
    }else   angVelZero = true;

    double lv = getLinVelControl(linVel,dt);
    if(lv<0)lv =0; // do not use reverse motion, because we have no means to detect actual direction of platform- we only have absolute value from gps

    lastLinVelControl = lv;
    double av = getAngVelControl(angVel,dt);
    motorControl->setWheelSpeedsFromAngVel(lv,av);
    if(++counter % 6 == 0)
        std::cout<<"[TE] btz angVel: "<<angVel<<" linVel: "<<linVel<< std::endl;
    return angVelZero && linVelZero;

}
/**
 * returns values in range 0 ..1
 */
double TrajectoryExecutor::calcCastorFactor(double linVelActual, double angVelActual){
    double actualRadi = linVelActual/angVelActual;// use angle of platform front wheels to estimate additional force needed to turn them
    if(std::abs(angVelActual)<0.0001)  actualRadi = linVelActual/0.0001; // to avoid divBy0
    double actualCastorAngle = std::atan(Odometry::WHEELBASE/actualRadi);

    double targetlRadi = linVel/angVel;
    if(std::abs(angVel)<0.0001)  targetlRadi = linVel/0.0001; // to avoid divBy0
    double targetCastorAngle = std::atan(Odometry::WHEELBASE/targetlRadi);

    double deltaCastor = std::abs(targetCastorAngle)-std::abs(actualCastorAngle);

    double factor =  std::abs(deltaCastor)/M_PI_2;
    if(factor>1) factor =1;
    return factor;
}
bool TrajectoryExecutor::trajectoryStepAngVelOnly(){     // uses targetAv calculated least frequently, just calculating motor controls on latest gyro
    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousAngVelUpdateTime;
    if(dt>0.3){previousAngVelUpdateTime = time;return false;}// to avoid large dt after waiting

    double avContol = getAngVelControl(angVel,dt);
    motorControl->setWheelSpeedsFromAngVel(lastLinVelControl,avContol);

    previousAngVelUpdateTime = time;
    return false;

}
bool TrajectoryExecutor::trajectoryStepPid(){


    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousTime;
    if(dt>0.3){previousTime = time;return false;}// to avoid large dt after waiting
    double localAngAcc = angAccel;
    //    Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.avgParticle.direction);
    //Position2D curPose(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.dirComplRad);//avgParticle.direction);
    Position2DGPS pos;
    Control::particleFilter.getGpsPosition(pos);
    Position2D curPose(pos.lon,pos.lat,Control::particleFilter.dirComplRad);// using unfiltred x and y from gps

    double dist = targetPos.distance(curPose)*ParticleFilter::radiOfEarthForDegr; // dist in meters
    distAvg = distAvg*distAvgLpfRatio+dist*(1-distAvgLpfRatio);
    if(distAvg<approachingDist && distAvg>lastUpdateDistance){ UiUdp::uiParser.sendText("reached pt at dist:  "+std::to_string(distAvg));return true;}

    //direction
    double deltaYaw;

    deltaYaw =curPose.calcYawPoseToPoint(targetPos);
    deltaYaw = std::remainder(deltaYaw,2*M_PI); // normalize to -pi;pi

    double angVelActual = Control::particleFilter.lastGyroAngVelRad;
    double linVelActual = std::abs(Control::particleFilter.linVelGpsLpf);

    //calc desired angVel at this deltaYaw
    double targetAngVel;
    targetAngVel =std::sqrt(2*angAccel*std::abs(deltaYaw)); //use only lo ang acc

    if(targetAngVel>angVelMax){//we need to decrease abs value of ang vel,because we are close to target direction
        targetAngVel = angVelMax;
    }
    if(deltaYaw<0)targetAngVel *=-1;
    double angAccSign = std::abs(targetAngVel-angVel)/(targetAngVel-angVel);
    localAngAcc *= angAccSign;
    // if we are close to target then reduce abs ang vel to avoid changing approach direction, because it looks wrong
    if(dist<approachingDist){
        if(angVel<0){
            if(angVel+dt*angAccel<0)
                angVel+= dt*angAccel;

        }
        else if(angVel>0){
            if(angVel-dt*angAccel>0)
                angVel-= dt*angAccel;

        }
    }else    if(std::abs(angVel+dt*localAngAcc)<angVelMax) angVel+=dt*localAngAcc; // this is trying to increase ang ve in directio of target


    if(std::abs(targetAngVel)<std::abs(angVel))angVel = targetAngVel;// use smallest value
    //    double angVelSet = pidAngVel.calcControlValue(angVel-Control::particleFilter.lastGyroAngVelRad);
    //    targetAngVel= 1.3*angVel+2*pidRatioAngVel*angVelSet;

    //linear vel;
    double curLinVelTarget =0;
    double linVelMaxCur = linVelMax*((M_PI/2-std::abs(deltaYaw))/(M_PI/2));//?
    // if(useObstDetection) {if(Control::uartUltra.distances.hasObstacleFront())linVelMaxCur =0;}
    if(linVelMaxCur<0) linVelMaxCur =0;
    if(linVel + dt*acc<linVelMaxCur)     linVel +=dt*acc; // increase linVel
    if(linVel - dt*decc>linVelMaxCur )   linVel -=dt*decc; // decrese target linVel towards value dictated by dYaw
    double linVelDecc = std::sqrt(2*decc*(dist))+targetEndLinVel;// min speed that is needed to be able to reach distance at end vel - if end Vel is same as normal vel, it wont affect targetVel
    if(linVelDecc<linVel) linVel = linVelDecc; // override target linVel, if we are approaching target
    //ang vel I proportional to linear vel
    double linVelRatio = std::abs((linVelMax-linVel)*1.5/linVelMax);
    double angVelRatio = std::abs(0.25/angVelActual);
    if(angVelRatio>3)angVelRatio =3;
    double icAvLocal = pidAngVel.ic+pidAngVel.ic*linVelRatio;//*angVelRatio;
    double angVelSet = pidAngVel.calcControlValue(angVel-angVelActual,icAvLocal,dt);
    double castorFactor = 0.20*calcCastorFactor(linVelActual,angVelActual);// adjust multiplier for smooth operation
    if(deltaYaw<0) castorFactor *= -1;
    targetAngVel= 1.3*angVel+castorFactor+2*pidRatioAngVel*angVelSet;

    // linVelPid
    double linVelPid = pidLinVel.calcControlValue(linVel-linVelActual,dt);
    double linVelContr = linVel*1.0+ linVelPid;// linVelSet*0.3+linVel; // adding pid to model
    if(linVelContr<0)linVelContr =0; // do not use reverse motion, because we have no means to detect actual direction of platform- we only have absolute value from gps

    lastLinVelControl = linVelContr;

    motorControl->setWheelSpeedsFromAngVel(linVelContr,targetAngVel);

    if(++counter % 6 == 0)
        std::cout<<"dist: "<<dist<<" dYaw: "<<deltaYaw*180/M_PI<<" actAV: "<<angVelActual<<" targAV: "<<angVel<<" linVelTarg: "<<linVel<<" linVelAct: "<<linVelActual<<" linVelPid: "<<linVelPid<<" avset: "<< angVelSet<<" avI: "<<pidAngVel.i*icAvLocal<<" avD: "<<pidAngVel.d<<" castFact: "<<castorFactor<<" t: "<<(time-timeStart
                                                                                                                                                                                                                                                                                                                        )<<std::endl;

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
    if(std::abs(deltaYaw)<deltaYawArrived){motorControl->setWheelSpeedsCenter(0,0); UiUdp::uiParser.sendText("reached angle deg :  "+std::to_string(deltaYaw*180/M_PI));return true;}

    double angVelActual = Control::particleFilter.lastGyroAngVelRad;
    double linVelActual = std::abs(Control::particleFilter.linVelGpsLpf);

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



    //double angVelSet = pidAngVel.calcControlValue(angVel-Control::particleFilter.lastGyroAngVelRad);
    //ang vel I proportional to linear vel
    double linVelRatio = std::abs((linVelMax-linVel)*2/linVelMax);
    double angVelRatio = std::abs(0.1/angVelActual);
    if(angVelRatio>1.5)angVelRatio =1.5; //todo change to linear
    double icAvLocal = pidAngVelStatic.ic+pidAngVelStatic.ic*linVelRatio*angVelRatio;

    double angVelSet = pidAngVelStatic.calcControlValue(angVel-angVelActual,icAvLocal,dt);
    double castorFactor = 0.20*calcCastorFactor(linVelActual,angVelActual);// adjust multiplier for smooth operation
    if(deltaYaw<0) castorFactor *= -1;
    targetAngVel= 1.3*angVel+castorFactor+2*pidRatioAngVel*angVelSet;

    //if(targetAngVel< 0.15 )targetAngVel = 0; // clamp to 0 near 0, todo test

    motorControl->setWheelSpeedsFromAngVel(0,targetAngVel);
    // std::cout<<"dYaw: "<<deltaYaw*180/M_PI<<" actAV: "<<Control::particleFilter.lastGyroAngVelRad<<" targAV: "<<angVel<<" avset: "<< angVelSet<<std::endl;
    if(++counter % 6 == 0)
        std::cout<<"dYaw: "<<deltaYaw*180/M_PI<<" actAV: "<<Control::particleFilter.lastGyroAngVelRad<<" targAV: "<<angVel<<" avset: "<< angVelSet <<" avI: "<<pidAngVelStatic.i*icAvLocal<<" avD: "<<pidAngVelStatic.d<<" castFact: "<<castorFactor<<std::endl;

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
