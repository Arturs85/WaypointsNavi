#ifndef TRAJECTORYEXECUTOR_H
#define TRAJECTORYEXECUTOR_H
#include <cmath>
#include <iostream>

class MotorControl;
class Odometry;
enum class DrivingStateTe{TO_TARGET,ARRIVED, PAUSED};// is it needed at all
class Position2D{
public:
    double x; double y; double yaw;bool shouldStop=true;

    Position2D():x(0),y(0),yaw(0) {}
    Position2D(double x, double y, double yaw):x(x),y(y),yaw(yaw) {}
    double distance(Position2D other){
        return(std::sqrt((x-other.x)*(x-other.x)+(y-other.y)*(y-other.y)));

    }

    double calcYawPointToPoint(Position2D other){

        return std::atan2(other.y-y,other.x-x);
    }
    double calcDeltaYaw(Position2D other){

        return std::remainder(other.yaw-yaw,2*M_PI);
    }
    double calcGlobalYawOfPoint(){

        return std::atan2(y,x);
    }
    double calcYawPoseToPoint(Position2D other){
        double yawToOther = calcYawPointToPoint(other);
        return  yawToOther-yaw;
    }

};

class Position2DGPS{
public:
    static const int radiOfEarth = 6371000;//m (~ at LV)
    static constexpr double radiOfEarthForDegr = 111194.926644559;//m

    double lat; double lon; double yaw;
    Position2DGPS():lat(0),lon(0),yaw(0) {}
    Position2DGPS(double lat, double lon, double yaw):lat(lat),lon(lon),yaw(yaw) {}
    double distanceDegr(Position2DGPS other){
        return(std::sqrt((lat-other.lat)*(lat-other.lat)+(lon-other.lon)*(lon-other.lon)));

    }
    double distanceMeters(Position2DGPS other){
        double dist = radiOfEarthForDegr * std::sqrt((lat-other.lat)*(lat-other.lat)+(lon-other.lon)*(lon-other.lon));
        if(dist>5) std::cout<<"large gps dist: "<<dist<<" this: "<<lat<<" "<<lon<<" other: "<<other.lat<<" "<<other.lon <<std::endl;
        return dist;

    }
    double calcYawPointToPoint(Position2DGPS other){//north = 90 deg, east = 0 deg

        return std::atan2(other.lat-lat,other.lon-lon);
    }
    double calcDeltaYaw(Position2DGPS other){

        return other.yaw-yaw;
    }
    double calcGlobalYawOfPoint(){

        return std::atan2(lat,lon);
    }
    double calcYawPoseToPoint(Position2DGPS other){
        double yawToOther = calcYawPointToPoint(other);
        return  yawToOther-yaw;
    }

};

class Pid{
public:
    double weightNewDForLpf = 0.35; //  0.3 = heavy lpf , 0.6 = light  dampening
    double pc = 1.0; //0.8 was better than 0.5, 0.5 seems to be underactuated
    double ic = 0.02;
    double dc = 0.1;
    double maxI = 1.5;
    double p = 0;
    double i = 0;
    double d = 0;
    double previousP;
    bool first = true;
    void reset(){
        first = true;
        i=0;
        d=0;
    }
    void initFromControlValue(double cVal,double delta){ //for transition from angVel control to anglr control
        reset();
        i= (cVal-pc*delta)/ic;
        if(i<0 && i<-maxI){
            i = -maxI+0.01; //adding small value to ensure that i is below maxI
                    }
        if(i>0 && i>maxI){
            i = maxI-0.01; //adding small value to ensure that i is below maxI
                    }
    }
    double calcControlValue(double delta,double dt){
        p = delta;
        i += delta*dt; //todo add limit
        if(std::abs(i)>maxI) i-=(delta*dt);
        if(first)first = false;
        else{
            d = (p - previousP)/dt*weightNewDForLpf+(1-weightNewDForLpf)*d;
        }
         if(((p > 0) - (p < 0))!=((i > 0) - (i < 0)))i=0; //cler i, if it has opose sign to p
        previousP = delta;
        return pc*p+ic*i+dc*d;//pc*p+ic*i;//+dc*d;
    }
    double calcControlValue(double delta,double customIc,double dt){
        p = delta;
        i += delta*dt; //todo add limit
        if(std::abs(i)>maxI) i-=(delta*dt);
        if(first)first = false;
        else{
            d = (p - previousP)/dt*weightNewDForLpf+(1-weightNewDForLpf)*d;
        }
          if(((p > 0) - (p < 0))!=((i > 0) - (i < 0)))i=0; //cler i, if it has opose sign to p
        previousP = delta;
        return pc*p+customIc*i+dc*d;//pc*p+ic*i;//+dc*d;
    }

};
class Deliniariser{
public:
    double x1 =0.05;
    double x2 = 0.2;
    double y1 =0.15;
    double y2 = 0.2;
    double m1 =(y1)/(x1);//slope for l1
    double m2 =(y2-y1)/(x2-x1);//slope for l2
    double a2 = y1-m2*x1;//elevation of l2

    double delin(double v){
        int sign = (v > 0) - (v < 0);
        v = std::abs(v);
        if(v<x1)return sign*m1*v;//first line
        if(v<x2)return sign*(m2*v+a2);//second line
        return sign*v;// original value
    }

};

class TrajectoryExecutor
{
public:
    TrajectoryExecutor();
    void setTarget(double desiredRadius,double desiredSpeed, double endX,double endY);
    void setTarget(double desiredSpeed, double endX,double endY);
    void pause();
    bool tick();
    static constexpr double minRadius = 0.1;
    static constexpr double angVelMax = 0.6;//0.6;//0.5 rad ~ 30 deg, 1.8; // rad /sec to limit linerar vel on platforms outside
    static constexpr double angVelMaxStatic = 0.45;//0.6;//0.5 rad ~ 30 deg, 1.8; // rad /sec to limit linerar vel on platforms outside

    static constexpr double linVelMax = 0.7;//m/s
    static constexpr double linVelMin = 0.1;//m/s minLinvel from wich to stop movement
    static constexpr double decc = 0.2;// m/s^2
    static constexpr double acc = 0.1;// m/s^2
    static constexpr double angAccel = 0.9;
    // static constexpr double angAccelLo = 0.15;
    static constexpr double deltaYawRadPidSwitchTreshold = 0.3;// shows when to switch to angle control
    static constexpr double deltaYawArrived = 0.02; //0.02 rad ~= 1 degr
    static  double getSystemTimeSec();
    void setTarget(Position2D targetPose, double endLinVel=0);
    bool trajectoryStep();
    void resume();
    MotorControl* motorControl;
    Pid pidAngVel;
    Pid pidAngle;
    Pid pidLinVel;
    Pid pidAngVelStatic;
    double pidRatioAngVel = 0.7; // 0.5 was underactuated, 0.8 was better
    Deliniariser delin;
    bool adjustDirectionStepPid();

    bool trajStepBrakeToZero();
    bool trajectoryStepAngVelOnly();
private: Odometry* odometry;
    Pid pidYaw;
    Position2D targetPos;
    double targetEndLinVel =0;
    //  double minRadius = 0.3;
    double desiredSpeed;
    double angVel = 0;// to store calculated taret ang vel so it can be used also on more frequent angVel only updates
    double linVel =0;
    double radius =0;
    double lastUpdateTime;
    double lastUpdateDistance;
    double previousTime=0;
    double previousAngVelUpdateTime=0;
    double arrivedDistTreshold = 0.1;
    double approachingDist = 0.3;// at this distance start to look for distance increment to know when to stop
    bool useObstDetection = true;
    bool isAngleControl = false; // vs angVel control
    bool useAngleControl = true; // for faster toggle
    double distAvg =0;
    double distAvgLpfRatio = 0.7;
    double lastLinVelControl =0;  // to store linVelcontrol value obtained least freauently, so it can be used together with more frequent ang vel
    Position2D lastEstimatedPosition;
    double timeStart =0;
    Position2D calcDeltaEstimatedPosition();
    double calcDistanceToTarget(Position2D actualPos);
    int counter=0;
    DrivingStateTe drivingState = DrivingStateTe::PAUSED;
    // VaribleRadiusMotion* variRadiMotion;
    // bool varibleRadiMotionControl();// periodicaly adjusts radius of motion, to achieve fluent motion
    bool trajectoryStepPid();

    double getLinVelControl(double targetLinVel, double dt);
    double getAngVelControl(double targetAngVel, double dt);
    double inline calcCastorFactor(double actLinVel, double angVelActual);
};

#endif // TRAJECTORYEXECUTOR_H
