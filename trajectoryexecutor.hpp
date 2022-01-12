#ifndef TRAJECTORYEXECUTOR_H
#define TRAJECTORYEXECUTOR_H
#include <cmath>

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

        return other.yaw-yaw;
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
    double lat; double lon; double yaw;
    Position2DGPS():lat(0),lon(0),yaw(0) {}
    Position2DGPS(double lat, double lon, double yaw):lat(lat),lon(lon),yaw(yaw) {}
    double distanceDegr(Position2DGPS other){
        return(std::sqrt((lat-other.lat)*(lat-other.lat)+(lon-other.lon)*(lon-other.lon)));

    }
    double distanceMeters(Position2DGPS other){
        return radiOfEarth * std::sqrt((lat-other.lat)*(lat-other.lat)+(lon-other.lon)*(lon-other.lon));

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
    double pc = 0.4;
    double ic = 0.01;
    double dc = 0.8;
    double maxI = 0.6;
    double p = 0;
    double i = 0;
    double d = 0;
    double deltaPrevious;
    bool first = true;
    void reset(){
        first = true;
        i=0;
        d=0;
    }
    double calcControlValue(double delta){
        p = delta;
        i += delta; //todo add limit
        if(std::abs(i)>maxI) i-=delta;
        if(first)first = false;
        else{
            d = delta - deltaPrevious;
        }
        deltaPrevious = delta;
        return pc*p+ic*i+dc*d;//pc*p+ic*i;//+dc*d;
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
    static constexpr double angVelMax = 0.5;//0.5 rad ~ 30 deg, 1.8; // rad /sec to limit linerar vel on platforms outside
    static constexpr double acc = 0.1;// m/s^2
    static constexpr double angAccel = 0.31;
    static constexpr double angAccelLo = 0.15;
    static constexpr double deltaYawRadForLo = 0.1;// shows when to use angAccelLo

    static  double getSystemTimeSec();
    void setTarget(Position2D targetPose);
    bool trajectoryStep();
    void resume();
    MotorControl* motorControl;
    Pid pidAngVel;
    double pidRatioAngVel = 0.5;
private: Odometry* odometry;
    Pid pidLinVel;
    Pid pidYaw;
    Position2D targetPos;
    //  double minRadius = 0.3;
    double desiredSpeed;
    double angVel = 0;
    double linVel =0;
    double radius =0;
    double lastUpdateTime;
    double lastUpdateDistance;
    double previousTime=0;
    double arrivedDistTreshold = 0.1;
    Position2D lastEstimatedPosition;

    Position2D calcDeltaEstimatedPosition();
    double calcDistanceToTarget(Position2D actualPos);
    int counter=0;
    DrivingStateTe drivingState = DrivingStateTe::PAUSED;
    // VaribleRadiusMotion* variRadiMotion;
    // bool varibleRadiMotionControl();// periodicaly adjusts radius of motion, to achieve fluent motion
    bool trajectoryStepPid();
};

#endif // TRAJECTORYEXECUTOR_H
