#ifndef TRAJECTORYEXECUTOR_H
#define TRAJECTORYEXECUTOR_H
#include <cmath>

class MotorControl;
class Odometry;
enum class DrivingState{TO_TARGET,ARRIVED};
class Position2D{
public:
    double x; double y; double yaw;
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

class TrajectoryExecutor
{
public:
    TrajectoryExecutor();
    void setTarget(double desiredRadius,double desiredSpeed, double endX,double endY);
    void setTarget(double desiredSpeed, double endX,double endY);

    bool tick();

    static  double getSystemTimeSec();
private: MotorControl* motorControl;
private: Odometry* odometry;

    Position2D targetPos;
    double minRadius = 0.3;
    double desiredSpeed;
    double angAccel = 0.11;//rad/s^2 0.017 rad = 1 deg
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
    DrivingState drivingState = DrivingState::ARRIVED;
    // VaribleRadiusMotion* variRadiMotion;
    // bool varibleRadiMotionControl();// periodicaly adjusts radius of motion, to achieve fluent motion
};

#endif // TRAJECTORYEXECUTOR_H
