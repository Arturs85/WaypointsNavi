#ifndef PATHEXECUTOR_HPP
#define PATHEXECUTOR_HPP
#include "trajectoryexecutor.hpp"
#include <vector>
#include <cstddef> //for size_t on rpi
#include <wiringPi.h>
#include <thread>
#include <unistd.h>
//#include "motorcontrol.h"
//#include "logfilesaver.hpp"

enum class DrivingState{DEPARTING,STRIGHT,ARRIVAL,IDLE,TO_TARGET,PAUSED,ADJUSTING_DIR,BRAKEING,INTERRUPTED,FINISHED};

class Waypoint{


    size_t curIndexInTrajectory =0;
public:
    std::vector<Position2D> trajectory;
    Waypoint(){ }//

    Waypoint(Position2D end, double dwellTimeSec = 0.0,int triggerOutputPin =0,int orientToYaw=0){
        trajectory.push_back(end); this->dwellTimeSec = dwellTimeSec;this->orientToYaw= orientToYaw,this->triggerOutputPin = triggerOutputPin;}// trajectory with endpoint only
    void resetTrjectory(){curIndexInTrajectory =0;}
    Position2D* getNextPointOfTrajectory(){
        if(trajectory.size()<=curIndexInTrajectory) return 0;
        return &(trajectory.at(curIndexInTrajectory++));
    }
    double dwellTimeSec = 0.0;
    int triggerOutputPin = 0;
    int orientToYaw = 0;
};

class GpioControl{

    double delayAfterStartSec =1;
    double pulseLengthSec =0.1;
    double startTime = 0;
    double endTime =0;

    void tick(){
        while(1){
            double time = TrajectoryExecutor::getSystemTimeSec();
            if(time>startTime && time<endTime){
                digitalWrite(0, 1);
                digitalWrite(1, 1);

            }else if(time>endTime){
                digitalWrite(0, 0);
                digitalWrite(1, 0);
                break;
            }
            usleep(10000);
        }
    }
public:
    GpioControl(){wiringPiSetup();                  // Setup the library
                 }
    void start(){
        pinMode(0, OUTPUT);		// Configure GPIO0 as an output
        pinMode(1, OUTPUT);		// Configure GPIO0 as an output
        digitalWrite(0, 0);
        digitalWrite(1, 0);
        startTime = TrajectoryExecutor::getSystemTimeSec()+delayAfterStartSec;
        endTime = startTime+pulseLengthSec;
        std::thread t1(&GpioControl::tick,*this); // passing 'this' by value
        t1.detach();
    }


};


class PathExecutor
{
public:
    PathExecutor();
    void tick();
    void enterPausedState();
    void resumeFromPause();
    bool startPath();
    TrajectoryExecutor te;

    void startOrientToYaw();
    void checkGpsAge();
    void loadPointsFile(std::string fileName);
    void tickAngVelOnly();
     bool useObstacleDetection = true;
     bool repeatOn = false;
private:
    Waypoint* curWp=0;
 //   void setTarget(Position2D t);
    void startDwell(double timeSec);
    Position2D *switchToNextWaypoint();
    std::size_t currentWaypointIndex=0;
    std::vector<Waypoint> wayPoints;
    double dwellTimeEnd =0;
    DrivingState state = DrivingState::PAUSED;
    DrivingState previousState = DrivingState::TO_TARGET;
    bool hasStarted = false;

    void enterFinishedState();
};

#endif // PATHEXECUTOR_HPP
