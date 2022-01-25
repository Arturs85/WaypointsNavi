#include "pathexecutor.hpp"
#include "waypointsfilesaver.hpp"
#include <iostream>
PathExecutor::PathExecutor()
{
    //    wayPoints.push_back(Waypoint(Position2D(25.767017,56.649878,0)));
    //    wayPoints.push_back(Waypoint(Position2D(25.767136,56.649861,0)));
    //    wayPoints.push_back(Waypoint(Position2D(25.767153,56.649918,0)));
    //    wayPoints.push_back(Waypoint(Position2D(25.767014,56.649934,0)));

    WaypointsFileSaver::waypointsFileSaver.readStoredPoints(&wayPoints);

}



void PathExecutor::tick()
{
    switch (state) {
    case DrivingState::TO_TARGET:
    {
        bool done =te.tick();
        if(done) {
            //if(wayPoints.at(currentWaypointIndex).trajectory.size()== currentIndexInTrajectory){
            Position2D * nextTrajPoint = curWp->getNextPointOfTrajectory();
            if(nextTrajPoint==0){
                curWp->resetTrjectory();// for next use in loop
                // reached waypoint, check if we need to wait here

               if(wayPoints.at(currentWaypointIndex).orientToYaw){startOrientToYaw(); return;}
               if(wayPoints.at(currentWaypointIndex).triggerOutputPin){
                   GpioControl gpioControl;
                   gpioControl.start();// starts gpio sequence in other thread
               }
               if(wayPoints.at(currentWaypointIndex).dwellTimeSec>0.001){
                    startDwell(wayPoints.at(currentWaypointIndex).dwellTimeSec);//read dwell time from Waypoint?
                    return;
                }// immediately move on to the next waypoint

                nextTrajPoint = switchToNextWaypoint();
            }
            te.setTarget(*nextTrajPoint);//get next waypoint

        }
    }
        break;
    case DrivingState::IDLE:{
        // check if waiting time is over
        double time = TrajectoryExecutor::getSystemTimeSec();
        if(time >= dwellTimeEnd){// start moving to the next waypoint
            
            Position2D * nextTrajPoint=  switchToNextWaypoint();
            te.setTarget(*nextTrajPoint);
            te.resume();
            return;
        }
    }
        break;
    case DrivingState::PAUSED:{}
        break;
    case DrivingState::ADJUSTING_DIR:{
        bool done =te.adjustDirectionStepPid();
        if(done) {
            if(wayPoints.at(currentWaypointIndex).triggerOutputPin){
                GpioControl gpioControl;
                gpioControl.start();// starts gpio sequence in other thread after class specified delay (1sec)
            }

            if(wayPoints.at(currentWaypointIndex).dwellTimeSec>0.001){
                startDwell(wayPoints.at(currentWaypointIndex).dwellTimeSec);//read dwell time from Waypoint?
                return;
            }
   Position2D * nextTrajPoint=  switchToNextWaypoint();// move to next point
            te.setTarget(*nextTrajPoint);
            te.resume();
        }
    }
        break;
    default:
        break;
    }

}

void PathExecutor::enterPausedState(){

    previousState = state;
    state = DrivingState::PAUSED;
    te.pause();
}

void PathExecutor::resumeFromPause(){
    if(!hasStarted){// first pres of resume btn will start path exec
        bool res = startPath();
        if(!res){std::cout<<"[PE] could not start path"<<std::endl;
            return;}
        else{std::cout<<"[PE] started path"<<std::endl;}

    }
    state = previousState;
    te.resume();
}
void PathExecutor::setTarget(Position2D t)//not used?
{
    //this target is point wo direction - no need to calculate trajectory, just turn to direction and drive stright
    te.setTarget(t);
    std::cout<<"[PE] setting target "<<t.x<<" "<<t.y<<std::endl;
}

void PathExecutor::startDwell(double timeSec)
{
    te.pause();
    dwellTimeEnd = timeSec + TrajectoryExecutor::getSystemTimeSec();
    state = DrivingState::IDLE;
    std::cout<<"[PE] started dwell"<<std::endl;

}
void PathExecutor::startOrientToYaw()
{
    te.pause();
    state = DrivingState::ADJUSTING_DIR;
    std::cout<<"[PE] started Adjusting Direction"<<std::endl;
}
Position2D* PathExecutor::switchToNextWaypoint()
{
    std::cout<<"[PE] switch to next waypoint"<<std::endl;

    if(wayPoints.size()<1) {
        std::cout<<"waypoints size = 0 "<<std::endl;
        return 0;
    }
    currentWaypointIndex++;
    if(currentWaypointIndex >= wayPoints.size()) currentWaypointIndex =0; //todo if there is only one wp, then dont loop
    curWp = & wayPoints.at(currentWaypointIndex);
    //check if there is multiple points, i.e trajectory, or only single target
    Position2D*  nextTrajPoint = curWp->getNextPointOfTrajectory();
    if(nextTrajPoint==0){std::cout<<"[PE] next traj poin is null"<<std::endl;}// shold not be null because it is new waypoint, that should contain at least one point
    state = DrivingState::TO_TARGET;
    return nextTrajPoint;
}
bool PathExecutor::startPath()
{
    if(wayPoints.size()<1) {
        std::cout<<"waypoints size = 0 "<<std::endl;
        return 0;
    }
    currentWaypointIndex =0;
    if(currentWaypointIndex >= wayPoints.size()) currentWaypointIndex =0; //todo if there is only one wp, then dont loop
    curWp = & wayPoints.at(currentWaypointIndex);
    //check if there is multiple points, i.e trajectory, or only single target
    Position2D*  nextTrajPoint = curWp->getNextPointOfTrajectory();
    if(nextTrajPoint==0){}// shold not be null because it is new waypoint, that should contain at least one point
    state = DrivingState::TO_TARGET;
    hasStarted = true;
    te.setTarget(*nextTrajPoint);
    return true;
}
