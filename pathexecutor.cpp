#include "pathexecutor.hpp"

PathExecutor::PathExecutor()
{
//    wayPoints.push_back(Waypoint(Position2D(25.767017,56.649878,0)));
//    wayPoints.push_back(Waypoint(Position2D(25.767136,56.649861,0)));
//    wayPoints.push_back(Waypoint(Position2D(25.767153,56.649918,0)));
//    wayPoints.push_back(Waypoint(Position2D(25.767014,56.649934,0)));

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
                // reached waypoint, check if we need to wait here
                if(wayPoints.at(currentWaypointIndex).dwell){
                    startDwell(10);// read dwell time from Waypoint?
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
            switchToNextWaypoint();
            return;
        }
    }
        break;
    default:
        break;
    }

}

void PathExecutor::setTarget(Position2D t)
{
    //this target is point wo direction - no need to calculate trajectory, just turn to direction and drive stright
    te.setTarget(t);
}

void PathExecutor::startDwell(double timeSec)
{
    dwellTimeEnd = timeSec + TrajectoryExecutor::getSystemTimeSec();
    state = DrivingState::IDLE;
}

Position2D* PathExecutor::switchToNextWaypoint()
{
    currentWaypointIndex++;
    if(currentWaypointIndex >= wayPoints.size()) currentWaypointIndex =0; //todo if there is only one wp, then dont loop
    curWp = & wayPoints.at(currentWaypointIndex);
    //check if there is multiple points, i.e trajectory, or only single target
    Position2D*  nextTrajPoint = curWp->getNextPointOfTrajectory();
    if(nextTrajPoint==0){}// shold not be null because it is new waypoint, that should contain at least one point
    state = DrivingState::TO_TARGET;
    return nextTrajPoint;
}

