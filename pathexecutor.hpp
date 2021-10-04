#ifndef PATHEXECUTOR_HPP
#define PATHEXECUTOR_HPP
#include "trajectoryexecutor.hpp"
#include <vector>

enum class DrivingState{DEPARTING,STRIGHT,ARRIVAL,IDLE,TO_TARGET};

class Waypoint{

    std::vector<Position2D> trajectory;

    size_t curIndexInTrajectory =0;
public:
    Waypoint(Position2D end, bool dwell = false){trajectory.push_back(end); this->dwell = dwell;}// trajectory with endpoint only

    Position2D* getNextPointOfTrajectory(){
        if(trajectory.size()<=curIndexInTrajectory) return 0;
        return &(trajectory.at(curIndexInTrajectory++));
    }
    bool dwell = false;

};

class PathExecutor
{
public:
    PathExecutor();
    void tick();
private:
    Waypoint* curWp=0;
    void setTarget(Position2D t);
    void startDwell(double timeSec);
    Position2D *switchToNextWaypoint();
    std::size_t currentWaypointIndex=0;
    TrajectoryExecutor te;
    std::vector<Waypoint> wayPoints;
    double dwellTimeEnd =0;
    DrivingState state = DrivingState::IDLE;
};

#endif // PATHEXECUTOR_HPP
