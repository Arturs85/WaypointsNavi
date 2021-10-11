#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include "odometry.h"
#include "trajectoryexecutor.hpp"
#include <vector>
#include <random>
class Particle{
public:
    Particle(double x,double y, double yaw);
    Particle(){}
    double x;
    double y;
    double direction;
    double fitness;
    bool isValid = true;

    void addToDirectionAndNormalize(double dYaw){
        direction+= dYaw;
        direction = std::remainder(direction,2*M_PI);

    }
    void moveForward(double dist){ //x= long, y = lat
        x+=dist*std::cos(direction);
        y+=dist*std::sin(direction);
    }
    bool operator < (const Particle& other) const
    {
        return (fitness < other.fitness);
    }

};
class GpsDriftCounter{
    static double constexpr timeWindow = 3;//seconds between positions to compare
    double lat;
    double lon;
    double timeOfPosition;
public:
    double lastDriftM =10;
    void onGps(double lat, double lon){
        double time = TrajectoryExecutor::getSystemTimeSec();
        if(time-timeOfPosition> timeWindow){
            lastDriftM = (std::abs(lat-this->lat)+std::abs(lon-this->lon))* 111194.926644559;
            this->lat = lat;
            this->lon = lon;
            timeOfPosition = time;
        }

    };

};

class ParticleFilter{
public:
    GpsDriftCounter gpsDriftCounter;
    ParticleFilter();
    double lastGpsSdnM = 100;//initialization with a large value
    void initializeParticles(double x, double y);
    static const int PARTICLE_COUNT = 100;
    static const int GPS_DIST_ERR = 1;
    static const int radiOfEarth = 6371000;//m
    static constexpr double radiOfEarthForDegr = 111194.926644559;//m
    int notValidCount =0;

    double deltaYaw=0;
    std::normal_distribution<double> yawSpeedDtribution;
    std::normal_distribution<double> linMovementDistribution; //mean = 1m/s
    std::normal_distribution<double> regenSpatialDist; //for gps degrees

    std::default_random_engine generator;
    //GuiWindow guiWindow;
    void onGps(double x, double y);
    void onGyro(double angSpeedZDeg, double dt);
    void turnParticles(double angSp, double dt);
    Particle avgParticle;
    void onGpsWoOdo(double lat, double lon,double sdn_m);
    void calcFitnessFromYaw(double yawGPS);
    void addLinearMovementNoise(double dt);
    void onOdometry(double dt);
    void onGps(double lat, double lon, double sdn_m);
    Position2DGPS previousGPSPos;

    void calcFitness(double xGps, double yGps, double gpsErr);
    void addRegenNoise();
    void reduceUnequality(double coef,double maxWeigth);
protected:
    void onOdometry(Position2D position, Position2D deltaPosition);
private:
    std::vector<Particle> particles;
    void moveParticles(double dx, double dy, double dyaw);
    void calcFitness(double xGps, double yGps);
    void regenerateParticles();
    void addMovementNoise();
    Particle calcAverageParticle();
};

#endif // PARTICLEFILTER_H
