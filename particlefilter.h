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

double addToDirectionAndNormalize(double dYaw){
    direction+= dYaw;
    direction = std::remainder(direction,2*M_PI);

}
    bool operator < (const Particle& other) const
        {
            return (fitness < other.fitness);
        }

};

class ParticleFilter{
public:
    ParticleFilter();

    void initializeParticles(int x, int y);
    static const int PARTICLE_COUNT = 100;
    static const int GPS_DIST_ERR = 1;
    double deltaYaw=0;
std::normal_distribution<double> yawSpeedDtribution;
    std::default_random_engine generator;
//GuiWindow guiWindow;
    void onGps(double x, double y);
    void onGyro(double angSpeedZDeg, double dt);
    void turnParticles(double angSp, double dt);
    Particle avgParticle;
    void onGpsWoOdo(double lat, double lon,double sdn_m);
    void calcFitnessFromYaw(double yawGPS);
protected:
    void onOdometry(Position2D position, Position2D deltaPosition);
private:
    std::vector<Particle> particles;
    void moveParticles(double dx, double dy, double dyaw);
void calcFitness(double xGps, double yGps);
void regenerateParticles();
void addMovementNoise();
Particle calcAverageParticle();
Position2DGPS previousGPSPos;
};

#endif // PARTICLEFILTER_H
