#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include "odometry.h"
#include "trajectoryexecutor.h"
#include <vector>
class Particle{
public:
    Particle(double x,double y, double yaw);
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

class ParticleFilter: public OdometryListener, public GpsListener
{
public:
    ParticleFilter(Odometry *odometry);

    void initializeParticles(int x, int y);
    static const int PARTICLE_COUNT = 100;
    static const int GPS_DIST_ERR = 1;
//GuiWindow guiWindow;
    void onGps(double x, double y);
protected:
    void onOdometry(Position2D position, Position2D deltaPosition);
private:
    std::vector<Particle> particles;
    void moveParticles(double dx, double dy, double dyaw);
void calcFitness(double xGps, double yGps);
void regenerateParticles();
void addMovmentNoise();
Particle calcAverageParticle();
};

#endif // PARTICLEFILTER_H
