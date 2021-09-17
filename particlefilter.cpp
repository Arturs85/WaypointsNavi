#include "particlefilter.h"
#include "odometry.h"
#include <iostream>
#include <algorithm>
#include <random>

ParticleFilter::ParticleFilter()
{
    initializeParticles(0,0);

}

void ParticleFilter::onOdometry(Position2D position, Position2D deltaPosition){
    //  std::cout<<"particleFilter onOdometry called "<<position.x<<std::endl;
    moveParticles(deltaPosition.x,deltaPosition.y,deltaPosition.yaw);
    addMovementNoise();

}

void ParticleFilter::onGyro(double angSpeedZDeg, double dt){
    //  std::cout<<"particleFilter onGyro called "<<x<<" "<<y<<std::endl;
    turnParticles(angSpeedZDeg,dt);
    Particle avg =calcAverageParticle();

}

void ParticleFilter::onGps(double x, double y){
    //  std::cout<<"particleFilter onGps called "<<x<<" "<<y<<std::endl;
    calcFitness(x,y);
    regenerateParticles();
    Particle avg =calcAverageParticle();

}
void ParticleFilter::turnParticles(double angSp, double dt ){
    std::normal_distribution<double> yawSpeedDtribution(0.0,0.2);// stddev value?
    std::default_random_engine generator;

    for (int i = 0; i < particles.size(); i++) {
        double errYawSpDeg = (yawSpeedDtribution(generator));

        particles.at(i).addToDirectionAndNormalize((errYawSpDeg+angSp)*dt*M_PI/180);

    }
}



void ParticleFilter::moveParticles(double dx, double dy, double dyaw)
{
    for (int i = 0; i < particles.size(); i++) {

        double dist =  std::sqrt(dx*dx+dy*dy);
        double travelAngle = particles.at(i).direction+dyaw/2;


        double fi = atan2(dy,dx);
        //double dxg = dx*cos(fi+particles.at(i).direction)/cos(fi);
        //double dyg = dy*sin(fi+particles.at(i).direction)/sin(fi);
        double dxg = dist*cos(travelAngle);
        double dyg = dist*sin(travelAngle);

        particles.at(i).x+=dxg;
        particles.at(i).y+=dyg;
        particles.at(i).addToDirectionAndNormalize(dyaw);
    }
}

void ParticleFilter::calcFitness(double xGps, double yGps)
{
    double distanceSum =0;
    double longestDistance =0;
    for (int i = 0; i < particles.size(); i++) {
        Particle* p = & (particles.at(i));

        p->fitness= sqrt((xGps-p->x)*(xGps-p->x)+(yGps-p->y)*(yGps-p->y));//calc distance
        distanceSum += p->fitness;//store largest distance
        if(p->fitness>longestDistance) longestDistance = p->fitness;
        //if(p->fitness>GPS_DIST_ERR)p->isValid=false;
    }
    double avgDist = distanceSum/particles.size();
    for (int i = 0; i < particles.size(); i++) {
        particles.at(i).fitness = avgDist/particles.at(i).fitness;
    }
    //todo calc amount of fitness for one descendant, iterate trough particles, if fitnes > min add new particle, decrease fit by amount

}

void ParticleFilter::regenerateParticles()
{
    std::sort(particles.begin(), particles.end());// sort by fitness, at this point fitness should point, how much descendants particle should have
    std::vector<Particle> particlesRegenerated;
    int parentCount =0;
    for (int i = particles.size()-1; i >= 0; i--) {
        int descendantCount =   ((int)(particles.at(i).fitness))+1;// round up
        parentCount ++;
        //create descendants
        for (int j = 0; j < descendantCount; ++j) {
            particlesRegenerated.push_back(particles.at(i));//todo distribute descendants spatially
            if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

        }
        if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

    }
    std::cout<<"nr of parents "<<parentCount<<" max descendants count: "<<(((particles.at(particles.size()-1).fitness))+1)<<std::endl;

    particles = particlesRegenerated;// should we copy data  or adress only?
}

void ParticleFilter::addMovementNoise()
{
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.02);// stddev value?
    std::normal_distribution<double> angledistribution(0.0,M_PI/128);// stddev value?

    for (int i = 0; i < particles.size(); i++) {



        double errx = (distribution(generator));
        double erry = (distribution(generator));
        double errYaw = (angledistribution(generator));

        particles.at(i).x+=errx;//remove only, because wheel slip cant produce larger distance than measured by odo?
        particles.at(i).y+=erry;
        particles.at(i).addToDirectionAndNormalize(errYaw);

    }
}

Particle ParticleFilter::calcAverageParticle()
{
    Particle avg(0,0,0);
    //    avg.direction=0;
    //    avg.x=0;
    //    avg.y=0;
    double maxDeltaYawSoFar = 0;// find max dist of two consecutive particles, it should represent deviation TODO- improve to exact method

    for (int i = 0; i < particles.size(); i++) {
        avg.direction+=particles.at(i).direction;
        avg.x+=particles.at(i).x;
        avg.y+=particles.at(i).y;
        if(i>0){
            double dYaw =  std::abs(particles.at(i).direction-particles.at(i-1).direction);
            if(dYaw > maxDeltaYawSoFar)maxDeltaYawSoFar = dYaw;
        }
    }
    avg.direction =  avg.direction/particles.size();
    avg.x =  avg.x/particles.size();
    avg.y =  avg.y/particles.size();
    deltaYaw = maxDeltaYawSoFar;
    return avg;
}



void ParticleFilter::initializeParticles(int x, int y) {
    particles.clear();
    for (int i = 0; i < PARTICLE_COUNT; i++) {


        particles.push_back( Particle(x, y, (rand() % 360)*M_PI / 180 ));

    }

}


Particle::Particle(double x, double y, double yaw)
{
    this->x=x;
    this->y = y;
    this->direction = yaw;
}
