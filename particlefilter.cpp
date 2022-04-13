
#include "particlefilter.h"
#include "odometry.h"
#include <iostream>
#include <algorithm>
#include <iomanip>      // std::setprecision
#include "control.hpp"
#include "logfilesaver.hpp"
#include <sstream>
#include "uiudp.hpp"
#include <pthread.h>
pthread_mutex_t ParticleFilter::mutexParticles = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ParticleFilter::mutexGpsData = PTHREAD_MUTEX_INITIALIZER;


ParticleFilter::ParticleFilter()
{
    yawSpeedDtribution = std::normal_distribution<double>(0.0,10.8);// stddev value?
    linMovementDistribution = std::normal_distribution<double>(0.0,1.8);// stddev value?
    regenSpatialDist = std::normal_distribution<double>(0.0, 0.01/radiOfEarthForDegr); //0.1m stddev , use speed instead?

    initializeParticles(0,0);

}


void ParticleFilter::onOdometry(double leftWheelSpeed, double rightWheelSpeed){
    // std::cout<<" particles size: "<<particles.size()<<std::endl;
    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time- previousOdometryTime;


    if(dt>0.2) dt = 0.1; //to avoid unrealistic movements at initialisation and pause
    std::normal_distribution<double> wheelSpeedDtributionRight = std::normal_distribution<double>(rightWheelSpeed,rightWheelSpeed);// stddev value?
    std::normal_distribution<double> wheelSpeedDtributionLeft = std::normal_distribution<double>(leftWheelSpeed,leftWheelSpeed);// stddev value?
    previousOdometryTime = time;
    pthread_mutex_lock( &mutexParticles );
    if(saveParticlesToFile){

        std::stringstream ss;
        for (int i = 0; i < particles.size(); ++i) {
            ss<<particles.at(i).x<<" "<<particles.at(i).y<<" "<<particles.at(i).direction<<std::endl;
        }
        ss<<"eol"<<std::endl;
        LogFileSaver::logfilesaver.writeString(ss);
    }
    for (int i = 0; i < particles.size(); i++) {

        double travelRight = wheelSpeedDtributionRight(generator)*dt*Odometry::WHEEL_RADI;
        double travelLeft = wheelSpeedDtributionLeft(generator)*dt*Odometry::WHEEL_RADI;
        double travel = (travelRight+travelLeft)/2;

        double deltaYaw = (travelRight-travelLeft)/Odometry::WHEELS_TRACK;

        double dxg = travel*cos(particles.at(i).direction+deltaYaw/2);
        double dyg = travel*sin(particles.at(i).direction+deltaYaw/2);

        particles.at(i).x +=dxg;
        particles.at(i).y +=dyg;

        particles.at(i).addToDirectionAndNormalize(deltaYaw);
        particles.at(i).angVel = deltaYaw/dt;
        particles.at(i).linearVel = travel/dt;

    }
    pthread_mutex_unlock(&mutexParticles );


}
void ParticleFilter::onOdometryWGps(double leftWheelSpeed, double rightWheelSpeed){
    // std::cout<<" particles size: "<<particles.size()<<std::endl;
    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time- previousOdometryTime;
    previousOdometryTime = time;

    if(dt>0.2){ return;} //to avoid unrealistic movements at initialisation and pause
    std::normal_distribution<double> wheelSpeedDtributionRight = std::normal_distribution<double>(rightWheelSpeed,rightWheelSpeed);// stddev value?
    std::normal_distribution<double> wheelSpeedDtributionLeft = std::normal_distribution<double>(leftWheelSpeed,leftWheelSpeed);// stddev value?
    return; // not updating particles

    pthread_mutex_lock( &mutexParticles );
    if(saveParticlesToFile){

        std::stringstream ss;
        ss<<std::setprecision(9);
        for (int i = 0; i < particles.size(); ++i) {
            ss<<particles.at(i).x<<" "<<particles.at(i).y<<" "<<particles.at(i).direction<<std::endl;
        }
        ss<<"eol"<<std::endl;
        LogFileSaver::logfilesaver.writeString(ss);
    }
    for (int i = 0; i < particles.size(); i++) {

        double travelRight = wheelSpeedDtributionRight(generator)*dt*Odometry::WHEEL_RADI;
        double travelLeft = wheelSpeedDtributionLeft(generator)*dt*Odometry::WHEEL_RADI;
        double travel = (travelRight+travelLeft)/2;

        double deltaYaw = (travelRight-travelLeft)/Odometry::WHEELS_TRACK;

        double dxg = travel*cos(particles.at(i).direction+deltaYaw/2);
        double dyg = travel*sin(particles.at(i).direction+deltaYaw/2);

        particles.at(i).x +=dxg/radiOfEarthForDegr;
        particles.at(i).y +=dyg/radiOfEarthForDegr;

        particles.at(i).addToDirectionAndNormalize(deltaYaw);
        particles.at(i).angVel = deltaYaw/dt;
        particles.at(i).linearVel = travel/dt;
        //  particles.at(i).isValid = true; //reset validity, if onGps had cleared it, //todo reinitialize pf in such case?

    }
    pthread_mutex_unlock(&mutexParticles );

    //  previousOdometryTime = time;
}

void ParticleFilter::onOdometry( Position2D deltaPosition){
    //  std::cout<<"particleFilter onOdometry called "<<position.x<<std::endl;
    moveParticles(deltaPosition.x,deltaPosition.y,deltaPosition.yaw);
    //addMovementNoise();

    addLinearMovementNoise(0.1);// todo use actual time
}

void ParticleFilter::onOdometry(double dt){// for use wo actual odometry
    addLinearMovementNoise(dt);
}

void ParticleFilter::onGyro(double angSpeedZDeg, double dt){
    //  std::cout<<"particleFilter onGyro called "<<x<<" "<<y<<std::endl;
    pthread_mutex_lock( &mutexGpsData );

    dirComplRad = dirComplRad+angSpeedZDeg*M_PI/180*dt;
    dirComplRad = std::remainder(dirComplRad,2*M_PI);
    pthread_mutex_unlock( &mutexGpsData );

    //calc fitness of each particle depending on how well its angular vel from odometry is comparable to gyro angular speed
    lastGyroAngVelRad = angSpeedZDeg*M_PI/180;
    return; // not updating particles
    pthread_mutex_lock( &mutexParticles );

    if(particles.size()<1) return; // todo -do not return wo mutex unlock
    if(saveParticlesToFile){
        std::stringstream ss;
        ss<<std::setprecision(9);

        for (int i = 0; i < particles.size(); ++i) {
            ss<<particles.at(i).x<<" "<<particles.at(i).y<<" "<<particles.at(i).direction<<std::endl;
        }
        ss<<"eol"<<std::endl;
        LogFileSaver::logfilesaver.writeString(ss);
    }
    calcFitness(angSpeedZDeg*M_PI/180);
    regenerateParticlesAfterGyro();
    double time = TrajectoryExecutor::getSystemTimeSec();
    if(time - previousOdometryTime <0.2) {turnParticles(0,dt);}; //aplly angular noise  only if platform is being moved, to align direction of particles to dir from gps
    //    turnParticles(angSpeedZDeg,dt);
    Particle avg = calcAverageParticle();
    pthread_mutex_unlock( &mutexParticles );

    //  std::cout<<"avg particle "<<avgParticle.x<<" "<<avgParticle.y<<" dir: "<<avgParticle.direction*180/M_PI<<" angV: "<<avgParticle.angVel<<" angVelGyr: "<<lastGyroAngVelRad<<" linVel: "<<avgParticle.linearVel<<" parentsCnt: "<<lastParentsCount<<std::endl;
}

void ParticleFilter::updateGyro(double angSpeedZDeg)
{
    lastGyroAngVelRad = angSpeedZDeg*M_PI/180;

}
void ParticleFilter::onGpsWoOdo(double lat, double lon, double sdn_m){
    //  std::cout<<"particleFilter onGps called "<<x<<" "<<y<<std::endl;
    //calc angle err delta
    lastGpsSdnM = sdn_m; // used by supervisory control to know when gps is initialised
    double gpsErrM = 0.1; //temp
    Position2DGPS curPos(lat,lon,0);
    double yawGPS = previousGPSPos.calcYawPointToPoint(curPos);

    calcFitnessFromYaw(yawGPS);
    std::stringstream ss;
    if(saveParticlesToFile){
        for (int i = 0; i < particles.size(); ++i) {
            ss<<particles.at(i).x<<" "<<particles.at(i).y<<" "<<particles.at(i).direction<<std::endl;

        }
        ss<<"eol"<<std::endl;
        LogFileSaver::logfilesaver.writeString(ss);
    }
    regenerateParticles();

    Particle avg = calcAverageParticle();

    std::cout<<"[pf] avgDir: "<<avg.direction*180/M_PI<<" dYPf: "<<deltaYaw*180/M_PI<<" gpsDir: "<<yawGPS*180/M_PI<<" "<<std::setprecision(9)<<lon<<" "<<lat<<" "<<std::setprecision(4)<<" sdn_m "<<sdn_m <<" gyroInt "<<Control::gyroReader.directionZ<<" angV: "<<avgParticle.angVel<<" angVelGyr: "<<lastGyroAngVelRad<<" linVel: "<<avgParticle.linearVel<<std::endl;
    previousGPSPos.lat = lat;
    previousGPSPos.lon = lon;
}
double ParticleFilter::getGpsAgeSec(){
    double time = TrajectoryExecutor::getSystemTimeSec();
    pthread_mutex_lock( &mutexGpsData );
    time = time-previousGpsTime;
    pthread_mutex_unlock( &mutexGpsData );
    return time;
}
void ParticleFilter::onGps(double lat, double lon, double sdn_m,double sde_m){
    //  std::cout<<"particleFilter onGps called "<<x<<" "<<y<<std::endl;
    //calc angle err delta

    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousGpsTime;

    gpsDriftCounter.onGps(lat,lon);
    //if(gpsDriftCounter.lastDriftM<0.3)//todo value
    lastGpsSdnM = sdn_m; // used by supervisory control to know when gps is initialised
    lastGpsSdeM = sde_m; // used by supervisory control to know when gps is initialised
    Position2DGPS curPos(lat,lon,0);
    double yawGPS = previousGPSPos.calcYawPointToPoint(curPos);
    double distGps = previousGPSPos.distanceMeters(curPos);


    pthread_mutex_lock( &mutexGpsData );
    previousGpsTime = time;
    previousGPSPos.lat = lat;
    previousGPSPos.lon = lon;
    previousGPSPos.yaw = yawGPS;
    linVelGpsLpf = distGps/dt*(1-linVelLpfWeigth)+linVelGpsLpf*linVelLpfWeigth;//todo hardcoded time 0.2 sec, change to actual time

    if(linVelGpsLpf>minLinVelForGpsDir){ //use lin vel from gps only if platform is moving forward with noticable speed

        // complementary filter
        double sina = std::sin(dirComplRad );
        double sinb = std::sin(previousGPSPos.yaw);
        double cosa = std::cos(dirComplRad );
        double cosb = std::cos(previousGPSPos.yaw);
        double localGyroWeight = gyroWeigth-startupGpsFactor;// makes more weight on gps dir at startup to faster narrow in on usable direction

        dirComplRad = std::atan2(localGyroWeight*sina+(1-localGyroWeight)*sinb,localGyroWeight*cosa+(1-localGyroWeight)*cosb);
        dirComplRad = std::remainder(dirComplRad,2*M_PI);
        startupGpsFactor*= 0.9; //decay rate

    }

    pthread_mutex_unlock( &mutexGpsData );

    if(time - previousOdometryTime > 0.2) return; //aplly gps only if platform is being moved
    return; // not updating particles
    pthread_mutex_lock( &mutexParticles );

    calcFitness(lon,lat,sdn_m);
    if(saveParticlesToFile){

        std::stringstream ss;
        ss<<std::setprecision(9);
        for (int i = 0; i < particles.size(); ++i) {
            ss<<particles.at(i).x<<" "<<particles.at(i).y<<" "<<particles.at(i).direction<<" "<<Control::gyroReader.directionZ<<std::endl;

        }
        ss<<"eol"<<std::endl;
        ss<<lon<<" "<<lat<<" "<<sdn_m<<" "<<sde_m<<std::endl;
        ss<<"eol"<<std::endl;
        LogFileSaver::logfilesaver.writeString(ss);

    }
    regenerateParticles();
    addRegenNoise();// to compensate for positive linear movment noise, regen noise distributes p in all directions
    Particle avg = calcAverageParticle();
    pthread_mutex_unlock( &mutexParticles );

    //    std::cout<<"[pf] avgDir: "<<avg.direction*180/M_PI<<" dYPf: "<<deltaYaw<<" gpsDir: "<<yawGPS*180/M_PI<<" "<<std::setprecision(10)<<lon<<" "<<lat<<" pf: "<<avgParticle.x<<" "<<avgParticle.y<<std::setprecision(4)<<" sdn,e_m "<<sdn_m <<" "<<sde_m<< " gpsdDrift: "<<gpsDriftCounter.lastDriftM<<" RIC: "<<gpsLostReinitCounter <<" gyroInt "<<Control::gyroReader.directionZ<<std::endl;
    std::cout<<"[pf] avgDir: "<<avg.direction*180/M_PI<<" complDir: "<<dirComplRad*180/M_PI<<" gpsDir: "<<yawGPS*180/M_PI<<" "<<std::setprecision(10)<<lon<<" "<<lat<<" pf: "<<avgParticle.x<<" "<<avgParticle.y<<std::setprecision(4)<<" sdn,e_m "<<sdn_m <<" "<<sde_m<< " linVelLpf: "<<linVelGpsLpf<<" angV: "<<avgParticle.angVel<<" angVelGyr: "<<lastGyroAngVelRad<<" linVel: "<<avgParticle.linearVel<<" RIC: "<<gpsLostReinitCounter <<" gyroInt "<<Control::gyroReader.directionZ<<std::endl;

    //UiUdp::uiParser.sendDeltYaw(deltaYaw);
}
void ParticleFilter::onGps(double x, double y){
    //  std::cout<<"particleFilter onGps called "<<x<<" "<<y<<std::endl;
    calcFitness(x,y);
    regenerateParticles();
    Particle avg =calcAverageParticle();

}
void ParticleFilter::turnParticles(double angSp, double dt ){
    

    for (int i = 0; i < particles.size(); i++) {
        double errYawSpDeg = (yawSpeedDtribution(generator));
        //std::cout<<errYawSpDeg<<" " ;
        particles.at(i).addToDirectionAndNormalize((errYawSpDeg+angSp)*dt*M_PI/180);

    }
    //std::cout<<std::endl;
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
void ParticleFilter::calcFitness(double xGps, double yGps, double gpsErr)
{
    //   gpsErr /=ParticleFilter::radiOfEarthForDegr;// convert to gps degrees
    if(gpsErr<0.1)gpsErr =0.1;// dont use cery small gps error values for it can lead to 0 regenerated particles, because no particles may be at such a small distance after onGyro
    gpsErr =gpsErr*2/ParticleFilter::radiOfEarthForDegr;// convert to gps degrees
    double distanceSum =0;
    double longestDistance =0;
    int validCount =0;
    for (int i = 0; i < particles.size(); i++) {
        Particle* p = & (particles.at(i));

        p->fitness= sqrt((xGps-p->x)*(xGps-p->x)+(yGps-p->y)*(yGps-p->y));//calc distance
        if(p->fitness>gpsErr)p->isValid=false;
        else{
            validCount++;
            distanceSum += p->fitness;//store largest distance
            if(p->fitness>longestDistance) longestDistance = p->fitness;
        }
    }
    //    reduceUnequality(0.1,longestDistance);// reduce difference between weigths to include more particles in regen

    distanceSum =0;
    for (int i = 0; i < particles.size(); i++) {
        if(particles.at(i).isValid)
            distanceSum+=longestDistance - particles.at(i).fitness;
    }

    //  double avgDist = distanceSum/validCount;//todo calc amount of desc
    for (int i = 0; i < particles.size(); i++) {
        particles.at(i).fitness = 4*PARTICLE_COUNT*(longestDistance-particles.at(i).fitness)/distanceSum;
    }
    //todo calc amount of fitness for one descendant, iterate trough particles, if fitnes > min add new particle, decrease fit by amount
    notValidCount = particles.size()-validCount;
}

void ParticleFilter::calcFitness(double angVel)
{
    double distanceSum =0;
    double longestDistance =0;

    for (int i = 0; i < particles.size(); i++) {
        Particle* p = & (particles.at(i));

        p->fitness= std::abs(p->angVel-angVel);//calc distance

        distanceSum += p->fitness;//store largest distance
        if(p->fitness>longestDistance) longestDistance = p->fitness;

    }
    //    reduceUnequality(0.1,longestDistance);// reduce difference between weigths to include more particles in regen

    distanceSum =0;
    for (int i = 0; i < particles.size(); i++) {

        distanceSum+=longestDistance - particles.at(i).fitness;
    }
    //std::cout<<"ld "<<longestDistance<<" ds "<<distanceSum<<std::endl;

    //  double avgDist = distanceSum/validCount;//todo calc amount of desc
    for (int i = 0; i < particles.size(); i++) {
        particles.at(i).fitness = 8*PARTICLE_COUNT*(longestDistance-particles.at(i).fitness)/distanceSum;
        //    std::cout<<" "<<particles.at(i).fitness;

    }
    //todo calc amount of fitness for one descendant, iterate trough particles, if fitnes > min add new particle, decrease fit by amount

}
void ParticleFilter::calcFitnessFromYaw(double yawGPS)
{
    double distanceSum =0;
    double longestDistance =0;
    for (int i = 0; i < particles.size(); i++) {
        Particle* p = & (particles.at(i));

        p->fitness= std::abs(yawGPS-p->direction);
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
        if(!particles.at(i).isValid){particles.at(i).isValid=true;continue;}
        int descendantCount =   (int)(particles.at(i).fitness+0.5);// round up
        parentCount ++;
        //create descendants
        for (int j = 0; j < descendantCount; ++j) {
            particlesRegenerated.push_back(particles.at(i));//todo distribute descendants spatially
            if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

        }
        if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

    }
    //    std::cout<<" particles.size: "<<particles.size()<<std::endl;
    //  std::cout<<"nr of parents "<<parentCount<<" max descendants count: "<<(((particles.at(particles.size()-1).fitness))+0.5)<<" notValidCount: "<<notValidCount <<std::endl;
    if(particlesRegenerated.size()>0){

        particles = particlesRegenerated;}// should we copy data  or adress only?
    else{
        std::cout<<" wng: no particles regenerated, max fit: "<<((particles.at(particles.size()-1)).fitness+0.5)<<std::endl;
        //reinitialize particles with last good position and distribute widely
        initializeParticles(previousGPSPos.lon,previousGPSPos.lat,previousGPSPos.yaw);
        //addNoiseAfterOutOfGps();//todo right amount of noise and only apply when  gps error

    }
    lastParentsCount = parentCount;
}

void ParticleFilter::regenerateParticlesAfterGyro()// do not reinitialise particles if they cant be regenerated, leave old ones
{
    std::sort(particles.begin(), particles.end());// sort by fitness, at this point fitness should point, how much descendants particle should have
    std::vector<Particle> particlesRegenerated;
    int parentCount =0;
    for (int i = particles.size()-1; i >= 0; i--) {
        if(!particles.at(i).isValid){particles.at(i).isValid=true;continue;}
        int descendantCount =   (int)(particles.at(i).fitness+0.5);// round up
        parentCount ++;
        //create descendants
        for (int j = 0; j < descendantCount; ++j) {
            particlesRegenerated.push_back(particles.at(i));//todo distribute descendants spatially
            if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

        }
        if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

    }
    //    std::cout<<" particles.size: "<<particles.size()<<std::endl;
    //  std::cout<<"nr of parents "<<parentCount<<" max descendants count: "<<(((particles.at(particles.size()-1).fitness))+0.5)<<" notValidCount: "<<notValidCount <<std::endl;
    if(particlesRegenerated.size()>0){

        particles = particlesRegenerated;}// should we copy data  or adress only?
    else{// do not print message to reduce log entries
        //   std::cout<<" cant regen particles after onGyro, max fit: "<<((particles.at(particles.size()-1)).fitness+0.5)<<std::endl;

    }
    lastParentsCount = parentCount;
}


void ParticleFilter::addNoiseAfterOutOfGps()
{
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.25);// stddev value?
    std::normal_distribution<double> angledistribution(0.0,M_PI/4);// stddev value?

    for (int i = 0; i < particles.size(); i++) {

        double errx = (distribution(generator))/ParticleFilter::radiOfEarthForDegr;
        double erry = (distribution(generator))/ParticleFilter::radiOfEarthForDegr;

        double errYaw = (angledistribution(generator));

        particles.at(i).x+=errx;//remove only, because wheel slip cant produce larger distance than measured by odo?
        particles.at(i).y+=erry;
        particles.at(i).addToDirectionAndNormalize(errYaw);

    }
}
void ParticleFilter::addRegenNoise()
{
    for (int i = 0; i < particles.size(); i++) {

        double errx = (regenSpatialDist(generator));
        double erry = (regenSpatialDist(generator));


        particles.at(i).x+=errx;
        particles.at(i).y+=erry;

    }
}

void ParticleFilter::reduceUnequality(double coef, double maxWeigth)// c: 0...1 , large coef means eaqual weigths, small - unchanged
{
    for (int i = 0; i < particles.size(); i++) {

        particles.at(i).fitness = particles.at(i).fitness/(maxWeigth-(maxWeigth-particles.at(i).fitness)*coef);
    }
}

void ParticleFilter::addLinearMovementNoise(double dt)// for testing wo actual odometry from wheels - asuuume that linear speed can be from 0 to 1 m/s
{
    //  std::default_random_engine generator;
    // std::normal_distribution<double> distribution(1.0,1.0);// stddev value? //mean = 1m/s

    for (int i = 0; i < particles.size(); i++) {
        double dist = dt*(linMovementDistribution(generator));
        dist = dist/ParticleFilter::radiOfEarthForDegr; //converting from m to degrees, because lat, lon is degrees
        // particles.at(i).moveForward(std::abs(dist));
        particles.at(i).moveForward(dist);// testing w moving both directions

    }
}
Particle ParticleFilter::calcAverageParticle()
{
    Particle avg(0,0,0);
    //    avg.direction=0;
    //    avg.x=0;
    //    avg.y=0;
    double maxDeltaYawSoFar = 0;// find max dist of two consecutive particles, it should represent deviation TODO- improve to exact method
    double sina =0;// for cirlular mean
    double cosa =0;
    for (int i = 0; i < particles.size(); i++) {

        sina += std::sin(particles.at(i).direction);
        cosa += std::cos(particles.at(i).direction);

        avg.x+=particles.at(i).x;
        avg.y+=particles.at(i).y;
        avg.angVel += particles.at(i).angVel;
        avg.linearVel += particles.at(i).linearVel;
        if(i>0){
            double dYaw =  std::abs(particles.at(i).direction-particles.at(i-1).direction);
            if(dYaw > maxDeltaYawSoFar)maxDeltaYawSoFar = dYaw;

        }
    }
    sina /=particles.size();
    cosa /=particles.size();

    avg.direction =  std::atan2(sina,cosa);
    //avg.direction = std::remainder(avg.direction,2*M_PI);// converting to -pi..+pi
    avg.x =  avg.x/particles.size();
    avg.y =  avg.y/particles.size();
    avg.angVel /= particles.size();
    avg.linearVel /= particles.size();
    deltaYaw = maxDeltaYawSoFar*180/M_PI;
    avgParticle = avg;
    return avg;
}



void ParticleFilter::initializeParticles(double x, double y) {
    particles.clear();
    for (int i = 0; i < PARTICLE_COUNT; i++) {


        particles.push_back( Particle(x, y, (rand() % 360)*M_PI / 180 ));
        //   particles.push_back( Particle(x, y, 0));
    }
    addRegenNoise();// to move particles away from single point, as in this case fitness function will be zero
    previousGpsTime = TrajectoryExecutor::getSystemTimeSec();
}
void ParticleFilter::initializeParticles(double x, double y,double yaw) {//for reinitialisation after gps lost - using last known gps direction
    particles.clear();
    for (int i = 0; i < PARTICLE_COUNT; i++) {


        particles.push_back( Particle(x, y, yaw));
    }
    addNoiseAfterOutOfGps();
    gpsLostReinitCounter++;
}

void ParticleFilter::getGpsPosition(Position2DGPS &pos)
{
    pthread_mutex_lock( &mutexGpsData );
    pos = previousGPSPos;
    pthread_mutex_unlock( &mutexGpsData );
}

void ParticleFilter::getLinVelGpsLpf(double &vel)
{
    pthread_mutex_lock( &mutexGpsData );
    vel = linVelGpsLpf;
    pthread_mutex_unlock( &mutexGpsData );
}

Particle::Particle(double x, double y, double yaw)
{
    this->x=x;
    this->y = y;
    this->direction = yaw;
}
