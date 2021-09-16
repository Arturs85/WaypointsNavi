#ifndef GYROREADER_HPP
#define GYROREADER_HPP

#include <thread>

#define MAXLINE 1024

class GyroDriftCounter {
    double startTime = 0;
    double endTime = 0;
    double gyroSum = 0;
    double previousRotSpeed =0;
    static const int MIN_SAMPLE_COUNT =10;
    static constexpr double MOVEMENT_TRESHOLD_GYROZ = 0.15;//degrees/sec NOTE this value is sample rate dependent, therfore it should be revised if sample rate is changed
public:
    int sampleCount = 0;
    double driftRadSec = 0;

    double driftForPublishing = std::numeric_limits<double>::max();

    bool getGyroDriftZ(double*val){
        if(driftForPublishing >=std::numeric_limits<double>::max())return false;
        *val = driftForPublishing;
        return true;
    }

    void reset() {
        startTime = 0;
        endTime = 0;
        gyroSum = 0;
        sampleCount =0;
    }
    void onGyroEvent(double dt, double rotationSpeed) {//rad/s
        if(std::abs(previousRotSpeed - rotationSpeed)>MOVEMENT_TRESHOLD_GYROZ )
            reset();

        gyroSum += rotationSpeed*dt;
        endTime += dt;
        driftRadSec = gyroSum / endTime;
        if(sampleCount >MIN_SAMPLE_COUNT) driftForPublishing=driftRadSec;
        sampleCount++;
        previousRotSpeed = rotationSpeed;
    }
};

class GyroReader{
public:

    GyroDriftCounter gdc;

    double timePreviousSec = 0;
    double directionZ = 0;

    int fd;
    static const std::string TAG;

    void readGyro();
    double getSystemTimeSec();
    short read_raw_data(int addr);
    void MPU6050_Init();
    void startReadingThread();
};

#endif // GYROREADER_HPP
