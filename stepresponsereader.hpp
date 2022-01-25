#ifndef STEPRESPONSEREADER_HPP
#define STEPRESPONSEREADER_HPP
#include "trajectoryexecutor.hpp"
#include <vector>
#include <cstddef> //for size_t on rpi

#include <unistd.h>

#include "logfilesaver.hpp"

class StepResponseReader{

    double recordLengthSec =5;
double startTime =0;
    double endTime =0;
    LogFileSaver responseFile;
   // MotorControl mc;
public:
    void start();
    void tick();
bool isRunning = false;

};



#endif // STEPRESPONSEREADER_HPP
