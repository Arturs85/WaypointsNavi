#include "stepresponsereader.hpp"
#include <thread>
#include "control.hpp"
#include "motorcontrol.h"

void StepResponseReader::tick(){
    while(isRunning){
        Control::pathExecutor.te.motorControl->setWheelSpeedsFromAngVel(0,2.4);
        double time = TrajectoryExecutor::getSystemTimeSec();
        double angVel = Control::particleFilter.lastGyroAngVelRad;
        // write to file
        std::stringstream ss;
        ss<<(time-startTime)<<" "<<angVel<<std::endl;
        responseFile.writeString(ss);

        if(endTime<time)break;
        usleep(100000);
    }
    responseFile.closeFile();

}

void StepResponseReader::start(){
isRunning = true;   
 responseFile.openFileStepResponse();
    startTime = TrajectoryExecutor::getSystemTimeSec();
    endTime = startTime + recordLengthSec;
    std::thread t1(&StepResponseReader::tick,this); // passing 'this' by value
    t1.detach();
}

