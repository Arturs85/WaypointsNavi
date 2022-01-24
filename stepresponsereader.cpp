#include "stepresponsereader.hpp"
#include <thread>
#include "control.hpp"
#include "motorcontrol.h"

void StepResponseReader::tick(){
    while(1){
        Control::pathExecutor.te.motorControl->setWheelSpeedsFromAngVel(0,0.1);
        double time = TrajectoryExecutor::getSystemTimeSec();
        double angVel = Control::particleFilter.lastGyroAngVelRad;
        // write to file
        std::stringstream ss;
        ss<<time<<" "<<angVel<<std::endl;
        responseFile.writeString(ss);

        if(endTime>time)break;
        usleep(100000);
    }
    responseFile.closeFile();

}

void StepResponseReader::start(){
    responseFile.openFileStepResponse();
    endTime = TrajectoryExecutor::getSystemTimeSec()+recordLengthSec;
    std::thread t1(&StepResponseReader::tick,this); // passing 'this' by value
    t1.detach();
}
