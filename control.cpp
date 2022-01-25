#include "control.hpp"
#include "uiudp.hpp"
#include <iostream>
#include "udpcommunication.hpp"
#include "logfilesaver.hpp"

ParticleFilter Control::particleFilter = ParticleFilter();
UartTest Control::uartTest;
UartUltra Control::uartUltra;
GyroReader Control::gyroReader;
PathExecutor Control::pathExecutor;

void Control::control()
{
    uartTest.initialize();
    uartTest.startReceiveing();//starts receiving and sending threads
    uartUltra.initialize();
    uartUltra.startReceiveing();
    motorControl = pathExecutor.te.motorControl;

    gyroReader.startReadingThread();
    int counterTest =0;
    int msgCount =0;
    while (true) {
        counterTest++;
        if(counterTest%30==0){GpioControl gc; gc.start();}
        usleep(100000);
        //obstacle detection
        double time = TrajectoryExecutor::getSystemTimeSec();
        double dt = time- uartUltra.distances.timeSec;
        if(dt<2)
            std::cout<< "obst detected: "<<uartUltra.distances.hasObstacle()<<std::endl;

        switch (state) {
        case States::INIT_PLATFORM:{
            if(UdpCommunication::platformMsgparser.testCommunication()){
                std::cout<<TAG<<"received reply from platform"<<std::endl;
                UiUdp::uiParser.sendText("communication with platform ok ");

                state = States::INIT_GPS;
            }

        }

            break;
        case States::INIT_GPS:{
            //  state = States::INIT_GYRO; // skip gps for testing
            if(particleFilter.lastGpsSdnM<0.5 && particleFilter.gpsDriftCounter.lastDriftM < 0.2 ){

                particleFilter.initializeParticles(particleFilter.previousGPSPos.lon,particleFilter.previousGPSPos.lat);// reinitialize pf with good gps cord

                std::cout<<TAG<<"GPS last sdn is ok (less than 0.5 m) ang gps drift is less than 0.2 m"<<std::endl;
                UiUdp::uiParser.sendText("GPS last sdn is ok (less than 0.3 m) and gpsdrift<0.1 ");
                state = States::IDLE;
                //reset pf log file
                LogFileSaver::logfilesaver.openFile();

            }
        }
            break;
        case States::INIT_GYRO:{
            double drift;
            if(gyroReader.gdc.getGyroDriftZ(&drift)){
                std::cout<<TAG<<"Initial Gyro drift calib done"<<std::endl;
                UiUdp::uiParser.sendText("Initial Gyro drift calib done");

                state = States::INIT_GPS;
            }
        }
            break;
        case States::IDLE:{

            //do nothing
        }
            break;
        case States::AUTO:{

            pathExecutor.tick();
        }
            break;
        default:
            break;
        case States::MANUAL:{

            // uiParser will send motor control msgs from ui to platform, if this state is active
        }
            break;
        case States::STEP_RESPONSE:{

        }
            break;
        }
        if(UdpCommunication::platformMsgparser.replies.size()>0){
            PlatformMsg m = UdpCommunication::platformMsgparser.replies.at(0);
            std::cout<<"ID: "<<m.id<<" type: "<<m.type<<" val0: "<<m.values.at(0)<<std::endl;
            UdpCommunication::platformMsgparser.replies.erase(UdpCommunication::platformMsgparser.replies.begin());


        }
        UiUdp::uiParser.sendState(state);// todo send less frequently?
    }
}

bool Control::enterAutoMode()//check if conditions are met, switch state
{
    if(state!= States::IDLE && state!= States::MANUAL) return false;
    state = States::AUTO;
    UiUdp::uiParser.sendText("switched to AUTO by UI request");
return true;
}
bool Control::enterManualMode()//check if conditions are met, switch state
{
    if(state!= States::IDLE && state!=States::AUTO && state!=States::STEP_RESPONSE) return false;
    state = States::MANUAL;
    UiUdp::uiParser.sendText("switched to MANUAL by UI request");
return true;
}
