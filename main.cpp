
#include "udpcommunication.hpp"
#include "uiudp.hpp"
#include "control.hpp"
#include <iostream>
#include "uarttest.hpp"
#include <limits>
#include <cstdlib>
#include <signal.h>
#include "logfilesaver.hpp"
//#include "roombaController.hpp"
std::string TAG = "[main] ";

void my_handler(int s){
    LogFileSaver::logfilesaver.closeFile();
  //  Control::pathExecutor.te.motorControl->rc->shutDown();
//usleep(1000);

//Control::pathExecutor.te.motorControl->rc->shutDown();
    std::cout<<"Caught signal"<<  s<<std::endl;;

    exit(0);

}

int main(){
    //try{
    //  ReachLLHmsg m =ReachLLHmsg::parseString("2021/09/13 14:52:56.600   56.951945641   24.078461689    27.6636   1  21   0.0100   0.0100   0.0100   0.0000   0.0000   0.0000   1.60    0.0");
    //std::cout<<m.lat<<" "<<m.lon<<" "<<m.sdn_m;

    //}catch(std::invalid_argument){std::cout<<"error while parsing";}

    // exit(0);


    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    UdpCommunication::startReceivingThread();
    Control control;
    UiUdp::uiParser.control = &control;

    UiUdp::startReceivingThread();
    control.control();//starts control cycle


}
