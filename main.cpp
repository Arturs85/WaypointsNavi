
#include "udpcommunication.hpp"
#include "uiudp.hpp"
#include "control.hpp"
#include <iostream>
#include "uarttest.hpp"
#include <limits>
std::string TAG = "[main] ";

int main(){
try{
    ReachLLHmsg m =ReachLLHmsg::parseString("2021/09/13 14:52:56.600   56.951945641   24.078461689    27.6636   1  21   0.0100   0.0100   0.0100   0.0000   0.0000   0.0000   1.60    0.0");
    std::cout<<m.lat<<" "<<m.lon<<" "<<m.sdn_m;

    }catch(std::invalid_argument){std::cout<<"error while parsing";}

    exit(0);
    UdpCommunication::startReceivingThread();
UiUdp::startReceivingThread();
    Control control;
    UiUdp::uiParser.control = &control;

    control.control();//starts control cycle


}
