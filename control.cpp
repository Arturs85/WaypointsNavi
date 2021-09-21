#include "control.hpp"
#include "uiudp.hpp"
#include <iostream>
#include "udpcommunication.hpp"


ParticleFilter Control::particleFilter = ParticleFilter();
UartTest Control::uartTest;
  GyroReader Control::gyroReader;
void Control::control()
{
 uartTest.initialize();
    uartTest.startReceiveing();//starts receiving and sending threads

gyroReader.startReadingThread(); 
    while (true) {
        usleep(50000);

        switch (state) {
        case States::INIT_PLATFORM:{
            if(UdpCommunication::platformMsgparser.testCommunication()){
                std::cout<<TAG<<"received reply from platform"<<std::endl;
                UiUdp::uiParser.sendText("communication with platform ok ");

                state = INIT_GPS;
            }

        }

            break;
        default:
            break;
        }
        if(UdpCommunication::platformMsgparser.replies.size()>0){
            PlatformMsg m = UdpCommunication::platformMsgparser.replies.at(0);
            std::cout<<"ID: "<<m.id<<" type: "<<m.type<<" val0: "<<m.values.at(0)<<std::endl;
            UdpCommunication::platformMsgparser.replies.erase(UdpCommunication::platformMsgparser.replies.begin());


        }

    }
}
