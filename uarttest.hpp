
#ifndef UARTTEST_H
#define UARTTEST_H

#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>	 	//Used for UART
#include <pthread.h>
#include <cstring>
#include <vector>
#include <stdint.h>
#include <string>
#include <sstream>
#include <iostream>
using namespace std;
struct ReachLLHmsg{double lat; double lon; double sdn_m;double sde_m;

                   static ReachLLHmsg parseString(std::string r){//throws std::invalid_argument
                       ReachLLHmsg msg;

                       std::stringstream ss(r);
                       std::string s;
                       int cnt=0;
                       while ( std::getline( ss, s,' ' ) ) {
                        //  if(s.compare(" ")==0)continue;//skip whitespaces
                           if(s.length()<1)continue;
                       //   std::cout<<cnt<<": "<<s<<std::endl;
                           switch (cnt) {
                           case 2://
                               msg.lat= std::stod(s);
                               break;
                           case 3:
                               msg.lon= std::stod(s);
                               break;
                           case 7:
                               msg.sdn_m= std::stod(s);
                               break;
                           case 8:
                               msg.sde_m= std::stod(s);
                               break;
                           default:
                               break;
                           }
                           cnt++;
                       }
                       return msg;
                   }
                  }; //https://community.emlid.com/t/reach-llh-protocol-format/1354/4
class UartTest
{
public:
    static int uart0_filestream;
    pthread_t receivingThreadUart;
    pthread_t sendingThreadUart;
    static pthread_mutex_t mutexSend;
    static pthread_mutex_t mutexReceive;

    static char tx_buffer[];
    static int tx_size;
    static vector<uint8_t> rxframe;
    UartTest();
    ~UartTest();
    void initialize();
    void send();
    void startReceiveing();
    void clearRxFrame();
    static void setDataToTransmit(char* data, int size);
    static void setDataToTransmit(vector<uint8_t> comm);
    vector<uint8_t> readNumberOfBytes(uint8_t noOfBytes );

    static void* receive(void* arg);
    static void* sendingLoop(void* arg);
    void waitUartThreadsEnd();

};






#endif //UARTTEST_H


