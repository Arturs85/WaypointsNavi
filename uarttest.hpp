
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
struct ReachLLHmsgDate{std::string year; std::string month; std::string day;};
struct ReachLLHmsg{
    double lat; double lon; double sdn_m;double sde_m;

    static ReachLLHmsgDate parseDate(std::string dateString ){
        //example "2020/05/18"
        ReachLLHmsgDate msg;
        std::stringstream ss(dateString);
        std::string s;
        int cnt=0;

        while ( std::getline( ss, s,'/' ) ) {
            switch (cnt) {
            case 0:
                msg.year = s;
                break;
            case 1:
                msg.month = s;
                break;
            case 2:
                msg.day = s;
                break;
            default:
                break;
            }
        cnt++;
        }
            if(cnt!=3){ //date string must contain two '/' characters
                throw std::invalid_argument("date string not valid");}
            return msg;

    }
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
            case 0://
                parseDate(s);
                break;
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
    static pthread_mutex_t mutexReceive;
static double timeOfLastRead;
    static char tx_buffer[];
    static int tx_size;
    static bool isInitialisedUart;
    UartTest();
    ~UartTest();
    void initialize();
    void startReceiveing();

    static void* receive(void* arg);
    void waitUartThreadsEnd();

};






#endif //UARTTEST_H


