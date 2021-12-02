
#ifndef UARTULTRA_H
#define UARTULTRA_H

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
#include "trajectoryexecutor.hpp"
using namespace std;

struct DistancesMsg{
    vector<double> distances;
    double timeSec;
    static DistancesMsg parseString(std::string r){//throws std::invalid_argument
        DistancesMsg res;
        std::stringstream ss(r);
        std::string s;
        std::getline( ss, s,',' );
        if(s.compare("dist")!=0) return res;// if start of msg not found return fail- empty vector


        while ( std::getline( ss, s,',' ) ) {
            try{
                double dist = std::stod(s);
                res.distances.push_back(dist);
            }catch(std::invalid_argument){
                return res;
            }
        }
        if(res.distances.size()==6){ //set time only if values are present, so that absence of time can by used to check invalid msg
            res.timeSec=TrajectoryExecutor::getSystemTimeSec();// move getSystime to utils?
        }
        return res;
    }
};
class UartUltra
{
public:
    static int uart0_filestream;
    pthread_t receivingThreadUart;
    pthread_t sendingThreadUart;
    static pthread_mutex_t mutexSend;
    static pthread_mutex_t mutexReceive;
static DistancesMsg distances;
    static char tx_buffer[];
    static int tx_size;
    static vector<uint8_t> rxframe;
    UartUltra();
    ~UartUltra();
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






#endif //UARTULTRA_H


