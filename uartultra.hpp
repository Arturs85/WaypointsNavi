
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
    vector<int> distances;
    double timeSec;
    static const int obstTresholdSides = 30;
    static const int obstTresholdFront = 40;
    static const int sensorCount =6;//or rather measurements in the message
    double ratioLpf = 0.3;

    void addDistancesWithLp(vector<int> distancesOther){
        if(distancesOther.size()!=distances.size())  return;
        for (int i = 0; i < distances.size(); ++i) {
            distances.at(i) = distances.at(i)*ratioLpf+(1-ratioLpf)*distancesOther.at(i);
        }

    }
    static DistancesMsg parseString(std::string r){//throws std::invalid_argument
        DistancesMsg res;
        std::stringstream ss(r);
        std::string s;
        std::getline( ss, s,',' );
        if(s.compare("dist")!=0) {std::cout<<"[UU] cant parse dist msg, unexpected start"<<std::endl; return res;}// if start of msg not found return fail- empty vector


        while ( std::getline( ss, s,',' ) ) {
            try{
                double dist = std::stoi(s);
                res.distances.push_back(dist);
            }catch(std::invalid_argument){
                std::cout<<"[UU] cant parse dist msg, stoi arg not valid"<<std::endl;
                return res;
            }
        }
        if(res.distances.size()==6){ //set time only if values are present, so that absence of time can by used to check invalid msg
            res.timeSec=TrajectoryExecutor::getSystemTimeSec();// move getSystime to utils?
        }
        return res;
    }
    bool hasObstacle(){// 0,5 - sides, 1,4- front
        if(distances.size()<sensorCount) return true;
        //if(distances.at(0)<obstTresholdSides || distances.at(5)<obstTresholdSides) return true;
        //if(distances.at(1)<obstTresholdFront || distances.at(4)<obstTresholdFront) return true;
        if(distances.at(0)<obstTresholdSides) return true;
        if(distances.at(1)<obstTresholdFront) return true;
        return false;
    }
    bool hasObstacleSides(){// 0,5 - sides, 1,4- front
        if(distances.size()<sensorCount) return true;
        if(distances.at(0)<obstTresholdSides || distances.at(3)<obstTresholdSides) return true;
        return false;
    }
    bool hasObstacleFront(){// 0,5 - sides, 1,4- front
        if(distances.size()<sensorCount) return true;
        if(distances.at(2)<obstTresholdFront) return true;
        if(distances.at(1)<obstTresholdFront) return true;
        return false;
    }
    bool hasObstacleFront(double velMs, double decc){// 0,5 - sides, 1,4- front
        double brakeingDistanceM = velMs*velMs/(2*decc);
       // std::cout<<"[UU] brakeingDist, m: "<<brakeingDistanceM<<std::endl;
        double obstTresholdFrontLocal = obstTresholdFront+brakeingDistanceM*100;
        if(distances.size()<sensorCount) return true;
        if(distances.at(2)<obstTresholdFrontLocal) return true;
        if(distances.at(1)<obstTresholdFrontLocal) return true;
        return false;
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
   static void* receive(void* arg);
    static void* sendingLoop(void* arg);
    void waitUartThreadsEnd();

};






#endif //UARTULTRA_H


