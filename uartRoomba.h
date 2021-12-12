
#ifndef UARTROOMBA_H
#define UARTROOMBA_H

#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>	 	//Used for UART
#include <pthread.h>
#include <cstring>
#include <vector>
#include <stdint.h>

using namespace std;

class UartRoomba
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
    UartRoomba();
    ~UartRoomba();
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






#endif //UARTROOMBA_H

