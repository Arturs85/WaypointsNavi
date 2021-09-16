#ifndef UDPCOMMUNICATION_H
#define UDPCOMMUNICATION_H


// Server side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string>
#include "platformmsgparser.hpp"



#define MAXLINE 1024
class Control;

class UdpCommunication{
public:
    static pthread_t receivingThread;
    static const int receivingPort = 8888;
    static const int sendingPort = 8889;
    static void* receivingLoop(void* arg);
    static void startReceivingThread();
    static 	int sockfd;
    static struct sockaddr_in servaddr, cliaddr;
    static char buffer[MAXLINE];
    //static char *hello = "Hello from server(comms)";
    static unsigned int lengthOfCliAdrr;
static PlatformMsgParser platformMsgparser;
   static void sendString(std::string s);
  static const std::string TAG;

};

#endif // UDPCOMMUNICATION_H
