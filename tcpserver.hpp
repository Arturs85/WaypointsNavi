#ifndef TCPSERVER_HPP
#define TCPSERVER_HPP


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



#define MAXLINE 1024
class Control;

class TcpServer{
public:
  
    static const int receivingPort = 9888;
    static const int sendingPort = 9889;
    void* receivingLoop(void* arg);
   void startReceivingThread();
     volatile   int sockfd, connfd;
     struct sockaddr_in servaddr, cliaddr;
     char buffer[MAXLINE];
    //static char *hello = "Hello from server(comms)";
     unsigned int lengthOfCliAdrr;

    void sendString(std::string s);
  static const std::string TAG;

  void waitForClient();
  void startWaitForClientThread();
  void handlerSigpipe(int s);
  void startServerSocket();

  bool hasClientConnected  = false;
};

#endif // TCPSERVER_HPP
