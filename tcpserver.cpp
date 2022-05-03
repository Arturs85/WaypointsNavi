#include "tcpserver.hpp"
#include <signal.h>

#include <iostream>
#include <thread>
#define SA struct sockaddr
const std::string TcpServer::TAG = "[tcpClient] ";


void TcpServer::startWaitForClientThread(){

    //   struct sigaction sigIntHandler;

    //  sigIntHandler.sa_handler = &TcpServer::handlerSigpipe;
    //  sigemptyset(&sigIntHandler.sa_mask);
    //  sigIntHandler.sa_flags = 0;

    // sigaction(SIGINT, &sigIntHandler, NULL);
    hasClientConnected = false;
    std::thread t1(&TcpServer::waitForClient,this); // passing 'this' by
    t1.detach();
}

void TcpServer::startServerSocket()
{
    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        printf("socket creation failed...\n");
        //  exit(0);
    }
    else
        printf("Socket successfully created..\n");
    bzero(&servaddr, sizeof(servaddr));

    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(sendingPort);

    // Binding newly created socket to given IP and verification
    while ((bind(sockfd, (SA*)&servaddr, sizeof(servaddr))) != 0) {
        printf("socket bind failed...\n");
        usleep(1000000);
        //   exit(0);
    }

    printf("Socket successfully binded..\n");




}
void TcpServer::waitForClient()
{
    // Now server is ready to listen and verification
    if ((listen(sockfd, 1)) != 0) {
        printf("Listen failed...\n");
        //     exit(0);
    }
    else
        printf("Server listening..\n");
    lengthOfCliAdrr = sizeof(cliaddr);

    // Accept the data packet from client and verification
    connfd = accept(sockfd, (SA*)&cliaddr, &lengthOfCliAdrr);
    if (connfd < 0) {
        printf("server accept failed...\n");
        //     exit(0);
    }
    else{
        printf("server accept the client...\n");
        hasClientConnected = true;
    }
    std::string s = "from Server";
    //  int sent =  send (connfd, s.data(), s.size(),MSG_DONTWAIT);

    //if(sent<0)std::cout<<TAG<<" could not send: "<<s<<std::endl;

    std::cout<<TAG<< "started  thread, connfd: "<<connfd<<std::endl;
}

void TcpServer::sendString(std::string s)
{
    if(hasClientConnected){
        int sent =  send (connfd, s.data(), s.size(),MSG_DONTWAIT|MSG_NOSIGNAL);

        if(sent<0){
            std::cout<<TAG<<"connfd: "<<connfd<<" , could not send: "<<s<<std::endl;
            startWaitForClientThread();
        }
    }
}

void TcpServer::handlerSigpipe(int s){
    //  Control::pathExecutor.te.motorControl->rc->shutDown();
    //usleep(1000);

    //Control::pathExecutor.te.motorControl->rc->shutDown();
    std::cout<<"Caught signal"<<  s<<std::endl;;

    exit(0);

}

