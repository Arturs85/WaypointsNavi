#include "uiudp.hpp"

#include <iostream>
//int UdpCommunication::server_fd;
int UiUdp::sockfd;
struct sockaddr_in UiUdp::cliaddr;
struct sockaddr_in UiUdp::servaddr;
unsigned int UiUdp::lengthOfCliAdrr;

pthread_t UiUdp::receivingThread;
UiParser UiUdp::uiParser;
const std::string UiUdp::TAG = "[udpui] ";





void UiUdp::startReceivingThread()
{

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
       // servaddr.sin_addr.s_addr = inet_addr("192.168.43.245");
    servaddr.sin_port = htons(receivingPort);

   // cliaddr.sin_addr.s_addr = inet_addr("192.168.43.46");
    cliaddr.sin_port = htons(sendingPort);
    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
              sizeof(servaddr)) < 0 )
    {
      std::cout<<TAG<<"bind filed"<<std::endl;
        exit(EXIT_FAILURE);
    }

    lengthOfCliAdrr = sizeof(cliaddr); //len is value/resuslt


    int iret1 = pthread_create( &receivingThread, NULL,receivingLoop , 0);

    if(iret1)
    {
        fprintf(stderr,"Error creating receiving thread return code: %d\n",iret1);
        return;//exit(-1);
    }
    std::cout<<TAG<< "started receiving thread\n";
}

void UiUdp::sendString(std::string s)
{
    //std::cout<<TAG<<"sending to ui dev: "<<s<<std::endl;
    sendto(sockfd, s.data(), s.size(),
           MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
           lengthOfCliAdrr);
}

void * UiUdp::receivingLoop(void *arg)
{
    std::cout<<TAG<<" receiving loop started "<<std::endl;

    char buffer[MAXLINE] = {0};
    char *hello = "Hello from server";
    while(true){
        //int  valread = read( new_socket , buffer, 1024);

        int   n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &lengthOfCliAdrr);

        if(n>=1){
            buffer[n] = '\0';
            uiParser.parseReply(std::string(buffer));
          //  std::cout<<TAG<<"received from "<<cliaddr.sin_addr.s_addr<<std::endl;

           // printf("%s\n",buffer );

            // send(new_socket , hello , strlen(hello) , 0 );

        }else{
            // canSend=false;
            std::cout<<"receiving error "<<n<<std::endl;
            //break;
        }
    }
}

