#include "udpcommunication.hpp"

#include <iostream>
//int UdpCommunication::server_fd;
int UdpCommunication::sockfd;
struct sockaddr_in UdpCommunication::cliaddr;
struct sockaddr_in UdpCommunication::servaddr;
unsigned int UdpCommunication::lengthOfCliAdrr;

pthread_t UdpCommunication::receivingThread;
PlatformMsgParser UdpCommunication::platformMsgparser;
const std::string UdpCommunication::TAG = "[udpcommunication] ";





void UdpCommunication::startReceivingThread()
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
    servaddr.sin_port = htons(receivingPort);

    //cliaddr.sin_port = htons(sendingPort);
    cliaddr.sin_family = AF_INET; // IPv4
    cliaddr.sin_addr.s_addr = inet_addr("192.168.4.1");
    cliaddr.sin_port = htons(sendingPort);

    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
              sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
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

void UdpCommunication::sendString(std::string s)
{
    sendto(sockfd, s.data(), s.size(),
           MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
           lengthOfCliAdrr);
}

void * UdpCommunication::receivingLoop(void *arg)
{
     std::cout<<TAG<<" receiving loop started "<<std::endl;
    char buffer[MAXLINE] = {0};
    char *hello = "Hello from server";
    while(true){
        //int  valread = read( new_socket , buffer, 1024);

        int   n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &lengthOfCliAdrr);

        if(n>=1){
          std::cout<<TAG<<" received from "<<cliaddr.sin_addr.s_addr<<std::endl;
            buffer[n] = '\0';
            platformMsgparser.parseReply(std::string(buffer));
            printf("%s\n",buffer );
            // send(new_socket , hello , strlen(hello) , 0 );

        }else{
            // canSend=false;
            std::cout<<"receiving error "<<n<<std::endl;
            break;
        }
    }
}

