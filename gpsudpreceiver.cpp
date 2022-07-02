#include "gpsudpreceiver.hpp"
#include "control.hpp"
#include <iostream>
//int UdpCommunication::server_fd;
int GpsUdpReceiver::sockfd;
struct sockaddr_in GpsUdpReceiver::cliaddr;
struct sockaddr_in GpsUdpReceiver::servaddr;
unsigned int GpsUdpReceiver::lengthOfCliAdrr;

pthread_t GpsUdpReceiver::receivingThread;

const std::string GpsUdpReceiver::TAG = "[gpsUdp] ";





void GpsUdpReceiver::startReceivingThread()
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

   // cliaddr.sin_addr.s_addr = inet_addr("192.168.43.1");
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

void GpsUdpReceiver::sendString(std::string s)
{
    //std::cout<<TAG<<"sending to ui dev: "<<s<<std::endl;
    sendto(sockfd, s.data(), s.size(),
           MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
           lengthOfCliAdrr);
}

void * GpsUdpReceiver::receivingLoop(void *arg)
{
    std::cout<<TAG<<" receiving loop started "<<std::endl;

    char buffer[MAXLINE] = {0};
    while(true){
        //int  valread = read( new_socket , buffer, 1024);

        int   n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &lengthOfCliAdrr);

        if(n>=1){
            buffer[n] = '\0';
            //uiParser.parseReply(std::string(buffer));
            std::string llhData =std::string(buffer);
            //  std::cout<<llhData;
            try{
                ReachLLHmsg msg = ReachLLHmsg::parseNmeaString(llhData);
                // todo call onGPS()
                //Control::particleFilter.onGpsWoOdo(msg.lat,msg.lon,msg.sdn_m);
                Control::particleFilter.onGps(msg.lat,msg.lon,msg.sdn_m,msg.sde_m);

            }catch(std::invalid_argument){
                std::cout<<"[GpsUdpReceiver] nmea msg not valid: "<<llhData<<std::endl;
               // continue;
            }
        }else{
            // canSend=false;
            std::cout<<"receiving error "<<n<<std::endl;
            //break;
        }
    }
}

