#include "uarttest.hpp"
#include <iostream>
#include "control.hpp"

UartTest::UartTest()
{
}
UartTest::~UartTest()
{
    close(uart0_filestream);
}
int UartTest::uart0_filestream = -1;
int UartTest::tx_size = 0;
pthread_mutex_t UartTest::mutexSend = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t UartTest::mutexReceive = PTHREAD_MUTEX_INITIALIZER;

char UartTest::tx_buffer[255];// ={0,0,0,0,0,0,0,0,0,0} ;
vector<uint8_t> UartTest::rxframe;
void UartTest::initialize()
{
    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively


    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    uart0_filestream = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in  blocking read/write mode
    if (uart0_filestream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME]=0;
    options.c_cc[VMIN]=0;
    
usleep(10000);
    tcflush(uart0_filestream, TCIFLUSH);//add 10 ms delay before this line, change totcioFlush?
    tcsetattr(uart0_filestream, TCSANOW, &options);



}

void UartTest::send()
{
    //----- TX BYTES -----
    unsigned char tx_buffer[20];
    unsigned char *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];
    *p_tx_buffer++ = 'H';
    *p_tx_buffer++ = 'e';
    *p_tx_buffer++ = 'l';
    *p_tx_buffer++ = 'l';
    *p_tx_buffer++ = 'o';

    if (uart0_filestream != -1)
    {
        int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write

        printf("write returns %d\n",count);
        if (count < 0)
        {
            printf("UART TX error\n");
        }
    }
    else
    {
        printf("UART not opened \n");
    }
}


void* UartTest::receive(void* arg)
{
    //----- CHECK FOR ANY RX BYTES -----
    if (uart0_filestream != -1)
    {
                   struct timespec ts = {0, 15000000L };

        while(1){

            // Read up to 255 characters from the port if they are there

            char rx_buffer[256];
                            pthread_mutex_lock( &mutexReceive );

            int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
                  
                            pthread_mutex_unlock( &mutexReceive );

            if (rx_length < 0)
            {
                //An error occured (will occur if there are no bytes)
                            nanosleep (&ts, NULL);//sleep 15 ms

                continue;
            }
            else if (rx_length == 0)
            {
                //No data waiting
                
                            nanosleep (&ts, NULL);//sleep 15 ms
			continue;
            }
            else if(rx_length>0)
            {
                //Bytes received
                rx_buffer[rx_length] = '\0';
              //  printf("%i bytes read : \n", rx_length);//, rx_buffer);
              std::string llhData =std::string(rx_buffer);
              //  std::cout<<llhData;
                try{
ReachLLHmsg msg = ReachLLHmsg::parseString(llhData);
                   // todo call onGPS()
//Control::particleFilter.onGpsWoOdo(msg.lat,msg.lon,msg.sdn_m);
Control::particleFilter.onGps(msg.lat,msg.lon,msg.sdn_m,msg.sde_m);

              }catch(std::invalid_argument){
                 continue;
                }



                pthread_mutex_lock( &mutexReceive );

                for(int i =0;i<rx_length;i++){
                    rxframe.push_back(rx_buffer[i]);// remove
                   // cout<<+rx_buffer[i]<<" ";
                }
                        pthread_mutex_unlock( &mutexReceive );

         //       cout<<"\n";
            }
            nanosleep (&ts, NULL);//sleep 15 ms

        }//end while
        //printf("read returns %d\n", rx_length);
    }
}

void UartTest::clearRxFrame()
{
    rxframe.clear();
}

vector<uint8_t> UartTest::readNumberOfBytes(uint8_t noOfBytes )
{
    vector<uint8_t> result;
    int cyclesCounter =100;//timeout = 10*1.5 ms
    struct timespec ts = {0, 1500000L };

    while(cyclesCounter--)
    {

        pthread_mutex_lock( &mutexReceive );
        if(rxframe.size()>= noOfBytes)
        {
            result = rxframe;
            rxframe.clear();
            cyclesCounter=0;
        }

        pthread_mutex_unlock( &mutexReceive );
        if(cyclesCounter == 0) break;
        nanosleep (&ts, NULL);//sleep 1.5 ms

    }
    return result;
}

void UartTest::startReceiveing(){// and sending
    int iret1 = pthread_create( &receivingThreadUart, NULL, receive, 0);

    if(iret1)
    {
        fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
        return;//exit(-1);
    }

    int iret2 = pthread_create( &sendingThreadUart, NULL, sendingLoop, 0);

    if(iret1)
    {
        fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2);
        return;//exit(-1);
    }






}

void* UartTest::sendingLoop(void* arg){

    while(1){
        if (uart0_filestream != -1)
        {
            pthread_mutex_lock( &mutexReceive );
            if(tx_size)
            {
              //  printf("sending data\n");

                int count = write(uart0_filestream, tx_buffer, tx_size);		//Filestream, bytes to write, number of bytes to write
               if(count!=tx_size)
                printf("sent %d bytes of %d\n",count, tx_size);

                tx_size=0;
            }
            pthread_mutex_unlock(&mutexReceive);
            struct timespec ts = {0, 1500000L };

            nanosleep (&ts, NULL);
        }
    }
}
void UartTest::setDataToTransmit(char* dataPtr, int noOfBytes){
    pthread_mutex_lock( &mutexReceive );
    memcpy(&tx_buffer[0],dataPtr,noOfBytes);
    tx_size = noOfBytes;
    //printf("data set for sending\n");
    pthread_mutex_unlock(&mutexReceive);
}

void UartTest::setDataToTransmit(vector<uint8_t> comm){
    pthread_mutex_lock( &mutexReceive );

    memcpy(&tx_buffer[0], &comm[0], comm.size());
    tx_size = comm.size();
    //printf("data set for sending\n");
    pthread_mutex_unlock(&mutexReceive);
}

void UartTest::waitUartThreadsEnd()
{

    if(pthread_join(receivingThreadUart, NULL)) {

        fprintf(stderr, "Error joining thread\n");
        return ;
    }
    if(pthread_join(sendingThreadUart, NULL)) {

        fprintf(stderr, "Error joining thread\n");
        return ;
    }

}



