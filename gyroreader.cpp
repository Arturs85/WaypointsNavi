/*
        MPU6050 Interfacing with Raspberry Pi
        http://www.electronicwings.com
*/
#include "gyroreader.hpp"
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <sys/time.h>
#include <cmath>
#include <limits>
#include <float.h>
#include <iostream>
#include "uiudp.hpp"
#include "control.hpp"

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47




 

  const std::string GyroReader::TAG = "[GyroReader] ";

void GyroReader::MPU6050_Init(){

    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
    wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
    wiringPiI2CWriteReg8 (fd, CONFIG, 0x03);		/* Write to Configuration register medium LPF frequency(second digit 3) - 0 means hf, 7- lowest f */
    wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 0x00);	/* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */

}

void GyroReader::startReadingThread()
{
                std::cout<<TAG<<"startReadingThread()"<<std::endl;
 gyroThread =std::thread(&GyroReader::readGyro,this);
}
short GyroReader::read_raw_data(int addr){
    short high_byte,low_byte,value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr+1);
    value = (high_byte << 8) | low_byte;
    return value;
}

double GyroReader::getSystemTimeSec(void){
    struct timeval start_time;
    double milli_time;
    gettimeofday(&start_time, NULL);

    milli_time = ((start_time.tv_usec) / 1000000.0 + start_time.tv_sec);
    return milli_time;
}


void GyroReader::readGyro(){

                std::cout<<TAG<<"readGyro()"<<std::endl;


    float Acc_x,Acc_y,Acc_z;
    float Gyro_x,Gyro_y,Gyro_z;
    float Ax=0, Ay=0, Az=0;
    float Gx=0, Gy=0, Gz=0;
    fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/
              
  std::cout<<TAG<<"wiringPiI2CSetup() done"<<std::endl;    

MPU6050_Init();		
                std::cout<<TAG<<"MPU6050_Init() done"<<std::endl;                     /* Initializes MPU6050 */
    timePreviousSec = getSystemTimeSec();
    delay(50);
                std::cout<<TAG<<"delay() done"<<std::endl;                     /* Initializes MPU6050 */
    while(1)
    {
        double t = getSystemTimeSec();
        double dt = t - timePreviousSec;
        timePreviousSec = t;

        /*Read raw value of Accelerometer and gyroscope from MPU6050*/
        Acc_x = read_raw_data(ACCEL_XOUT_H);
        Acc_y = read_raw_data(ACCEL_YOUT_H);
        Acc_z = read_raw_data(ACCEL_ZOUT_H);

        Gyro_x = read_raw_data(GYRO_XOUT_H);
        Gyro_y = read_raw_data(GYRO_YOUT_H);
        Gyro_z = read_raw_data(GYRO_ZOUT_H);

        /* Divide raw value by sensitivity scale factor */
        Ax = Acc_x/16384.0;
        Ay = Acc_y/16384.0;
        Az = Acc_z/16384.0;

        Gx = Gyro_x/131;
        Gy = Gyro_y/131;
        Gz = Gyro_z/131;

        gdc.onGyroEvent(dt, Gz);
        double driftZ=100;
        if(gdc.getGyroDriftZ(&driftZ)){
            //apply to position calc;
            Gz= Gz-driftZ;
        }
        directionZ += dt * Gz;
       // Control::particleFilter.onOdometry(dt);//just adds movement noise, because there is no actual odometry available in this phase
        Control::particleFilter.onGyro(Gz,dt);
UiUdp::uiParser.sendGyroDirection(directionZ);//for test
        //printf("%.3f Gx= %.3f °/s\tGy= %.3f °/s\tGz= %.3f °/s\tAx= %.3f g\tAy= %.3f g\tAvgDir= %.3f g dYaw %.3f diZ %.3f\n",t,Gx,Gy,Gz,Ax,Ay,Control::particleFilter.avgParticle.direction*180/M_PI,Control::particleFilter.deltaYaw,directionZ);
        delay(100);

    }
    return;
}
