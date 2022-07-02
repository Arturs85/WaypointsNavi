
#ifndef ROOMBACONTROLLER_HPP
#define ROOMBACONTROLLER_HPP
#include <stdio.h>
#include <pthread.h>
#include <cstring>
#include <vector>
#include <stdint.h>
#include "uartRoomba.h"

using namespace std;

class RoombaController
{
public:

    const vector<uint8_t> lightBumpsRequest{142,45};
    const vector<uint8_t> battCapacityRequest{142,26};
    const vector<uint8_t> battChargeRequest{142,25};
    const vector<uint8_t> bumpsRequest{142,7};
    const vector<uint8_t> distanceRequest{142,19};
    const vector<uint8_t> angleRequest{142,20};
    const vector<uint8_t> startRequest{128};
    const vector<uint8_t> safeRequest{131};
    
    const vector<uint8_t> fullRequest{132};
    const vector<uint8_t> endRequest{133};
const vector<uint8_t> segmRequest{164, 65,66,67,68};



    RoombaController(UartRoomba* uartDevice);
    uint8_t readLightBumps();
    uint8_t readBumpsnWheelDrops();

    uint16_t readBattCapacity();
    uint16_t readBattCharge();

    int16_t readDistance();
    int16_t readAngle();

 static   void drive(int16_t velocity, int16_t radius);
 static   void driveMsg(int8_t WheelSpeedRightPercent, int8_t WheelSpeedLeftPercent);

    void sevenSegmentDisplay(uint8_t number);
void startSafe();
void startFull();
void shutDown();

static void driveForward();
static void turnLeft();
static void turnRight();

static void stopMoving();

private:
    int16_t readInt16(uint8_t* beData);
    uint16_t readUint16(uint8_t* beData);
    
   static UartRoomba* uartDevice;


};






#endif //ROOMBACONTROLLER_HPP

