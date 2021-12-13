#include "roombaController.hpp"
#include <iostream>

#define DRIVING_SPEED 110

UartRoomba* RoombaController::uartDevice=0;
RoombaController::RoombaController(UartRoomba *uartDevice)
{
    cout<<"roomba controller constructor\n";
    this->uartDevice=uartDevice;
}

uint8_t RoombaController::readLightBumps()
{
    uint8_t* data;
    uartDevice->setDataToTransmit(lightBumpsRequest);

    vector<uint8_t> ch = uartDevice->readNumberOfBytes(1);
    // cout<<"vector ch size: "<<ch.size()<<"\n";
    if(ch.size()==1)
    {//convertData

        data = reinterpret_cast<uint8_t*>(&ch[0]);
        //cout<<"bat charge received: "<<*batCharge<<"mAh\n";
        return *data;
    }
    else return 0;
    

}

uint8_t RoombaController::readBumpsnWheelDrops()
{
    uint8_t* data;
    uartDevice->setDataToTransmit(bumpsRequest);

    vector<uint8_t> ch = uartDevice->readNumberOfBytes(1);
    // cout<<"vector ch size: "<<ch.size()<<"\n";
    if(ch.size()==1)
    {//convertData

        data = reinterpret_cast<uint8_t*>(&ch[0]);
        //cout<<"bat charge received: "<<*batCharge<<"mAh\n";
        return *data;
    }
    else return 0;

}

uint16_t RoombaController::readBattCapacity()
{
    uartDevice->setDataToTransmit(battCapacityRequest);

    vector<uint8_t> ch = uartDevice->readNumberOfBytes(2);
    // cout<<"vector ch size: "<<ch.size()<<"\n";
    if(ch.size()==2)
    {//convertData
        return readUint16(reinterpret_cast<uint8_t*>(&ch[0]));
    }
    else return 0;
    
}


uint16_t RoombaController::readBattCharge()
{
    uartDevice->setDataToTransmit(battChargeRequest);

    vector<uint8_t> ch = uartDevice->readNumberOfBytes(2);
    // cout<<"vector ch size: "<<ch.size()<<"\n";
    if(ch.size()==2)
    {//convertData
        return readUint16(reinterpret_cast<uint8_t*>(&ch[0]));


    }
    else return 0;
    



}

int16_t RoombaController::readDistance()
{
    uartDevice->setDataToTransmit(distanceRequest);

    vector<uint8_t> ch = uartDevice->readNumberOfBytes(2);
    // cout<<"vector ch size: "<<ch.size()<<"\n";
    if(ch.size()==2)
    {//convertData

        return readInt16 (reinterpret_cast<uint8_t*>(&ch[0]));
        //cout<<"bat charge received: "<<*batCharge<<"mAh\n";
    }
    else return 0;

}

int16_t RoombaController::readAngle()
{
    uartDevice->setDataToTransmit(angleRequest);

    vector<uint8_t> ch = uartDevice->readNumberOfBytes(2);
    // cout<<"vector ch size: "<<ch.size()<<"\n";
    if(ch.size()==2)
    {//convertData
        return readInt16 (reinterpret_cast<uint8_t*>(&ch[0]));

    }
    else return 0;

}

void RoombaController::drive(int16_t velocity, int16_t radius)
{
    uint8_t array[5]={137,*(reinterpret_cast<uint8_t*>(&velocity)+1),*(reinterpret_cast<uint8_t*>(&velocity)),*(reinterpret_cast<uint8_t*>(&radius)+1),*(reinterpret_cast<uint8_t*>(&radius))};
    char* pt = reinterpret_cast<char*>(array);
    uartDevice->setDataToTransmit(pt,5);// +1 lai nesūta nulles pirms '137' kommandas
//    cout<<"first command byte: "<<(*(pt))+0;
}

void RoombaController::sevenSegmentDisplay(uint8_t number)
{
 vector<uint8_t> ch;
 ch.push_back(164);
 ch.push_back(number++); //temp
ch.push_back(number++);
ch.push_back(number++);
ch.push_back(number++);
uartDevice->setDataToTransmit(segmRequest);
}

void RoombaController::startSafe()
{
    uartDevice->setDataToTransmit(startRequest);
usleep(15000);
    uartDevice->setDataToTransmit(safeRequest);
usleep(1000);
}

void RoombaController::startFull()
{
 uartDevice->setDataToTransmit(startRequest);
usleep(100000);
    uartDevice->setDataToTransmit(fullRequest);
usleep(50000);
}

void RoombaController::shutDown()
{
    uartDevice->setDataToTransmit(endRequest);
    uartDevice->setDataToTransmit(endRequest);
    
cout<<"sent power off command\n";
}

int16_t RoombaController::readInt16(uint8_t* beData)
{
    uint8_t leData[2]={beData[1],beData[0]};
    int16_t* data = reinterpret_cast<int16_t*>(&leData[0]);
    //cout<<"bat charge received: "<<*batCharge<<"mAh\n";
    return *data;

}

uint16_t RoombaController::readUint16(uint8_t* beData)
{
    uint8_t leData[2]={beData[1],beData[0]};
    uint16_t* data = reinterpret_cast<uint16_t*>(&leData[0]);
    //cout<<"bat charge received: "<<*batCharge<<"mAh\n";
    return *data;

}

void RoombaController::driveForward()
{
    cout<<"fwd called \n";

drive(DRIVING_SPEED,32767);
}

void RoombaController::turnLeft()
{
    cout<<"left called \n";
    drive(DRIVING_SPEED,1);
}

void RoombaController::turnRight()
{
    cout<<"turn right called \n";
    drive(DRIVING_SPEED,-1);
}

void RoombaController::stopMoving()
{
    cout<<"stop called \n";
drive(0,0);
}
