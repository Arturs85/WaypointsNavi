#include "uiparser.hpp"
#include "uiudp.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include "control.hpp"
#include "waypointsfilesaver.hpp"
void UiParser::sendMotorControl(int rightSpeedProc, int leftSpeedProc)
{
    UiUdp::sendString(std::to_string(msgId)+";SSP;"+std::to_string(leftSpeedProc)+";"+std::to_string(rightSpeedProc));
    msgId++;
}

void UiParser::sendMotorSpeedRequest()
{
    UiUdp::sendString(std::to_string(msgId)+";RSP");
    msgId++;

}

void UiParser::sendText(std::string text)
{
    UiUdp::sendString("TEXT,"+text);
}

void UiParser::sendGyroDirection(double dir)
{
    UiUdp::sendString("GYRO_DIR,"+std::to_string(dir));
}
void UiParser::sendDeltYaw(double deltaYaw)
{
    UiUdp::sendString("DELTA_YAW,"+std::to_string(deltaYaw));
}

void UiParser::parseReply(std::string r) // process reply
{
    //spilt input to get original msg fields
    // std::cout<<"UiParser parsing: "<<r<<std::endl;

    std::vector<std::string> msgSplited;
    std::stringstream ss(r);
    std::string s;
    while ( std::getline( ss, s,',' ) ) {
        msgSplited.push_back(s);
    }
    if(msgSplited.size()<1) return;
    int msgId=0;
    //    try{
    //        msgId = std::stoi(msgSplited.at(0));
    //    }catch(std::invalid_argument){
    //        return;
    //    }
    UiMsgs msgType = parseMsgType(msgSplited.at(0));
    switch (msgType) { //MODE_MANUAL, MODE_AUTO, CLEAR_WAYPOINTS, ADD_WAYPOINT, LEFT, RIGHT
    case UiMsgs::MODE_AUTO : {
        control->enterAutoMode();
    }
        break;
    case UiMsgs::MODE_MANUAL : {
        control->enterManualMode();
    }
        break;
    case UiMsgs::CONTROL : {
        if(control->state!=States::MANUAL) break; // forward motor control only in MANUAL state
        if(msgSplited.size()<3)return;
        try{
            int speed = std::stoi(msgSplited.at(1));

            double radi = std::stod(msgSplited.at(2));
            control->motorControl.setWheelSpeedsCenter(speed,radi);

            //send speed to platform via motorControl
        }catch(std::invalid_argument){
            return;
        }

    }
        break;
    case UiMsgs::ADD_WAYPOINT : {
        if(control->state!=States::MANUAL) break; // save waypoints only in MANUAL state
        Position2D pos(Control::particleFilter.avgParticle.x,Control::particleFilter.avgParticle.y,Control::particleFilter.avgParticle.direction);
        WaypointsFileSaver::waypointsFileSaver.waypointsToSave.push_back(Waypoint(pos,3.0));
        sendText("Waypoint added");
    }
        break;
    case UiMsgs::SAVE_WAYPOINTS : {
        if(control->state!=States::MANUAL) break; // save waypoints only in MANUAL state
        WaypointsFileSaver::waypointsFileSaver.saveAddedPoints();
        sendText("Waypoints saved");
    }
        break;
    case UiMsgs::PAUSE:{
        if(control->state!=States::AUTO) break;
        Control::pathExecutor.enterPausedState();
        control->motorControl.setWheelSpeedsCenter(0,0);
        sendText("Paused");
    }
        break;
    case UiMsgs::RESUME:{
        if(control->state!=States::AUTO) break;
        Control::pathExecutor.resumeFromPause();
        sendText("Resumed");

    }
        break;
    default:

        break;
    }

}

UiParser::UiMsgs UiParser::parseMsgType(std::string s)
{
    if(s.compare("MODE_MANUAL")==0)return UiMsgs::MODE_MANUAL;
    if(s.compare("MODE_AUTO")==0)return UiMsgs::MODE_AUTO;
    if(s.compare("CONTROL")==0)return UiMsgs::CONTROL;
    if(s.compare("ADD_WAYPOINT")==0)return UiMsgs::ADD_WAYPOINT;
    if(s.compare("SAVE_WAYPOINTS")==0)return UiMsgs::SAVE_WAYPOINTS;
    if(s.compare("PAUSE")==0)return UiMsgs::PAUSE;
    if(s.compare("RESUME")==0)return UiMsgs::RESUME;


    else return UiMsgs::UNKNOWN;
}

int UiParser::testCommunication()
{
    //if(cyslesBeforeResend++ > 20) hasSentRequest = false;
    //    if(!hasSentRequest){
    //        replies.clear();
    //        hasSentRequest = true;
    //        sendMotorSpeedRequest();
    //    }
    //    if(replies.size()>0){ //todo synchronisation
    //        if(replies.at(replies.size()-1).type == PlatformMsgType::S){
    //            replies.clear();
    //            return 1;
    //        }
    //    }
    return 0;
}
