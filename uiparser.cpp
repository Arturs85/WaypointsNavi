#include "uiparser.hpp"
#include "uiudp.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include "control.hpp"
#include "waypointsfilesaver.hpp"


void UiParser::sendText(std::string text)
{
    UiUdp::sendString("TEXT,"+text);
}

void UiParser::sendGyroDirection(double dir)
{
    UiUdp::sendString("GYRO_DIR,"+std::to_string(dir));
    //std::cout<<" gyrodir: "<<dir<<std::endl;
}
void UiParser::sendDeltYaw(double deltaYaw)
{
    UiUdp::sendString("DELTA_YAW,"+std::to_string(deltaYaw));
}
void UiParser::sendState(States state)
{
    std::string stateStr;
    switch (state) {
    case States::IDLE:
        stateStr = "IDLE";
        break;
    case States::AUTO:
        stateStr = "AUTO";
        break;
    case States::MANUAL:
        stateStr = "MANUAL";
        break;
    case States::INIT_GPS:
        stateStr = "INIT_GPS";
        break;
    case States::INIT_GYRO:
        stateStr = "INIT_GYRO";
        break;

    default:
        stateStr = std::to_string((int)state);
        break;
    }
    UiUdp::sendString("STATE,"+stateStr);

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
        if(control->state!=States::MANUAL && control->state!=States::INIT_GPS && control->state!=States::IDLE) break; // forward motor control only in MANUAL state, but allow manual comands on startup, when initiialising gps
        if(msgSplited.size()<3)return;
        try{
            int speed = std::stoi(msgSplited.at(1));

            double radi = std::stod(msgSplited.at(2));
            control->motorControl->setWheelSpeedsCenter(speed/200.0,radi);

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
        control->motorControl->setWheelSpeedsCenter(0,0);
        sendText("Paused");
    }
        break;
    case UiMsgs::RESUME:{
        if(control->state!=States::AUTO) break;
        Control::pathExecutor.resumeFromPause();
        sendText("Resumed");

    }
        break;
    case UiMsgs::PID:{
        if(msgSplited.size()<5)return;
        try{
            int p = std::stoi(msgSplited.at(1));
            int i = std::stoi(msgSplited.at(2));
            int d = std::stoi(msgSplited.at(3));
            int t = std::stoi(msgSplited.at(4));
            Control::pathExecutor.te.pidLinVel.pc=p/100.0;
            Control::pathExecutor.te.pidLinVel.ic=i/100.0;
            Control::pathExecutor.te.pidLinVel.dc=d/100.0;
           // Control::pathExecutor.te.pidRatioAngVel=t/100.0;
            sendText("angVel PID updated");
        }catch(std::invalid_argument){
            return;
        }

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
    if(s.compare("PID")==0)return UiMsgs::PID;

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
