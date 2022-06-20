#include "uiparser.hpp"
#include "uiudp.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include "control.hpp"
#include "waypointsfilesaver.hpp"
#include "uiweb.hpp"

void UiParser::sendString(std::string text){
    UiUdp::sendString(text);
    UiWeb::uiWeb.sendString(text);
}

void UiParser::sendText(std::string text)
{
    sendString("TEXT,"+text);

}

void UiParser::sendGyroDirection(double dir)
{
    sendString("GYRO_DIR,"+std::to_string(dir));
    //std::cout<<" gyrodir: "<<dir<<std::endl;
}
void UiParser::sendDeltYaw(double deltaYaw)
{
    sendString("DELTA_YAW,"+std::to_string(deltaYaw));
}
void UiParser::sendHasObstaclesTimed(bool hasObstaclesFront, bool hasObstaclesSides)
{
    if(obstMsgCallCount++ % 20 ==0 ){

        if(hasObstaclesFront)  sendText("Obstacle Front");
        if(hasObstaclesSides)  sendText("Obstacle Sides");

    }
}

void UiParser::sendFileNames(){
    std::vector<std::string> names =  WaypointsFileSaver::waypointsFileSaver.readFileNames();
    for (int i = 0; i < names.size(); ++i) {
        sendString("FILE,"+names.at(i));
    }
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
    sendString("STATE,"+stateStr);

}
void UiParser::parseReply(std::string r) // process reply
{
    //spilt input to get original msg fields
    std::cout<<"UiParser parsing: "<<r<<std::endl;

    std::vector<std::string> msgSplited;
    std::stringstream ss(r);
    std::string s;
    while ( std::getline( ss, s,',' ) ) {
        msgSplited.push_back(s);
    }
    std::cout<<"msgSplited size: "<<msgSplited.size()<<std::endl;

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
        ssr.isRunning = false;// turn off stepresponse

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
        Position2D pos(Control::particleFilter.previousGPSPos.lon,Control::particleFilter.previousGPSPos.lat,Control::particleFilter.dirComplRad);
        WaypointsFileSaver::waypointsFileSaver.waypointsToSave.push_back(Waypoint(pos,0.0));//waypoint means intermediatePoint wo stoping
        sendText("Waypoint added");
    }
        break;
    case UiMsgs::ADD_FOTOPOINT : {
        if(control->state!=States::MANUAL) break; // save waypoints only in MANUAL state
        Position2D pos(Control::particleFilter.previousGPSPos.lon,Control::particleFilter.previousGPSPos.lat,Control::particleFilter.dirComplRad);
        WaypointsFileSaver::waypointsFileSaver.waypointsToSave.push_back(Waypoint(pos,11.0,1,1));
        sendText("Fotopoint added");
    }
        break;
    case UiMsgs::SAVE_WAYPOINTS : {
        if(control->state!=States::MANUAL || msgSplited.size()<2) break; // save waypoints only in MANUAL state

        WaypointsFileSaver::waypointsFileSaver.saveAddedPoints(msgSplited.at(1));// todo change to filename received from ui
        sendText("Waypoints saved: "+msgSplited.at(1) );
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
    case UiMsgs::SHUTDOWN:{

        sendText("Initiated shutdown, turn off power after 10 sec");
        system("shutdown -P now");
    }
        break;
    case UiMsgs::SEND_NAMES:{

        sendFileNames();
    }
        break;
    case UiMsgs::OPEN_FILE:{
        if(msgSplited.size()<2) break;
        Control::pathExecutor.loadPointsFile(msgSplited.at(1));
    }
        break;
    case UiMsgs::PID:{
        if(msgSplited.size()<5)return;
        try{
            int p = std::stoi(msgSplited.at(1));
            int i = std::stoi(msgSplited.at(2));
            int d = std::stoi(msgSplited.at(3));
            int t = std::stoi(msgSplited.at(4));
            Control::pathExecutor.te.pidAngleStatic.pc=3*p/100.0;
            Control::pathExecutor.te.pidAngleStatic.ic=i/100.0;
            Control::pathExecutor.te.pidAngleStatic.dc=3*d/100.0;
            Control::pathExecutor.te.pidAngleStatic.maxI=t*3/100.0;
            sendText("angVel PID updated");
        }catch(std::invalid_argument){
            return;
        }

    }
        break;
    case UiMsgs::STEP_RESPONSE : {
        if(control->state!=States::MANUAL && control->state!=States::INIT_GPS && control->state!=States::IDLE) break;

        sendText("Step response command received");
        control->state = States::STEP_RESPONSE;
        ssr.start();
        // stay in this state, user must select next state from ui
    }
        break;
    case UiMsgs::USE_ULTRA:{
        control->pathExecutor.useObstacleDetection= true;
        sendText("Using obstacle detection ");
        //todo remove after test
        //Control::pathExecutor.sendTCPTrigerr(23.4567,56.3456789,"filename",123);
    }
        break;
    case UiMsgs::IGNORE_ULTRA:{
        control->pathExecutor.useObstacleDetection= false;
        sendText("Not using obstacle detection ");
    }
        break;
    case UiMsgs::FROM_STRING:{
        try {
            Waypoint wp = Waypoint::fromString(msgSplited.at(1));
            WaypointsFileSaver::waypointsFileSaver.waypointsToSave.push_back(wp);
            sendText("Point added");
        } catch (std::invalid_argument) {
            sendText("Could not add point from string");

        }
    }
        break;
    case UiMsgs::SEND_POINT:{
        //int pointNr = Control::pathExecutor.getCurrentPointNr();
        try {
            int nr = std::stoi(msgSplited.at(1));
            if(nr<0) nr =Control::pathExecutor.getCurrentPointNr();
            //std::string wp = Control::pathExecutor.getWaypointAsString(nr);
            int radius = 2;
            if(msgSplited.size()>=3)
                radius = std::stoi(msgSplited.at(2));
            std::vector<std::string>pointsStrings = Control::pathExecutor.getsSuroundingPoints(nr,radius);

            for (int i = 0; i < pointsStrings.size(); ++i) {
                std::string wp = pointsStrings.at(i);
                sendString("POINT,"+wp);
                std::cout<<"sending: "<<("POINT,"+wp)<<std::endl;
            }

        } catch (std::invalid_argument) {
            sendText("Could not read point with index "+msgSplited.at(1));

        }
    }
        break;

    case UiMsgs::MODIFY_POINT:{
        int start = msgSplited.at(1).find(" ");
        std::string indexStr = msgSplited.at(1).substr(0,start);
        std::string wptStr = msgSplited.at(1).substr(start);

        int index = std::stoi(indexStr);
        Waypoint wp = Waypoint::fromString(wptStr);
        std::cout<<"received modifie: "<<("nr: "+std::to_string(index)+". string: "+wptStr)<<std::endl;
        if(index<0){ // negative index means insert
            Control::pathExecutor.insertPointAfter(index* -1,wp);
        }else Control::pathExecutor.replacePoint(index,wp);

        Control::pathExecutor.saveCurrentPoints();
    }
        break;
    case UiMsgs::SET_TARGET:{
        if(msgSplited.size()<2) return;

        try {
            int index = std::stoi(msgSplited.at(1));
            Control::pathExecutor.manualTargetPointSelection(index);
            sendText("Manually selected point: "+std::to_string(index));
        } catch (std::invalid_argument) {
            sendText("Could not read point from string");
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
    if(s.compare("ADD_FOTOPOINT")==0)return UiMsgs::ADD_FOTOPOINT;
    if(s.compare("STEP_RESPONSE")==0)return UiMsgs::STEP_RESPONSE;
    if(s.compare("SHUTDOWN")==0)return UiMsgs::SHUTDOWN;
    if(s.compare("SEND_NAMES")==0)return UiMsgs::SEND_NAMES;
    if(s.compare("OPEN_FILE")==0)return UiMsgs::OPEN_FILE;
    if(s.compare("USE_ULTRA")==0)return UiMsgs::USE_ULTRA;
    if(s.compare("IGNORE_ULTRA")==0)return UiMsgs::IGNORE_ULTRA;
    if(s.compare("FROM_STRING")==0)return UiMsgs::FROM_STRING;
    if(s.compare("SEND_POINT")==0)return UiMsgs::SEND_POINT;
    if(s.compare("MODIFY_POINT")==0)return UiMsgs::MODIFY_POINT;
    if(s.compare("SET_TARGET")==0)return UiMsgs::SET_TARGET;

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
