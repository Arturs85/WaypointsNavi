#include "platformmsgparser.hpp"
#include "udpcommunication.hpp"
#include <iostream>
#include <string>
#include <sstream>

void PlatformMsgParser::sendMotorControl(int rightSpeedProc, int leftSpeedProc)
{
    UdpCommunication::sendString(std::to_string(msgId)+";SSP;"+std::to_string(leftSpeedProc)+";"+std::to_string(rightSpeedProc));
    msgId++;
}

void PlatformMsgParser::sendMotorSpeedRequest()
{
    UdpCommunication::sendString(std::to_string(msgId)+";RSP");
    msgId++;

}

void PlatformMsgParser::parseReply(std::string r)
{
    //spilt input to get original msg fields
    std::cout<<"PlatformMsgParser parsing: "<<r<<std::endl;

    std::vector<std::string> msgSplited;
    std::stringstream ss(r);
    std::string s;
    while ( std::getline( ss, s,';' ) ) {
        msgSplited.push_back(s);
    }
    if(msgSplited.size()<2) return;
    int msgId=0;
    try{
        msgId = std::stoi(msgSplited.at(0));
    }catch(std::invalid_argument){
        return;
    }
    PlatformMsgType msgType = parseMsgType(msgSplited.at(1));
    switch (msgType) {
    case PlatformMsgType::S : {
        if(msgSplited.size()<4)return;
        std::cout<<"msgSplited.size: "<<msgSplited.size()<<std::endl;
        int left;
        int right;
        try{
            left = std::stoi(msgSplited.at(2));
            right = std::stoi(msgSplited.at(3));
        } catch(std::invalid_argument){return;}
        PlatformMsg m;
        m.id = msgId;
        m.type = msgType;
        m.values.push_back((double)left);
        m.values.push_back((double)right);
        replies.push_back(m);
    }
        break;
    default:

        break;
    }

}

PlatformMsgType PlatformMsgParser::parseMsgType(std::string s)
{
    if(s.compare("S")==0)return PlatformMsgType::S;

    else return PlatformMsgType::UNKNOWN;
}

int PlatformMsgParser::testCommunication()
{
    if(cyslesBeforeResend++ > 20){
        hasSentRequest = false;
    cyslesBeforeResend =0;}
    if(!hasSentRequest){
        replies.clear();
        hasSentRequest = true;
        sendMotorSpeedRequest();
    }
    if(replies.size()>0){ //todo synchronisation
        if(replies.at(replies.size()-1).type == PlatformMsgType::S){
            replies.clear();
            return 1;
        }
    }
    return 0;
}
