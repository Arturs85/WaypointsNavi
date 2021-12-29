#ifndef UIPARSER_HPP
#define UIPARSER_HPP


// Server side implementation of UDP client-server model

#include <string>
#include <vector>
#include "control.hpp"
//class Control;

class UiParser{
    int msgId =0;

public:
    enum UiMsgs {MODE_MANUAL, MODE_AUTO, CLEAR_WAYPOINTS, ADD_WAYPOINT, CONTROL, GYRO_DIR, SAVE_WAYPOINTS, PAUSE, RESUME, UNKNOWN};

    Control* control;

    void sendText(std::string text);
    void sendGyroDirection(double dir);

    void parseReply(std::string r);
   UiMsgs parseMsgType(std::string s);
// std::vector<PlatformMsg> replies;
 bool hasSentRequest = false;
 int cyslesBeforeResend =0;
 int testCommunication();// send motor speed readout request and wait for answer

 void sendDeltYaw(double deltaYaw);
 void sendState(States state);
private:


};

#endif // UIPARSER_HPP
