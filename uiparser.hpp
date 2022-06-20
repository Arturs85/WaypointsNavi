#ifndef UIPARSER_HPP
#define UIPARSER_HPP


// Server side implementation of UDP client-server model

#include <string>
#include <vector>
#include "control.hpp"
//class Control;
#include "stepresponsereader.hpp"

class UiParser{
    int msgId =0;

public:
    enum UiMsgs {MODE_MANUAL, MODE_AUTO, CLEAR_WAYPOINTS, ADD_WAYPOINT, CONTROL, GYRO_DIR, SAVE_WAYPOINTS, PAUSE, RESUME,PID,ADD_FOTOPOINT,STEP_RESPONSE,SHUTDOWN,SEND_NAMES,OPEN_FILE,USE_ULTRA,IGNORE_ULTRA,FROM_STRING, SEND_POINT, MODIFY_POINT,SET_TARGET, UNKNOWN};

    Control* control;
    StepResponseReader ssr;

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
 void sendFileNames();
 void sendHasObstaclesTimed(bool hasObstaclesFront, bool hasObstaclesSides);
 void sendString(std::string text);
private:
int obstMsgCallCount =0;

};

#endif // UIPARSER_HPP
