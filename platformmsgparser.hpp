#ifndef PLATFORMMSGPARSER_HPP
#define PLATFORMMSGPARSER_HPP


// Server side implementation of UDP client-server model

#include <string>
#include <vector>
enum PlatformMsgType{S, UNKNOWN};
struct PlatformMsg
{
    int id;
    std::vector<double> values;
    PlatformMsgType type;
    PlatformMsg() {}
};
class PlatformMsgParser{
    int msgId =0;

public:


    void sendMotorControl(int rightSpeedProc, int leftSpeedProc);
    void sendMotorSpeedRequest();
    void parseReply(std::string r);
    PlatformMsgType parseMsgType(std::string s);

 std::vector<PlatformMsg> replies;
 bool hasSentRequest = false;
 int cyslesBeforeResend =0;
 int testCommunication();// send motor speed readout request and wait for answer

private:


};

#endif // PLATFORMMSGPARSER_HPP
