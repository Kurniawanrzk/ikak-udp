#ifndef HEXAPOD_COMM_H
#define HEXAPOD_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"

struct HexapodCommand {
    int speedX;
    int speedY;
    int speedR;
    int bodyPosX;
    int bodyPosY;
    int bodyPosZ;
    int gait;
    int frequency;
};

class HexapodComm {
private:
    WiFiUDP udp;
    IPAddress remoteIP;
    bool isConnected;
    
    // Private methods
    int limitSpeed(int speed);
    String buildCommand(const HexapodCommand& cmd);
    
public:
    HexapodComm();
    bool init();
    void sendCommand(int speedX, int speedY, int speedR);
    void sendCommandUDP(const HexapodCommand& cmd);
    void sendStop();
    bool isWiFiConnected() const;
    void printStatus() const;
};

#endif // HEXAPOD_COMM_H