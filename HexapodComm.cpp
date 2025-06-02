#include "HexapodComm.h"

HexapodComm::HexapodComm() : 
    remoteIP(REMOTE_IP_1, REMOTE_IP_2, REMOTE_IP_3, REMOTE_IP_4),
    isConnected(false) {
}

bool HexapodComm::init() {
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());
        
        // Initialize UDP
        udp.begin(LOCAL_PORT);
        Serial.printf("UDP initialized on port %d\n", LOCAL_PORT);
        
        isConnected = true;
        return true;
    } else {
        Serial.println();
        Serial.println("WiFi connection failed!");
        isConnected = false;
        return false;
    }
}

void HexapodComm::sendCommand(int speedX, int speedY, int speedR) {
    if (!isConnected) return;
    
    HexapodCommand cmd;
    cmd.speedX = limitSpeed(speedX);
    cmd.speedY = limitSpeed(speedY);
    cmd.speedR = limitSpeed(speedR);
    cmd.bodyPosX = 0;
    cmd.bodyPosY = 0;
    cmd.bodyPosZ = 30;
    cmd.gait = 1;
    cmd.frequency = 10;
    
    sendCommand(cmd);
}

void HexapodComm::sendCommand(const HexapodCommand& cmd) {
    if (!isConnected) return;
    
    String command = buildCommand(cmd);
    
    udp.beginPacket(remoteIP, REMOTE_PORT);
    udp.print(command);
    udp.endPacket();
    
    Serial.println("Sent to Hexapod: " + command);
}

void HexapodComm::sendStop() {
    sendCommand(0, 0, 0);
}

int HexapodComm::limitSpeed(int speed) {
    return constrain(speed, -HEXAPOD_SPEED_LIMIT, HEXAPOD_SPEED_LIMIT);
}

String HexapodComm::buildCommand(const HexapodCommand& cmd) {
    return "speedX=" + String(cmd.speedX) + 
           ",speedY=" + String(cmd.speedY) + 
           ",speedR=" + String(cmd.speedR) + 
           ",bodyPosX=" + String(cmd.bodyPosX) + 
           ",bodyPosY=" + String(cmd.bodyPosY) + 
           ",bodyPosZ=" + String(cmd.bodyPosZ) + 
           ",gait=" + String(cmd.gait) + 
           ",frequency=" + String(cmd.frequency);
}

bool HexapodComm::isWiFiConnected() const {
    return isConnected && (WiFi.status() == WL_CONNECTED);
}

void HexapodComm::printStatus() const {
    Serial.print("WiFi Status: ");
    Serial.print(isConnected ? "Connected" : "Disconnected");
    if (isConnected) {
        Serial.print(" | IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(" | Remote: ");
        Serial.println(remoteIP);
    } else {
        Serial.println();
    }
}