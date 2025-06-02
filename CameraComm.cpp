#include "CameraComm.h"

CameraComm::CameraComm() {
    lastData.message = 0;
    lastData.width = 0;
    lastData.pidValue = 0;
    lastData.isValid = false;
}

void CameraComm::init() {
    Serial4.begin(SERIAL4_BAUD);
    Serial.println("Camera Communication initialized");
}

bool CameraComm::hasNewData() {
    return Serial4.available() > 0;
}

CameraData CameraComm::readData() {
    if (hasNewData()) {
        String data = Serial4.readStringUntil('\n');
        data.trim();
        return parseData(data);
    }
    
    CameraData emptyData;
    emptyData.isValid = false;
    return emptyData;
}

CameraData CameraComm::parseData(const String& data) {
    CameraData parsedData;
    parsedData.isValid = false;
    
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    
    if (firstComma != -1) {
        String messageStr = data.substring(0, firstComma);
        String widthHexStr = data.substring(firstComma + 1, secondComma);
        String pidStr = "";
        
        if (secondComma != -1) {
            pidStr = data.substring(secondComma + 1);
        }
        
        parsedData.message = messageStr.toInt();
        parsedData.width = strtol(widthHexStr.c_str(), NULL, 16);
        parsedData.pidValue = pidStr.toInt();
        parsedData.isValid = true;
        
        lastData = parsedData;
    }
    
    return parsedData;
}

void CameraComm::printData(const CameraData& data) const {
    if (data.isValid) {
        Serial.print("Cam - Msg: ");
        Serial.print(data.message);
        Serial.print(", Width: ");
        Serial.print(data.width);
        Serial.print(", PID: ");
        Serial.println(data.pidValue);
    }
}