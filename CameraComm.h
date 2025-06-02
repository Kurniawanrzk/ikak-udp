#ifndef CAMERA_COMM_H
#define CAMERA_COMM_H

#include <Arduino.h>
#include "Config.h"

struct CameraData {
    int message;
    int width;
    int pidValue;
    bool isValid;
};

enum CameraMessage {
    NO_MESSAGE = 0,
    UNKNOWN = 1,
    MOVE_LEFT = 2,
    TARGET_CENTERED = 3,
    MOVE_RIGHT = 4
};

class CameraComm {
private:
    CameraData lastData;
    
    // Private methods
    CameraData parseData(const String& data);
    
public:
    CameraComm();
    void init();
    bool hasNewData();
    CameraData readData();
    void printData(const CameraData& data) const;
};

#endif // CAMERA_COMM_H