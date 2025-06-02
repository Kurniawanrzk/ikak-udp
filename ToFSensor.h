#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Wire.h>
#include <VL53L1X.h>
#include "Config.h"

class ToFSensor {
private:
    VL53L1X sensor;
    int distance;
    unsigned long lastReadTime;
    
public:
    ToFSensor();
    bool init();
    void update();
    int getDistance() const;
    bool isObjectClose() const;
    void printDistance() const;
};

#endif // TOF_SENSOR_H