#include "ToFSensor.h"

ToFSensor::ToFSensor() : distance(999), lastReadTime(0) {
}

bool ToFSensor::init() {
    // Initialize I2C for ToF sensor on Wire1
    Wire1.begin();
    Wire1.setClock(TOF_I2C_SPEED);
    
    // Initialize ToF sensor
    sensor.setBus(&Wire1);
    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize ToF sensor!");
        return false;
    }
    
    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    
    // Start continuous readings at a rate of one measurement every 50 ms
    sensor.startContinuous(50);
    
    Serial.println("ToF Sensor initialized successfully");
    return true;
}

void ToFSensor::update() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastReadTime >= TOF_INTERVAL) {
        lastReadTime = currentMillis;
        
        sensor.read();
        
        if (sensor.timeoutOccurred()) {
            Serial.println("ToF sensor timeout!");
            distance = 999; // Set to high value on timeout
        } else {
            distance = sensor.ranging_data.range_mm;
        }
    }
}

int ToFSensor::getDistance() const {
    return distance;
}

bool ToFSensor::isObjectClose() const {
    return (distance <= TOF_THRESHOLD && distance > 0);
}

void ToFSensor::printDistance() const {
    Serial.print("ToF Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
}