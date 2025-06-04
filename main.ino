/*
 * Robotic Gripper Control System
 * 
 * This system integrates:
 * - Camera-based target detection and tracking
 * - Servo-controlled gripper positioning with PID control
 * - Time-of-Flight sensor for object proximity detection
 * - WiFi communication with hexapod robot
 * 
 * Author: [Your Name]
 * Date: [Current Date]
 */

#include "Config.h"
#include "ToFSensor.h"
#include "ServoController.h"
#include "CameraComm.h"
#include "HexapodComm.h"
#include "RPC.h"

// System components
ToFSensor tofSensor;
ServoController servoController;
CameraComm camera;
HexapodComm hexapod;

// System state
bool graspingComplete = false;
unsigned long previousMillis = 0;

// Function prototypes
void processTargetData(const CameraData& data);
void updateSystem();
void printSystemStatus();

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("=== Robotic Gripper Control System ===");
    Serial.println("Initializing components...");
    
    // Initialize ToF sensor
    if (!tofSensor.init()) {
        Serial.println("ERROR: ToF sensor initialization failed!");
        while (1) { delay(1000); }
    }
    
    // Initialize servo controller
    servoController.init();
    
    // Initialize camera communication
    camera.init();
    
    // Initialize hexapod communication
    if (!hexapod.init()) {
        Serial.println("WARNING: Hexapod communication failed to initialize");
    }
    
    Serial.println("=== System Ready ===");
    Serial.println("Waiting for camera data...\n");
    RPC.begin();
    delay(100);
    RPC.bind("kirim_ke_slave_dari_m7", sendCommandUDP);
}

void loop() {
    unsigned long currentMillis = millis();
    
    // Update ToF sensor readings
    tofSensor.update();
    
    // Process camera data if available
    if (camera.hasNewData() && !graspingComplete) {
        CameraData data = camera.readData();
        if (data.isValid) {
            camera.printData(data);
            processTargetData(data);
        }
    }
    
    // Update servo controller at fixed intervals
    if (currentMillis - previousMillis >= MAIN_INTERVAL) {
        previousMillis = currentMillis;
        updateSystem();
    }
    // Optional: Print system status periodically
    // printSystemStatus();
    
}

void processTargetData(const CameraData& data) {
    switch (data.message) {
        case MOVE_LEFT:
            servoController.moveLeft(data.pidValue);
            break;
            
        case TARGET_CENTERED:
            servoController.centerAndStop();
            
            // Check if object is close enough to grasp
            if (tofSensor.isObjectClose()) {
                Serial.println("Object detected within grasp range!");
                servoController.executeGraspSequence();
                graspingComplete = true;
                
                // Send stop command to hexapod
                hexapod.sendStop();
                
                Serial.println("=== Grasping Sequence Complete ===");
            } else {
                Serial.print("Object too far - Distance: ");
                Serial.print(tofSensor.getDistance());
                Serial.println(" mm");
            }
            break;
            
        case MOVE_RIGHT:
            servoController.moveRight(data.pidValue);
            break;
            
        default:
            Serial.println("Unknown camera command received");
            break;
    }
}

void updateSystem() {
    // Update servo controller
    servoController.update();
    
    // Print servo status for debugging
    if (!servoController.isAtTarget()) {
        servoController.printStatus();
    }
    
    // Print ToF distance periodically
    static unsigned long lastToFPrint = 0;
    if (millis() - lastToFPrint > 1000) {  // Print every second
        tofSensor.printDistance();
        lastToFPrint = millis();
    }
}

void printSystemStatus() {
    static unsigned long lastStatusPrint = 0;
    
    if (millis() - lastStatusPrint > 5000) {  // Print every 5 seconds
        Serial.println("\n=== System Status ===");
        Serial.print("Grasping Complete: ");
        Serial.println(graspingComplete ? "Yes" : "No");
        
        Serial.print("Servo Position: ");
        Serial.print(servoController.getCurrentPosition());
        Serial.print(" / Target: ");
        Serial.println(servoController.getTargetPosition());
        
        Serial.print("ToF Distance: ");
        Serial.print(tofSensor.getDistance());
        Serial.println(" mm");
        
        Serial.print("Object in Range: ");
        Serial.println(tofSensor.isObjectClose() ? "Yes" : "No");
        
        hexapod.printStatus();
        Serial.println("=====================\n");
        
        lastStatusPrint = millis();
    }
}