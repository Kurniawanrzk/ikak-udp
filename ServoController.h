#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
#include "AX12A.h"
#include "Config.h"

class ServoController {
private:
    // Servo objects
    Servo servo1;
    Servo servo2;
    Servo servo3;
    
    // Position tracking
    int currentPosition;
    int targetPosition;
    int prevTargetPosition;
    
    // PID variables
    float integral;
    float prevError;
    unsigned long prevPIDTime;
    
    // Movement tracking
    int prevMovement;
    int stopCounter;
    bool isStopping;
    
    // Private methods
    int calculatePID(int target, int current);
    int limitSpeed(int speed);
    int calculateMoveSpeed(int distance);
    
public:
    ServoController();
    void init();
    void update();
    
    // Movement control
    void moveLeft(int pidValue);
    void moveRight(int pidValue);
    void centerAndStop();
    void setTargetPosition(int position);
    
    // Gripper control
    void gripperDown();
    void gripperUp();
    void executeGraspSequence();
    
    // Status getters
    int getCurrentPosition() const;
    int getTargetPosition() const;
    bool isAtTarget() const;
    bool getStoppingState() const;
    
    // Debug
    void printStatus() const;
};

#endif // SERVO_CONTROLLER_H