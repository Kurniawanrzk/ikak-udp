#include "ServoController.h"

ServoController::ServoController() : 
    currentPosition(CENTER_SERVO),
    targetPosition(CENTER_SERVO),
    prevTargetPosition(CENTER_SERVO),
    integral(0),
    prevError(0),
    prevPIDTime(0),
    prevMovement(0),
    stopCounter(0),
    isStopping(false) {
}

void ServoController::init() {
    // Initialize servo objects
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    
    // Initialize AX-12 servo
    ax12a.begin(BaudRate, DirectionPin, &Serial3);
    delay(100);
    
    // Set initial position with higher speed
    ax12a.moveSpeed(SERVO_ID, CENTER_SERVO, 300);
    delay(500);
    
    // Lower the gripper initially
    gripperDown();
    
    prevPIDTime = millis();
    Serial.println("Servo Controller initialized");
}

void ServoController::update() {
    if (isStopping) {
        // Gradually approach center point when stopping
        targetPosition = (targetPosition * 3 + CENTER_SERVO) / 4;
        stopCounter++;
        
        if (stopCounter > 8) {
            targetPosition = CENTER_SERVO;
        }
    } else {
        stopCounter = 0;
    }
    
    // Only move if not at target position
    if (abs(currentPosition - targetPosition) > 5) {
        // Calculate PID movement
        int movement = calculatePID(targetPosition, currentPosition);
        
        if (movement != 0) {
            currentPosition += movement;
            currentPosition = constrain(currentPosition, MIN_SERVO, MAX_SERVO);
            
            // Calculate movement speed based on distance
            int distance = abs(targetPosition - currentPosition);
            int moveSpeed = calculateMoveSpeed(distance);
            
            // Execute movement
            ax12a.moveSpeed(SERVO_ID, currentPosition, moveSpeed);
        }
    } else if (isStopping && abs(currentPosition - CENTER_SERVO) > 8) {
        // Force return to center when stopping
        currentPosition = (currentPosition * 3 + CENTER_SERVO) / 4;
        ax12a.moveSpeed(SERVO_ID, currentPosition, MIN_SPEED);
    }
}

int ServoController::calculatePID(int target, int current) {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevPIDTime) / 1000.0;
    
    if (dt <= 0) dt = 0.01;
    
    // Calculate error
    float error = target - current;
    
    // Apply deadband to prevent small oscillations
    if (abs(error) < DEADBAND) {
        integral = 0;
        prevError = 0;
        return 0;
    }
    
    // Calculate derivative
    float derivative = (error - prevError) / dt;
    
    // Calculate integral with anti-windup
    if (abs(error) < APPROACH_THRESHOLD_NEAR) {
        integral = 0;
    } else {
        integral += error * dt;
        integral = constrain(integral, -200, 200);
    }
    
    // Calculate PID output
    float output = (KP * error) + (KI * integral) + (KD * derivative);
    int movement = (int)output;
    
    // Apply constraints based on distance to target
    if (abs(error) < APPROACH_THRESHOLD_NEAR) {
        movement = constrain(movement, -15, 15);
    } else if (abs(error) < APPROACH_THRESHOLD_FAR) {
        movement = constrain(movement, -30, 30);
    } else {
        movement = constrain(movement, -50, 50);
    }
    
    // Update previous values
    prevError = error;
    prevPIDTime = currentTime;
    prevMovement = movement;
    
    return movement;
}

int ServoController::calculateMoveSpeed(int distance) {
    if (distance < APPROACH_THRESHOLD_NEAR) {
        return MIN_SPEED;
    } else if (distance < APPROACH_THRESHOLD_FAR) {
        return map(distance, APPROACH_THRESHOLD_NEAR, APPROACH_THRESHOLD_FAR, MIN_SPEED, MAX_SPEED);
    } else {
        return MAX_SPEED;
    }
}

void ServoController::moveLeft(int pidValue) {
    prevTargetPosition = targetPosition;
    targetPosition = currentPosition + constrain(abs(pidValue), 5, 25);
    targetPosition = constrain(targetPosition, MIN_SERVO, MAX_SERVO);
    isStopping = false;
    Serial.println("Servo moving left");
}

void ServoController::moveRight(int pidValue) {
    prevTargetPosition = targetPosition;
    targetPosition = currentPosition - constrain(abs(pidValue), 5, 25);
    targetPosition = constrain(targetPosition, MIN_SERVO, MAX_SERVO);
    isStopping = false;
    Serial.println("Servo moving right");
}

void ServoController::centerAndStop() {
    isStopping = true;
}

void ServoController::setTargetPosition(int position) {
    targetPosition = constrain(position, MIN_SERVO, MAX_SERVO);
}

void ServoController::gripperDown() {
    servo1.write(0);
    for(int i = 0; i <= 130; i++) {
        servo3.write(i);
        servo2.write(round(i / 5));
        delay(10);
    }
}

void ServoController::gripperUp() {
    for(int i = 130; i >= 0; i--) {
        servo3.write(i);
        servo2.write(round(i / 6.5));
        delay(10);
    }
}

void ServoController::executeGraspSequence() {
    Serial.println("Grasping object sequence initiated");
    servo1.write(35);
    delay(500);
    gripperUp();
    delay(50);
    ax12a.move(SERVO_ID, CENTER_SERVO);
}

int ServoController::getCurrentPosition() const {
    return currentPosition;
}

int ServoController::getTargetPosition() const {
    return targetPosition;
}

bool ServoController::isAtTarget() const {
    return abs(currentPosition - targetPosition) <= 5;
}

bool ServoController::getStoppingState() const {
    return isStopping;
}

void ServoController::printStatus() const {
    Serial.print("Servo pos: ");
    Serial.print(currentPosition);
    Serial.print(" (target: ");
    Serial.print(targetPosition);
    Serial.println(")");
}