#include "HexapodController.h"

HexapodController::HexapodController(uint8_t legCount_) {
    legCount = legCount_;
    numServo = legCount * 3;
}

void HexapodController::begin() {
    ax12a.begin(BaudRate, DirectionPin, &Serial);
    interpolating = false;
    lastFrame_ = millis();

    // MX28 memiliki resolusi 4096, sedangkan AX18 memiliki resolusi 1024
    // Nilai tengah untuk MX28 adalah 2048, untuk AX18 adalah 512
    for (int i = 0; i < poseSize; i++) {
        // Posisi tengah sesuai dengan tipe servo (coxa = AX18, femur/tibia = MX28)
        if (i % 3 == 0) { // Coxa (AX18)
            pose_[i] = 512;
            nextPose_[i] = 512;
        } else { // Femur dan Tibia (MX28)
            pose_[i] = 2048;
            nextPose_[i] = 2048;
        }
        speed_[i] = 0;
    }

    for (uint8_t i = 1; i <= numServo; i++) {
        ax12a.torqueStatus(i, true);
    }
}

void HexapodController::legDimension(uint16_t coxa, uint16_t femur, uint16_t tibia) {
    L_COXA = coxa;
    L_FEMUR = femur;
    L_TIBIA = tibia;
}

void HexapodController::bodyDimension(uint16_t Xcoxa, uint16_t Ycoxa, uint16_t Mcoxa) {
    X_COXA = Xcoxa / 2;
    Y_COXA = Ycoxa / 2;
    M_COXA = Mcoxa / 2;
}

void HexapodController::setSpeed(float x, float y, float r) {
    Xspeed = x;
    Yspeed = y;
    Rspeed = r;
}

float HexapodController::getSpeed(uint8_t type) {
    return type == XSPEED ? Xspeed : type == YSPEED ? Yspeed : type == RSPEED ? Rspeed : 1.0;
}

void HexapodController::setBodyPos(float x, float y, float z) {
    bodyPosX = x;
    bodyPosY = y;
    bodyPosZ = z;
}

void HexapodController::setBodyRot(float x, float y, float z) {
    bodyRotX = x;
    bodyRotY = y;
    bodyRotZ = z;
}

void HexapodController::initializeLegs(uint16_t x, uint16_t y, uint16_t z, uint16_t liftHeight_) {
    if (liftHeight_ < 14) liftHeight = 14;
    else liftHeight = liftHeight_;
    step = 0;

    legStep[0] = 4; legStep[1] = 0; legStep[2] = 4;
    legStep[3] = 0; legStep[4] = 4; legStep[5] = 0;

    halfLeg = legCount / 2;
    frontLeg = halfLeg / 2;
    backLeg = (halfLeg - frontLeg) - 1;
    // Testing
    // Xspeed = Yspeed = Rspeed = 0;
    // bodyRotX = bodyRotY = bodyRotZ = 0;
    // bodyPosX = bodyPosY = bodyPosZ = 0;

    // Initialize endpoints with symmetrical positions
    for (int i = 0; i < halfLeg; i++) {
        if (i < frontLeg)
            endPoint[i].x = x;
        else if (i > backLeg)
            endPoint[i].x = -x;
        else
            endPoint[i].x = 0;
        
        endPoint[i].y = -y;
        endPoint[i].z = z;

        // Mirror for right side
        endPoint[i + halfLeg].x = -endPoint[i].x;
        endPoint[i + halfLeg].y = y;
        endPoint[i + halfLeg].z = z;
    }
}

dof HexapodController::leg(int16_t x, int16_t y, int16_t z) {
    float coxa, femur, tibia;
    dof ans;

    // Improved inverse kinematics calculations
    coxa = degrees(atan2(x, y));
    
    // Calculate true X position considering coxa angle
    float trueX = sqrt(x*x + y*y) - L_COXA;
    float im = sqrt(pow(trueX, 2) + pow(z, 2));

    // Calculate femur angle
    float q1 = -atan2(z, trueX);
    float q2 = acos((pow(L_FEMUR, 2) + pow(im, 2) - pow(L_TIBIA, 2)) / (2 * L_FEMUR * im));
    femur = degrees(q1 + q2);

    // Calculate tibia angle
    tibia = degrees(acos((pow(L_FEMUR, 2) + pow(L_TIBIA, 2) - pow(im, 2)) / (2 * L_FEMUR * L_TIBIA)));

    // Convert to servo positions with proper scaling
    // AX18 (coxa) uses range 1023 steps for 300 degrees
    // MX28 (femur and tibia) uses range 4096 steps for 360 degrees
    ans.coxa = coxa * 1023 / 300;
    ans.femur = femur * 4096 / 360;
    ans.tibia = (180 - tibia) * 4096 / 360;

    return ans;
}

coordinate HexapodController::body(int16_t x, int16_t y, int16_t z, int16_t xDisp, int16_t yDisp, float zRot) {
    coordinate ans;
    
    float rotX = radians(bodyRotX);
    float rotY = radians(bodyRotY);
    float rotZ = radians(bodyRotZ + zRot);

    float cosB = cos(rotX);
    float sinB = sin(rotX);
    float cosG = cos(rotY);
    float sinG = sin(rotY);
    float cosA = cos(rotZ);
    float sinA = sin(rotZ);

    int totalX = x + xDisp + bodyPosX;
    int totalY = y + yDisp + bodyPosY;

    // Improved body transformation calculations
    ans.x = totalX - int(totalX * cosG * cosA + totalY * sinB * sinG * cosA + z * cosB * sinG * cosA - totalY * cosB * sinA + z * sinB * sinA) + bodyPosX;
    ans.y = totalY - int(totalX * cosG * sinA + totalY * sinB * sinG * sinA + z * cosB * sinG * sinA + totalY * cosB * cosA - z * sinB * cosA) + bodyPosY;
    ans.z = z - int(-totalX * sinG + totalY * sinB * cosG + z * cosB * cosG) + bodyPosZ;

    return ans;
}

position HexapodController::gaitGen(uint8_t legNum) {
    position gait = {0, 0, 0, 0};
    stepInCycle = 8;

    if (gaitsEnable) {
        switch((legStep[legNum] + step) % stepInCycle) {
            case 0:
                gait = {0, 0, -liftHeight, 0};
                break;
            case 1:
                gait = {Xspeed, Yspeed, -liftHeight, Rspeed};
                break;
            case 2:
                gait = {Xspeed, Yspeed, 0, Rspeed};
                break;
            case 3:
                gait = {Xspeed/2, Yspeed/2, 0, Rspeed/2};
                break;
            case 4:
                gait = {0, 0, 0, 0};
                break;
            case 5:
                gait = {-Xspeed/2, -Yspeed/2, 0, -Rspeed/2};
                break;
            case 6:
                gait = {-Xspeed, -Yspeed, 0, -Rspeed};
                break;
            case 7:
                gait = {-Xspeed, -Yspeed, -liftHeight, -Rspeed};
                break;
        }
    }
    
    return gait;
}

void HexapodController::run() {
    dof sol;
    coordinate req;
    position gait;
    unsigned char servoIDs[numServo];
    int positions[numServo];
    int speeds[numServo];

    // Initialize speeds array and servoIDs
    for (int i = 0; i < numServo; i++) {
        speeds[i] = 500;
        servoIDs[i] = i + 1;
    }

    // Calculate positions for all legs
    for (int i = 0; i < legCount; i++) {
        bool isRightSide = i >= halfLeg;
        int legIndex = isRightSide ? i - halfLeg : i;
        gait = gaitGen(i);

        // Determine X and Y coxa offsets based on leg position
        int yCoxa = isRightSide ? Y_COXA : -Y_COXA;
        int xCoxa;
        
        if (legIndex < frontLeg) {
            xCoxa = X_COXA;
        } else if (legIndex > backLeg) {
            xCoxa = -X_COXA;
        } else {
            xCoxa = 0;
        }

        // Calculate body transformations
        req = body(endPoint[i].x + gait.x, 
                  endPoint[i].y + gait.y, 
                  endPoint[i].z + gait.z, 
                  xCoxa, yCoxa, gait.r);

        // Calculate leg positions with corrected mirroring
        float yMultiplier = isRightSide ? 1 : -1;
        
        sol = leg(endPoint[i].x + req.x + gait.x,
                 yMultiplier * (endPoint[i].y + req.y + gait.y),
                 endPoint[i].z + req.z + gait.z);

        // Apply servo positions with corrected coxa angles and offsets
        int baseIndex = i * 3;
        
        // Corrected coxa angle calculation (AX18)
        if (isRightSide) {
            // Right side
            if (legIndex < frontLeg) {
                positions[baseIndex] = 512 - sol.coxa - nullVal;  // Front legs
            } else if (legIndex > backLeg) {
                positions[baseIndex] = 512 - sol.coxa + nullVal;  // Back legs
            } else {
                positions[baseIndex] = 512 - sol.coxa;  // Middle legs
            }
        } else {
            // Left side
            if (legIndex < frontLeg) {
                positions[baseIndex] = 512 + sol.coxa - nullVal;  // Front legs
            } else if (legIndex > backLeg) {
                positions[baseIndex] = 512 + sol.coxa + nullVal;  // Back legs
            } else {
                positions[baseIndex] = 512 + sol.coxa;  // Middle legs
            }
        }

        // Femur and tibia angles for MX28
        positions[baseIndex + 1] = 2048 + sol.femur;
        positions[baseIndex + 2] = 2048 + sol.tibia;

        // Ensure positions are within valid range
        positions[baseIndex] = constrain(positions[baseIndex], 0, 1023);       // AX18 range
        positions[baseIndex + 1] = constrain(positions[baseIndex + 1], 0, 4095); // MX28 range
        positions[baseIndex + 2] = constrain(positions[baseIndex + 2], 0, 4095); // MX28 range
    }

    // Apply leg shift if needed
    if (legShift > 0) {
        uint16_t temp[legShift];
        for (int i = 0; i < legShift; i++) {
            temp[i] = positions[i];
        }
        for (int i = legShift; i < numServo; i++) {
            positions[i - legShift] = positions[i];
        }
        for (int i = 0; i < legShift; i++) {
            positions[numServo - legShift + i] = temp[i];
        }
    }

    // Use syncWrite to directly update servo positions instead of using setNextPose
    ax12a.syncWritePosition(servoIDs, positions, speeds, numServo);

    step = (step + 1) % stepInCycle;
}

void HexapodController::setGaitEnable(bool enable) {
    gaitsEnable = enable;
}

void HexapodController::setLegShift(uint8_t shift) {
    legShift = (shift % legCount) * 3;
}

void HexapodController::setAngleLimit(uint16_t moving) {
    dof sol;
    
    // Calculate initial positions with improved symmetry
    for (int i = 0; i < legCount; i++) {
        bool isRightSide = i >= halfLeg;
        int legIndex = isRightSide ? i - halfLeg : i;
        
        // Calculate leg positions
        sol = leg(endPoint[i].x, endPoint[i].y, endPoint[i].z);
        
        // Apply servo positions with corrected symmetry
        int baseIndex = i * 3;
        
        // Coxa uses AX18 (center = 512)
        if (isRightSide) {
            servo[baseIndex] = 512 - sol.coxa + (legIndex < frontLeg ? -nullVal : 
                                              legIndex > backLeg ? nullVal : 0);
        } else {
            servo[baseIndex] = 512 + sol.coxa + (legIndex < frontLeg ? -nullVal :
                                              legIndex > backLeg ? nullVal : 0);
        }
        
        // Femur and Tibia use MX28 (center = 2048)
        servo[baseIndex + 1] = 2048 - sol.femur;
        servo[baseIndex + 2] = 2048 + sol.tibia;
        
        // Calculate and set limits for each servo
        for (int j = 0; j < 3; j++) {
            int servoIndex = baseIndex + j;
            // Adjust moving range based on servo type
            int range = (j == 0) ? (moving / 2) : moving * 4; // Scale for MX28
            
            // Set different limits based on servo type
            if (j == 0) { // Coxa (AX18)
                maxs[servoIndex] = servo[servoIndex] + range;
                mins[servoIndex] = servo[servoIndex] - range;
                
                // Enforce AX18 limits
                maxs[servoIndex] = constrain(maxs[servoIndex], 0, 1023);
                mins[servoIndex] = constrain(mins[servoIndex], 0, 1023);
            } else { // Femur and Tibia (MX28)
                maxs[servoIndex] = servo[servoIndex] + range;
                mins[servoIndex] = servo[servoIndex] - range;
                
                // Enforce MX28 limits
                maxs[servoIndex] = constrain(maxs[servoIndex], 0, 4095);
                mins[servoIndex] = constrain(mins[servoIndex], 0, 4095);
            }
            
            // Apply limits to servos
            ax12a.setAngleLimit(servoIndex + 1, mins[servoIndex], maxs[servoIndex]);
        }
    }
}

void HexapodController::setNull(uint16_t null_) {
    nullVal = null_;
}

void HexapodController::setNextPose(uint8_t id, uint16_t position) {
    if (id >= 1 && id <= poseSize) {
        // Ensure position is within valid range based on servo type
        if ((id - 1) % 3 == 0) { // Coxa (AX18)
            position = constrain(position, 0, 1023);
        } else { // Femur and Tibia (MX28)
            position = constrain(position, 0, 4095);
        }
        nextPose_[id - 1] = position;
    }
}

uint8_t HexapodController::readVoltage() {
    uint32_t totalVoltage = 0;
    uint8_t validReadings = 0;
    
    for (int i = 1; i <= numServo; i++) {
        uint8_t voltage = ax12a.readVoltage(i);
        if (voltage > 0) {  // Only count valid readings
            totalVoltage += voltage;
            validReadings++;
        }
    }
    
    return validReadings > 0 ? totalVoltage / validReadings : 0;
}

void HexapodController::moveToHome() {
    unsigned char servoIDs[numServo];
    int positions[numServo];
    int speeds[numServo];
    
    // Prepare arrays for sync write with different center positions
    for (int i = 0; i < numServo; i++) {
        servoIDs[i] = i + 1;
        if (i % 3 == 0) { // Coxa (AX18)
            positions[i] = 512;  // Center position for AX18
        } else { // Femur and Tibia (MX28)
            positions[i] = 2048; // Center position for MX28
        }
        speeds[i] = 100;     // Moderate speed for smooth movement
    }
    
    // Use sync write for efficient movement
    ax12a.syncWritePosition(servoIDs, positions, speeds, numServo);
}

void HexapodController::readPose() {
    for (int i = 0; i < poseSize; i++) {
        uint16_t pos = ax12a.readPosition(i + 1);
        if(pos == -1) {
          return;
        }
        if (pos != 0xFFFF) { // Check for valid reading
            pose_[i] = pos;
        }
    }
}

void HexapodController::interpolateSetup(uint16_t time) {
    float frames = (time / biolid_frame_length) + 1;
    lastFrame_ = millis();
    
    readPose();
    
    for (count = 0; count < poseSize; count++) {
        if (nextPose_[count] > pose_[count]) {
            speed_[count] = ((nextPose_[count] - pose_[count]) / frames) + 1;
        }
        else if (nextPose_[count] < pose_[count]) {
            speed_[count] = ((nextPose_[count] - pose_[count]) / frames) - 1;
        }
        else {
            speed_[count] = 0;
        }
    }
    
    interpolating = true;
}

void HexapodController::writePose() {
    unsigned char servoIDs[poseSize];
    int positions[poseSize];
    int speeds[poseSize];
    
    // Prepare arrays for sync write with improved error checking
    for (int i = 0; i < poseSize; i++) {
        servoIDs[i] = i + 1;
        
        // Apply constraints based on servo type
        if (i % 3 == 0) { // Coxa (AX18)
            positions[i] = constrain(pose_[i], mins[i], maxs[i]);
            speeds[i] = 500; // Default speed
        } else { // Femur and Tibia (MX28)
            positions[i] = constrain(pose_[i], mins[i], maxs[i]);
            speeds[i] = 500; // Default speed - might need adjustment for MX28
        }
    }
    
    // Use sync write for efficient movement
    ax12a.syncWritePosition(servoIDs, positions, speeds, poseSize);
}

void HexapodController::interpolateStep() {
    if (!interpolating) return;
    
    int16_t complete = poseSize;
    
    // Wait for frame timing
    while ((millis() - lastFrame_) < biolid_frame_length);
    lastFrame_ = millis();
    
    // Update positions with improved smoothing
    for (count = 0; count < poseSize; count++) {
        int16_t diff = nextPose_[count] - pose_[count];
        
        if (diff == 0) {
            complete--;
        }
        else {
            if (abs(diff) > abs(speed_[count])) {
                if (((diff > 0) && (speed_[count] > 0)) || 
                    ((diff < 0) && (speed_[count] < 0))) {
                    pose_[count] += speed_[count];
                }
                else {
                    pose_[count] -= (speed_[count] / 2);
                }
            }
            else {
                pose_[count] = nextPose_[count];
                complete--;
            }
        }
        
        // Ensure position stays within valid range based on servo type
        if (count % 3 == 0) { // Coxa (AX18)
            pose_[count] = constrain(pose_[count], 0, 1023);
        } else { // Femur and Tibia (MX28)
            pose_[count] = constrain(pose_[count], 0, 4095);
        }
    }
    
    if (complete <= 0) {
        interpolating = false;
    }
    
    writePose();
}