#ifndef ROBOT
#define ROBOT

#include <Arduino.h>
#include "AX12A.h"

#define CW_ANGLE_LIMIT_ADDR         6
#define CCW_ANGLE_LIMIT_ADDR        8
#define ANGLE_LIMIT_ADDR_LEN        2
#define OPERATING_MODE_ADDR_LEN     2
#define TORQUE_ENABLE_ADDR          24
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    25
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          30
#define GOAL_POSITION_ADDR_LEN      2
#define PRESENT_POSITION_ADDR       36
#define PRESENT_POSITION_ADDR_LEN   2
#define DXL_MAX_SERVOS 18
#define TIMEOUT 10    //default communication timeout 10ms
#define BIOLOID_SHIFT 4
#define DXL_MAX_SERVOS 18
#define TIME_OUT 100
#define biolid_frame_length 33  // ~30Hz frame rate
#define poseSize DXL_MAX_SERVOS // Use max servos as pose size
#define MIN_FRAME_TIME 20        // Minimum time between frames
#define MAX_FRAME_TIME 100       // Maximum time between frames#define poseSize DXL_MAX_SERVOS // Use max servos as pose size
#define DirectionPin  (16u)
#define BaudRate    (1000000ul)
#define BIOLOID_SHIFT 6
#define DXL_MAX_SERVOS 18
#define TIME_OUT 100

#define XSPEED 1
#define YSPEED 2
#define RSPEED 3



const float DYNAMIXEL_PROTOCOL_VERSION = 1.0;
const uint32_t DYNAMIXEL_BAUDRATE = 1000000;


// Structures
struct dof {
    float coxa;
    float femur;
    float tibia;
};

struct coordinate {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct position {
    int16_t x;
    int16_t y;
    int16_t z;
    float r;
};

class HexapodController {
private:
    // Robot dimensions
    uint16_t L_COXA;
    uint16_t L_FEMUR;
    uint16_t L_TIBIA;
    uint16_t X_COXA;
    uint16_t Y_COXA;
    uint16_t M_COXA;
    
    // Leg configuration
    uint8_t legCount;
    uint8_t numServo;
    uint8_t halfLeg;
    uint8_t frontLeg;
    uint8_t backLeg;
    uint8_t step;
    uint8_t stepInCycle;
    uint8_t legShift;
    uint16_t nullVal;  
    uint8_t turn_on = 1;
    uint8_t turn_off = 0;
    
    // Movement parameters
    int16_t servo[DXL_MAX_SERVOS];
    int16_t maxs[DXL_MAX_SERVOS];
    int16_t mins[DXL_MAX_SERVOS];
    coordinate endPoint[6];
    uint8_t legStep[6];
    bool gaitsEnable;
    
public:
    // Public parameters for control
    unsigned long lastFrame_;
    int16_t pose_[poseSize];
    int16_t nextPose_[poseSize];
    int16_t speed_[poseSize];
    uint16_t positions[DXL_MAX_SERVOS];
    uint8_t count;
    bool interpolating;
 // unsigned long currentMillis = millis();
  
  // // Handle setup sequence with millis() instead of delay()
  // if (!setupComplete) {
  //   switch (setupState) {
  //     case 0:
  //       if (currentMillis - setupMillis >= 2000) {
  //         setupMillis = currentMillis;
  //         setupState = 1;
  //       }
  //       break;
  //     case 1:
  //       ikak.run();
  //       setupMillis = currentMillis;
  //       setupState = 2;
  //       break;
  //     case 2:
  //       if (currentMillis - setupMillis >= 2000) {
  //         setupComplete = true;
  //       }
  //       break;
  //   }
  // } 
  // // Normal loop operation
  // else {
  //   digitalWrite(15, HIGH);
    
  //   // Run the hexapod every 100ms
  //   if (currentMillis - previousMillis >= 50) {
  //     previousMillis = currentMillis;
  //     ikak.run();
  //   }
  // }

    // Public parameters for control
    float Xspeed;
    float Yspeed;
    float Rspeed;
    float bodyRotX;
    float bodyRotY;
    float bodyRotZ;
    float bodyPosX;
    float bodyPosY;
    float bodyPosZ;
    float liftHeight;

    HexapodController(uint8_t legCount_);
    void begin();
    void initializeGaitPattern();
    void legDimension(uint16_t coxa, uint16_t femur, uint16_t tibia);
    void bodyDimension(uint16_t Xcoxa, uint16_t Ycoxa, uint16_t Mcoxa);
    void initializeLegs(uint16_t x, uint16_t y, uint16_t z, uint16_t liftHeight_);
    dof leg(int16_t x, int16_t y, int16_t z);
    coordinate body(int16_t x, int16_t y, int16_t z, int16_t xDisp, int16_t yDisp, float zRot);
    position gaitGen(uint8_t legNum);
    void setGaitEnable(bool enable);
    void setLegShift(uint8_t shift);
    void run();
    void setAngleLimit(uint16_t moving);
    void setNull(uint16_t null_);
    void setNextPose(uint8_t id, uint16_t position);
    uint8_t readVoltage();
    void moveToHome();
    void readPose();
    void interpolateSetup(uint16_t time);
    void writePose();
    void interpolateStep();
    void setSpeed(float x, float y, float r);
    float getSpeed(uint8_t type);
    void setBodyPos(float x, float y, float z);
    void setBodyRot(float x, float y, float z);
};


#endif
