#ifndef CONFIG_H
#define CONFIG_H

// Servo Control Definitions
#define DirectionPin        (2u)
#define BaudRate           (1000000ul) 
#define RX_PIN             10
#define TX_PIN             11                                      
#define SERVO_ID           (1u)

// WiFi Configuration
#define WIFI_SSID          "GigaHotspot"
#define WIFI_PASSWORD      "123456789"
#define LOCAL_PORT         8888
#define REMOTE_PORT        12345
#define REMOTE_IP_1        192
#define REMOTE_IP_2        168
#define REMOTE_IP_3        4
#define REMOTE_IP_4        2

// PID Constants - Anti-overshoot configuration
#define KP                 0.3f
#define KI                 0.01f
#define KD                 1.0f

// Servo Position Limits
#define MAX_SERVO          812
#define MIN_SERVO          212
#define CENTER_SERVO       512

// Control Parameters
#define DEADBAND           15
#define APPROACH_THRESHOLD_FAR   100
#define APPROACH_THRESHOLD_NEAR  40
#define MAX_SPEED          70
#define MIN_SPEED          30
#define HEXAPOD_SPEED_LIMIT      20

// ToF Sensor Configuration
#define TOF_THRESHOLD      40
#define TOF_I2C_SPEED      400000

// Timing Configuration
#define MAIN_INTERVAL      50
#define TOF_INTERVAL       100

// Servo Pins
#define SERVO1_PIN         4
#define SERVO2_PIN         10
#define SERVO3_PIN         9

// Communication
#define SERIAL_BAUD        115200
#define SERIAL4_BAUD       115200

#endif // CONFIG_H