#include <WiFi.h>
#include <WiFiUdp.h>
#include "HexapodController.h"

// WiFi credentials
const char* ssid = "GigaHotspot";
const char* password = "123456789";

// UDP settings
WiFiUDP udp;
unsigned int localPort = 12345;

HexapodController ikak(6);  // 6-legged hexapod

// Configuration structure
struct Config {
  float speedX = 20, speedY = 0, speedR = 0;
  uint16_t liftHeight = 25;
  bool gaitEnabled = true;
  uint16_t legCoxa = 20;
  uint16_t legFemur = 50;
  uint16_t legTibia = 66;
  uint16_t bodyX = 74;
  uint16_t bodyY = 74;
  uint16_t bodyM = 100;
  uint16_t initX = 59;
  uint16_t initY = 72;
  uint16_t initZ = 41;
  uint16_t nullValue = 30;
  float bodyPosX = 0;
  float bodyPosY = 0;
  float bodyPosZ = 30;
  float bodyRotX = 0;
  float bodyRotY = 0;
  float bodyRotZ = 0;
  uint16_t updateFrequency = 5; // Default to 25Hz (40ms interval)
} config;

// Timing variables
unsigned long previousReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  
  // Enable servo power
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
  
  setupWiFi();
  
  udp.begin(localPort);
  Serial.println("IP: " + WiFi.localIP().toString());
  
  // Initialize the hexapod controller
  ikak.begin();
  applyConfig();
    // pinMode(2,OUTPUT);
  Serial.println("Hexapod controller initialized");
}

void applyConfig() {
  // Apply the current configuration to the hexapod
  ikak.legDimension(config.legCoxa, config.legFemur, config.legTibia);
  ikak.bodyDimension(config.bodyX, config.bodyY, config.bodyM);
  ikak.initializeLegs(config.initX, config.initY, config.initZ, config.liftHeight);
  ikak.setNull(config.nullValue);
  ikak.setGaitEnable(config.gaitEnabled);
  ikak.setBodyPos(config.bodyPosX, config.bodyPosY, config.bodyPosZ);
  ikak.setBodyRot(config.bodyRotX, config.bodyRotY, config.bodyRotZ);
  ikak.setSpeed(config.speedX, config.speedY, config.speedR);
}

void setupWiFi() {
  Serial.println("\nConnecting to WiFi: " + String(ssid));
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.println("IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void reconnectWiFi() {
  unsigned long currentMillis = millis();
  
  if (WiFi.status() != WL_CONNECTED && 
      currentMillis - previousReconnectAttempt >= reconnectInterval) {
      
    previousReconnectAttempt = currentMillis;
    
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && 
           millis() - startAttempt < 5000) {
      delay(100);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi reconnected. IP: " + WiFi.localIP().toString());
    }
  }
}

void processCommand(String cmd) {
  cmd.toLowerCase();
  cmd.replace(" ", "");
  
  int start = 0;
  while (start < cmd.length()) {
    int eq = cmd.indexOf('=', start);
    if (eq == -1) break;
    
    String key = cmd.substring(start, eq);
    int valueEnd = cmd.indexOf(',', eq+1);
    if (valueEnd == -1) valueEnd = cmd.length();
    
    String val = cmd.substring(eq+1, valueEnd);
    
    if (val.length() > 0) {
      if (key == "speedx") {
        config.speedX = val.toInt();
                ikak.setSpeed(config.speedX, config.speedY, config.speedR);

      }
      else if (key == "speedy") {
        config.speedY = val.toInt();
        Serial.println("Set speedY: " + String(config.speedY));
        ikak.setSpeed(config.speedX, config.speedY, config.speedR);
      }
      else if (key == "speedr") {
        config.speedR = val.toInt();
        Serial.println("Set speedR: " + String(config.speedR));
        ikak.setSpeed(config.speedX, config.speedY, config.speedR);
      }
      else if (key == "gait") {
        config.gaitEnabled = (val.toInt() == 1);
        Serial.println("Set gait: " + String(config.gaitEnabled));
        ikak.setGaitEnable(config.gaitEnabled);
      }
      else if (key == "bodyposx") {
        config.bodyPosX = val.toInt();
        ikak.setBodyPos(config.bodyPosX, config.bodyPosY, config.bodyPosZ);
      }
      else if (key == "bodyposy") {
        config.bodyPosY = val.toInt();
        ikak.setBodyPos(config.bodyPosX, config.bodyPosY, config.bodyPosZ);
      }
      else if (key == "bodyposz") {
        config.bodyPosZ = val.toInt();
        ikak.setBodyPos(config.bodyPosX, config.bodyPosY, config.bodyPosZ);
      }
      else if (key == "bodyrotx") {
        config.bodyRotX = val.toInt();
        ikak.setBodyRot(config.bodyRotX, config.bodyRotY, config.bodyRotZ);
      }
      else if (key == "bodyroty") {
        config.bodyRotY = val.toInt();
        ikak.setBodyRot(config.bodyRotX, config.bodyRotY, config.bodyRotZ);
      }
      else if (key == "bodyrotz") {
        config.bodyRotZ = val.toInt();
        ikak.setBodyRot(config.bodyRotX, config.bodyRotY, config.bodyRotZ);
      }
      else if (key == "initx") {
        config.initX = val.toInt();
        ikak.initializeLegs(config.initX, config.initY, config.initZ, config.liftHeight);
      } else if(key == "inity") 
      {
        config.initY = val.toInt();
        ikak.initializeLegs(config.initX, config.initY, config.initZ, config.liftHeight);
      } else if(key == "initz") {
        config.initZ = val.toInt();
        ikak.initializeLegs(config.initX, config.initY, config.initZ, config.liftHeight);
      } else if(key == "liftheight") {
        config.liftHeight = val.toInt();
        ikak.initializeLegs(config.initX, config.initY, config.initZ, config.liftHeight);
      }
      else if (key == "frequency") {
        uint16_t newFreq = val.toInt();
        // Limit frequency to reasonable values (1-100 Hz)
        if (newFreq >= 1 && newFreq <= 100) {
          config.updateFrequency = newFreq;
          Serial.println("Set update frequency: " + String(config.updateFrequency));
        }
      }
    }
    
    start = valueEnd + 1;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check WiFi connection
  reconnectWiFi();
  
  // Process UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
      String cmd = String(packetBuffer);
      cmd.trim();
      
      Serial.println("Received: " + cmd);
      processCommand(cmd);
    }
  }
  
  // Calculate the interval based on the configured frequency
  unsigned long interval = 1000 / config.updateFrequency; // Convert Hz to milliseconds
  
  // Run hexapod control at the configured frequency using millis()
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ikak.run();
  }
}