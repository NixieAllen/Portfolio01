#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ESP32Servo.h>
#include "DHT.h"
#include "addons/TokenHelper.h"

// WiFi and Firebase Configuration
#define WIFI_SSID "S213 Wi-Fi"
#define WIFI_PASSWORD "L94G9FBU"
#define API_KEY "AIzaSyAMgoWNNPCOkMdT_uifafK-mAkVvfF1EgU"
#define USER_EMAIL "user@gmail.com"
#define USER_PASSWORD "User-123"
#define DATABASE_URL "https://puresense-d33c0-default-rtdb.firebaseio.com"

// Pin Definitions
#define DHT_PIN 4          
#define HUMIDITY_SERVO_PIN 13  
#define PIR_PIN 2          
#define PIR_RED_PIN 16     
#define PIR_GREEN_PIN 17   
#define ULTRASONIC_RED_PIN 25
#define ULTRASONIC_GREEN_PIN 26 
#define ULTRASONIC_BLUE_PIN 27  
#define TCS_S0_PIN 5       
#define TCS_S1_PIN 18      
#define TCS_S2_PIN 19      
#define TCS_S3_PIN 21      
#define TCS_OUT_PIN 22     
#define COLOR_SERVO_PIN 15 
#define TRIG_PIN 12        
#define ECHO_PIN 14        
#define MQ135_PIN 34       

// Sensor Configuration
#define DHTTYPE DHT22
#define HUMIDITY_THRESHOLD_LOW 30
#define MOTION_COOLDOWN 100
#define PIR_DETECTION_THRESHOLD 5

// Global Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
Servo humidityServo;
Servo colorServo;
DHT dht(DHT_PIN, DHTTYPE);

// Timing Variables
unsigned long currentMillis;
unsigned long previousFirebaseUpdate = 0;
const unsigned long FIREBASE_UPDATE_INTERVAL = 50;
unsigned long lastPirDetectionTime = 0;
const unsigned long PIR_COOLDOWN = 100;
unsigned long lastMotionTime = 0;
unsigned long motionDetectionDelay = 50;
bool wasMotionPreviously = false;

// State Variables
bool humidityServoActive = false;
bool colorServoActive = false;
bool motionDetected = false;
int pirCounter = 0;

// Color sensor readings
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void initWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");
}

int readColorFrequency(bool s2State, bool s3State) {
    digitalWrite(TCS_S2_PIN, s2State);
    digitalWrite(TCS_S3_PIN, s3State);
    delayMicroseconds(20);
    return pulseIn(TCS_OUT_PIN, LOW);
}


float getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return pulseIn(ECHO_PIN, HIGH);
}

void handleHumidity() {
    float humidity = dht.readHumidity();
    
    // Check if the humidity reading is valid
    if (!isnan(humidity)) {
        // If humidity is below the threshold
        if (humidity < HUMIDITY_THRESHOLD_LOW) {
            // Activate servo only if it's not already active
            if (!humidityServoActive) {
                humidityServo.write(45); // Rotate to 90 degrees (adjust as needed)
                humidityServoActive = true; // Set the active flag
                delay(15); // Allow time for the servo to move
                Serial.println("Humidity low - activating servo");
            }
        } else {
            // If humidity is above the threshold and servo is active
            if (humidityServoActive) {
                humidityServo.write(0); // Return to 0 degrees
                humidityServoActive = false; // Reset the active flag
                delay(15); // Allow time for the servo to move
                Serial.println("Humidity normal - deactivating servo");
            }
        }
    } else {
        Serial.println("Failed to read humidity");
    }
}

void setup() {
    Serial.begin(921600);
    
    // Initialize sensors and pins
    dht.begin();
    humidityServo.attach(HUMIDITY_SERVO_PIN);
    colorServo.attach(COLOR_SERVO_PIN);
    
    pinMode(PIR_PIN, INPUT);
    pinMode(PIR_RED_PIN, OUTPUT);
    pinMode(PIR_GREEN_PIN, OUTPUT);
    pinMode(ULTRASONIC_RED_PIN, OUTPUT);
    pinMode(ULTRASONIC_GREEN_PIN, OUTPUT);
    pinMode(ULTRASONIC_BLUE_PIN, OUTPUT);
    pinMode(TCS_S0_PIN, OUTPUT);
    pinMode(TCS_S1_PIN, OUTPUT);
    pinMode(TCS_S2_PIN, OUTPUT);
    pinMode(TCS_S3_PIN, OUTPUT);
    pinMode(TCS_OUT_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(MQ135_PIN, INPUT);
    
    // Initialize servos to 0 position
    humidityServo.write(0);
    colorServo.write(0);
    
    // Set frequency scaling to 20%
    digitalWrite(TCS_S0_PIN, HIGH);
    digitalWrite(TCS_S1_PIN, LOW);
    
    digitalWrite(PIR_RED_PIN, LOW);
    digitalWrite(PIR_GREEN_PIN, HIGH);

    digitalWrite(ULTRASONIC_RED_PIN, LOW);
    digitalWrite(ULTRASONIC_GREEN_PIN, LOW);
    digitalWrite(ULTRASONIC_BLUE_PIN, LOW);
    
    initWiFi();
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.database_url = DATABASE_URL;
    
    Firebase.reconnectWiFi(true);
    Firebase.begin(&config, &auth);
}

void updateFirebase() {
    if (currentMillis - previousFirebaseUpdate >= FIREBASE_UPDATE_INTERVAL) {
        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        float distance = getDistance();
        float airQuality = analogRead(MQ135_PIN);
        
        if (!isnan(humidity) && !isnan(temperature)) {
            Firebase.RTDB.setFloat(&fbdo, "Sensors/humidity", humidity);
            Firebase.RTDB.setFloat(&fbdo, "Sensors/temperature", temperature);
            Firebase.RTDB.setFloat(&fbdo, "Sensors/paperLevel", distance);
            Firebase.RTDB.setFloat(&fbdo, "Sensors/airQuality", airQuality);
            Firebase.RTDB.setBool(&fbdo, "Sensors/occupancy", motionDetected);
            Firebase.RTDB.setInt(&fbdo, "Sensors/color/red", redFrequency);
            Firebase.RTDB.setInt(&fbdo, "Sensors/color/green", greenFrequency);
            Firebase.RTDB.setInt(&fbdo, "Sensors/color/blue", blueFrequency);
        }
        previousFirebaseUpdate = currentMillis;
    }
}

void handlePIR() {
    int pirValue = digitalRead(PIR_PIN);
    unsigned long currentTime = millis();

    // Motion detection logic
    if (pirValue == HIGH) {
        // When motion is detected
        if (!wasMotionPreviously) {
            digitalWrite(PIR_RED_PIN, HIGH);
            digitalWrite(PIR_GREEN_PIN, LOW);
            motionDetected = true;
            wasMotionPreviously = true;
            lastMotionTime = currentTime;
            Serial.println("Motion detected!");
        }
    } else {
        // When no motion is detected
        if (wasMotionPreviously) {
            // Add a small delay to prevent quick toggles
            if (currentTime - lastMotionTime > motionDetectionDelay) {
                digitalWrite(PIR_RED_PIN, LOW);
                digitalWrite(PIR_GREEN_PIN, HIGH);
                motionDetected = false;
                wasMotionPreviously = false;
                Serial.println("No motion - area vacant");
            }
        }
    }
}

void loop() {
    currentMillis = millis();
    
    // Handle Motion Detection
    handlePIR();
    // Handle Humidity Control
    handleHumidity();
    
    // Color Detection
     redFrequency = readColorFrequency(LOW, LOW);
    greenFrequency = readColorFrequency(HIGH, HIGH);
    blueFrequency = readColorFrequency(LOW, HIGH);

    // Activate servo only when blue frequency is above 200
    if (blueFrequency < 200 && !colorServoActive) {
        colorServo.write(120);
        colorServoActive = true;
        Serial.println("Blue frequency high - activating color servo");
    }
    
    // Reset color servo when blue frequency is below 200
    if (colorServoActive && blueFrequency >= 200) {
        colorServo.write(0);
        colorServoActive = false;
        Serial.println("Blue frequency low - resetting color servo");
    }
    
    // Distance Monitoring
float distance = getDistance();
    if (distance <= 20) {
        digitalWrite(ULTRASONIC_RED_PIN, LOW);
        digitalWrite(ULTRASONIC_GREEN_PIN, HIGH);
        digitalWrite(ULTRASONIC_BLUE_PIN, LOW);
    } else if (distance <= 50) {
        digitalWrite(ULTRASONIC_RED_PIN, LOW);
        digitalWrite(ULTRASONIC_GREEN_PIN, LOW);
        digitalWrite(ULTRASONIC_BLUE_PIN, HIGH);
    } else if (distance > 1000) {
        digitalWrite(ULTRASONIC_RED_PIN, LOW);
        digitalWrite(ULTRASONIC_GREEN_PIN, HIGH);
        digitalWrite(ULTRASONIC_BLUE_PIN, LOW);
    } else {
        digitalWrite(ULTRASONIC_RED_PIN, HIGH);
        digitalWrite(ULTRASONIC_GREEN_PIN, LOW);
        digitalWrite(ULTRASONIC_BLUE_PIN, LOW);
    }
    
    // Firebase updates
    updateFirebase();
}
