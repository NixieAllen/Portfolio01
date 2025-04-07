#include <WiFi.h>
#include <FirebaseESP32.h>

// Firebase credentials
#define WIFI_SSID "" // put WIFI SSID here
#define WIFI_PASSWORD "" // put WIFI Password here, same with your laptop
#define API_KEY "" //put the API key for thinkspeak here for data analytics via cloud
#define DATABASE_URL ""// Put the firebase url here for your database
#define USER_EMAIL "" //user email used
#define USER_PASSWORD "" password used

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Pin definitions
#define DUST_MEASURE_PIN 34  
#define DUST_LED_POWER_PIN 13
#define DUST_THRESHOLD 300

#define MQ2_ANALOG_PIN 35
#define MQ2_THRESHOLD 300

#define MQ135_ANALOG_PIN 32
#define MQ135_THRESHOLD 400

#define MOTION_SENSOR_PIN 4

#define MOTOR_PIN_ENA 18
#define MOTOR_PIN_IN1 19
#define MOTOR_PIN_IN2 21

#define LED_LOW_PIN 25
#define LED_HIGH_PIN 27
#define LED_OFF_PIN 26

// Variables for dust sensor
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

// Timer variables for sensor alerts
unsigned long mq2AlertStartTime = 0;
unsigned long dustAlertStartTime = 0;
unsigned long mq135AlertStartTime = 0;
const unsigned long alertDelay = 3000;

// Timer variables for fan control
unsigned long lastSensorTime = 0;
unsigned long lastMotionTime = 0;
const unsigned long additionalRunTime = 5000;
bool fanRunning = false;
bool fanWasHigh = false;

void setup() {
    Serial.begin(9600);
    setupPins();
    connectToWiFi();
    initializeFirebase();
    warmUpSensors();
}

void loop() {
    int mq2Value = analogRead(MQ2_ANALOG_PIN);
    int motionDetected = digitalRead(MOTION_SENSOR_PIN);
    readDustSensor();
    int mq135Value = analogRead(MQ135_ANALOG_PIN);

    // Debugging output
    printSensorValues(mq2Value, mq135Value);

    // Check sensor thresholds and manage alerts
    manageAlerts(mq2Value, dustDensity, mq135Value);

    // Control fan based on sensors and motion
    bool aboveThreshold = (mq2Value > MQ2_THRESHOLD || dustDensity > DUST_THRESHOLD || mq135Value > MQ135_THRESHOLD);
    controlFan(aboveThreshold, motionDetected);

    // Send Data to Firebase
    sendDataToFirebase(mq2Value, motionDetected, mq135Value);

    delay(1000); // Delay for readability
}

void manageAlerts(int mq2Value, float dustDensity, int mq135Value) {
    // Manage MQ2 Alert
    if (mq2Value > MQ2_THRESHOLD) {
        if (mq2AlertStartTime == 0) {
            Serial.println("MQ2 Alert condition met.");
            mq2AlertStartTime = millis();
        } else if (millis() - mq2AlertStartTime >= alertDelay) {
            sendAlertToFirebase("MQ2 value exceeded threshold!");
            mq2AlertStartTime = 0;
        }
    } else {
        mq2AlertStartTime = 0;
    }

    // Manage Dust Density Alert
    if (dustDensity > DUST_THRESHOLD) {
        if (dustAlertStartTime == 0) {
            Serial.println("Dust Density Alert condition met.");
            dustAlertStartTime = millis();
        } else if (millis() - dustAlertStartTime >= alertDelay) {
            sendAlertToFirebase("Dust density exceeded threshold!");
            dustAlertStartTime = 0;
        }
    } else {
        dustAlertStartTime = 0;
    }

    // Manage MQ135 Alert
    if (mq135Value > MQ135_THRESHOLD) {
        if (mq135AlertStartTime == 0) {
            Serial.println("MQ135 Alert condition met.");
            mq135AlertStartTime = millis();
        } else if (millis() - mq135AlertStartTime >= alertDelay) {
            sendAlertToFirebase("MQ135 value exceeded threshold!");
            mq135AlertStartTime = 0;
        }
    } else {
        mq135AlertStartTime = 0;
    }
}

void setupPins() {
    pinMode(MOTOR_PIN_ENA, OUTPUT);
    pinMode(MOTOR_PIN_IN1, OUTPUT);
    pinMode(MOTOR_PIN_IN2, OUTPUT);
    pinMode(MOTION_SENSOR_PIN, INPUT);
    pinMode(DUST_LED_POWER_PIN, OUTPUT);
    pinMode(MQ135_ANALOG_PIN, INPUT);
    pinMode(MQ2_ANALOG_PIN, INPUT);
    pinMode(LED_LOW_PIN, OUTPUT);
    pinMode(LED_HIGH_PIN, OUTPUT);
    pinMode(LED_OFF_PIN, OUTPUT);

    digitalWrite(MOTOR_PIN_IN1, LOW);
    digitalWrite(MOTOR_PIN_IN2, LOW);
    digitalWrite(LED_LOW_PIN, LOW);
    digitalWrite(LED_HIGH_PIN, LOW);
    digitalWrite(LED_OFF_PIN, HIGH);
}

void connectToWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
}

void initializeFirebase() {
    config.api_key = API_KEY;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.database_url = DATABASE_URL;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

void warmUpSensors() {
    Serial.println("Sensors warm-up for 30 seconds.");
    delay(5000);
}

void readDustSensor() {
    digitalWrite(DUST_LED_POWER_PIN, LOW);
    delayMicroseconds(samplingTime);
    voMeasured = analogRead(DUST_MEASURE_PIN);
    delayMicroseconds(deltaTime);
    digitalWrite(DUST_LED_POWER_PIN, HIGH);
    delayMicroseconds(sleepTime);

    calcVoltage = voMeasured * (3.3 / 4095.0);
    dustDensity = 170 * calcVoltage - 0.1;
}

void printSensorValues(int mq2Value, int mq135Value) {
    Serial.print("MQ135 Raw Value: ");
    Serial.println(mq135Value);
    Serial.print("MQ2 Value: ");
    Serial.println(mq2Value);
    Serial.print("Dust Density: ");
    Serial.println(dustDensity);
}

void controlFan(bool aboveThreshold, int motionDetected) {
    if (aboveThreshold) {
        lastSensorTime = millis();
        setFanHighSpeed();
    } else if (motionDetected == HIGH) {
        lastMotionTime = millis();
        setFanLowSpeed();
    }
    checkFanStatus();
}

void setFanHighSpeed() {
    if (!fanRunning || (fanRunning && !fanWasHigh)) {
        Serial.println("Sensors above threshold: Setting fan speed to HIGH.");
        analogWrite(MOTOR_PIN_ENA, 255); // High speed
        digitalWrite(MOTOR_PIN_IN1, HIGH);
        digitalWrite(MOTOR_PIN_IN2, LOW);
        digitalWrite(LED_LOW_PIN, LOW);
        digitalWrite(LED_HIGH_PIN, HIGH);
        digitalWrite(LED_OFF_PIN, LOW);
        fanRunning = true;
        fanWasHigh = true;
    }
}

void setFanLowSpeed() {
    if (!fanRunning || (fanRunning && fanWasHigh)) {
        Serial.println("Motion detected: Setting fan speed to LOW.");
        analogWrite(MOTOR_PIN_ENA, 128); // Low speed
        digitalWrite(MOTOR_PIN_IN1, HIGH);
        digitalWrite(MOTOR_PIN_IN2, LOW);
        digitalWrite(LED_LOW_PIN, HIGH);
        digitalWrite(LED_HIGH_PIN, LOW);
        digitalWrite(LED_OFF_PIN, LOW);
        fanRunning = true; 
        fanWasHigh = false; 
    }
}

void checkFanStatus() {
    if (fanRunning) {
        if (millis() - lastSensorTime >= additionalRunTime && 
            millis() - lastMotionTime >= additionalRunTime) {
            Serial.println("Turning off the fan.");
            analogWrite(MOTOR_PIN_ENA, 0);
            digitalWrite(MOTOR_PIN_IN1, LOW);
            digitalWrite(MOTOR_PIN_IN2, LOW);
            digitalWrite(LED_LOW_PIN, LOW);
            digitalWrite(LED_HIGH_PIN, LOW);
            digitalWrite(LED_OFF_PIN, HIGH);
            fanRunning = false; 
            fanWasHigh = false;
        }
    }
}

void sendAlertToFirebase(const char* message) {
    String alertPath = "/alerts";
    FirebaseJson alertJson;
    alertJson.set("alert", message); 

    if (Firebase.pushJSON(firebaseData, alertPath, alertJson)) {
        Serial.println("Alert sent to Firebase successfully.");
    } else {
        Serial.print("Failed to send alert to Firebase: ");
        Serial.println(firebaseData.errorReason());
    }
}

void sendDataToFirebase(int mq2Value, int motionDetected, int mq135Value) {
    String path = "/sensorData";
    FirebaseJson json;

    json.set("fanStatus", fanRunning ? (fanWasHigh ? "HIGH" : "LOW") : "OFF");
    json.set("mq2Value", mq2Value);
    json.set("motionDetected", motionDetected);
    json.set("dustDensity", dustDensity);
    json.set("mq135Value", mq135Value);

    if (Firebase.setJSON(firebaseData, path, json)) {
        Serial.println("Data sent to Firebase successfully.");
    } else {
        Serial.print("Failed to send data to Firebase: ");
        Serial.println(firebaseData.errorReason());
    }
}
