#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

// Pin Definitions
#define MOTOR_LEFT_FWD    18  // L293D pin 2
#define MOTOR_LEFT_BWD    19  // L293D pin 7
#define MOTOR_LEFT_EN     32  // L293D pin 1
#define MOTOR_RIGHT_FWD   22  // L293D pin 10
#define MOTOR_RIGHT_BWD   23  // L293D pin 15
#define MOTOR_RIGHT_EN    33  // L293D pin 9
#define DHT_PIN           21  // DHT11 data pin
#define MQ135_PIN         34  // MQ135 analog pin
#define TRIG_PIN          13  // HC-SR04 trigger
#define ECHO_PIN          14  // HC-SR04 echo
#define WARNING_LED       27  // Yellow LED for obstacle warning
#define LDR_PIN           35  // LDR analog pin
#define HEADLIGHT_PIN     25  // Headlight LED pin

// Constants
#define OBSTACLE_THRESHOLD 15   // Distance in cm
#define DHT_TYPE DHT11
#define BLINK_INTERVAL 500      // LED blink interval in ms
#define ULTRASONIC_TIMEOUT 30000  // Microseconds
#define LDR_THRESHOLD 30      // LDR threshold (adjust based on calibration)

#define LEFT_MOTOR_SPEED  255 // Left motor speed
#define RIGHT_MOTOR_SPEED 140 // Right motor speed

// Direction enumeration
enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Global variables
DHT dht(DHT_PIN, DHT_TYPE);
unsigned long lastBlinkTime = 0;
bool ledState = false;
bool forwardCommandReceived = false;

// Hard-coded remote MAC address
uint8_t remoteAddress[] = {0xF8, 0xB3, 0xB7, 0x20, 0x35, 0x08};

// Structure for sending sensor data
struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
    int lightLevel;
};

// Structure for receiving commands
struct CommandData {
    Direction direction;
};

SensorData sensorData; // global variable to store sensor data

// Command callback function
void onDataReceived(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    memcpy(remoteAddress, esp_now_info->src_addr, 6);

    if (data_len == sizeof(CommandData)) {
        CommandData command;
        memcpy(&command, data, sizeof(command));
        processCommand(command.direction);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize pins
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_BWD, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_BWD, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(WARNING_LED, OUTPUT);
    pinMode(MQ135_PIN, INPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(HEADLIGHT_PIN, OUTPUT);

    //Enable pins
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    // Stop motors initially
    stopMotor(MOTOR_LEFT_EN);
    stopMotor(MOTOR_RIGHT_EN);

    // Initialize DHT sensor
    dht.begin();

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onDataReceived);

    // Add peer with the hard-coded MAC address
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, remoteAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer added successfully");
    } else {
        Serial.println("Failed to add peer");
    }
}

void loop() {
    // Read sensors
    readSensors();
    // Handle warning LED
    handleWarningLED();
    // Handle headlights
    handleHeadlights();

    //print CPU temperature
    Serial.printf("CPU Temp: %.2f C\n", temperatureRead());
}

void readDHT11() {
    sensorData.temperature = dht.readTemperature();
    sensorData.humidity = dht.readHumidity();
}

void readMQ135() {
    sensorData.airQuality = analogRead(MQ135_PIN);
}

void readLDR() {
    sensorData.lightLevel = analogRead(LDR_PIN);
}

void readSensors() {

    static unsigned long lastSendTime = 0;
    const unsigned long DELAY = 20;

    esp_err_t result = esp_now_send(remoteAddress, (uint8_t *)&sensorData, sizeof(sensorData));
    if (result != ESP_OK) {
        Serial.printf("Error sending data: %d\n", result);
    }

    if (millis() - lastSendTime >= DELAY) {

        checkObstacles();
        readDHT11();
        readMQ135();
        readLDR();

        lastSendTime = millis();
    }
    // Serial.printf("Temperature: %.2f C, Humidity: %.2f%%, Air Quality: %d, Light Level: %d, Obstacle: %s\n",
    //         sensorData.temperature, sensorData.humidity, sensorData.airQuality, sensorData.lightLevel,
    //         sensorData.obstacle ? "Yes" : "No");
}

void checkObstacles() {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
    float distance = duration * 0.034 / 2;

    // Update obstacle status
    sensorData.obstacle = (distance > 0 && distance < OBSTACLE_THRESHOLD);

    Serial.printf("Duration: %ld, Distance: %.2f cm\n", duration, distance);
}


void handleWarningLED() {
    if (sensorData.obstacle) {
        if (forwardCommandReceived) {
            // Blink LED
            if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
                ledState = !ledState;
                digitalWrite(WARNING_LED, ledState);
                lastBlinkTime = millis();
            }
        } else {
            // Solid LED
            digitalWrite(WARNING_LED, HIGH);
        }
    } else {
        digitalWrite(WARNING_LED, LOW);
    }
}

void handleHeadlights() {
    int lightLevel = analogRead(LDR_PIN);
    // Turn on headlights if light level is below threshold
    if (lightLevel < LDR_THRESHOLD) {
        digitalWrite(HEADLIGHT_PIN, HIGH);
    } else {
        digitalWrite(HEADLIGHT_PIN, LOW);
    }
}

void processCommand(Direction direction) {
    forwardCommandReceived = (direction == Direction::Forward);

    if (sensorData.obstacle && direction == Direction::Forward) {
        // Stop if obstacle detected
        stopMotor(MOTOR_LEFT_EN);
        stopMotor(MOTOR_RIGHT_EN);
        return;
    }

    switch (direction) {
        case Direction::Stop:
            stopMotor(MOTOR_LEFT_EN);
            stopMotor(MOTOR_RIGHT_EN);
            break;
        case Direction::Forward:
            moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, true, LEFT_MOTOR_SPEED);
            moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, true, RIGHT_MOTOR_SPEED);
            break;
        case Direction::Backward:
            moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, false, LEFT_MOTOR_SPEED);
            moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, false, RIGHT_MOTOR_SPEED);
            break;
        case Direction::Left:
            moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, false, LEFT_MOTOR_SPEED);
            moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, true, RIGHT_MOTOR_SPEED);
            break;
        case Direction::Right:
            moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, true, LEFT_MOTOR_SPEED);
            moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, false, RIGHT_MOTOR_SPEED);
            break;
    }

    //Serial.printf("Command: %d\n", static_cast<int>(direction));
}

// Function to control the motors
void moveMotor(int pin1, int pin2, int enablePin, bool forward, int speed) {
  digitalWrite(pin1, forward ? HIGH : LOW);
  digitalWrite(pin2, forward ? LOW : HIGH);
  analogWrite(enablePin, speed);
}

// Function to stop a motor
void stopMotor(int enablePin) {
  analogWrite(enablePin, 0);
}