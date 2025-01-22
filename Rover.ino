#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>
#include <map>

// Pin Definitions
#define MOTOR_LEFT_FWD    18
#define MOTOR_LEFT_BWD    19
#define MOTOR_LEFT_EN     32
#define MOTOR_RIGHT_FWD   22
#define MOTOR_RIGHT_BWD   23
#define MOTOR_RIGHT_EN    33
#define DHT_PIN           21
#define MQ135_PIN         34
#define TRIG_PIN          13
#define ECHO_PIN          14
#define LDR_PIN           35
#define WARNING_LED       27
#define HEADLIGHT_PIN     25
#define MOVEMENT_STATUS_LED 2

#define OBSTACLE_THRESHOLD 15
#define DHT_TYPE DHT11
#define ULTRASONIC_TIMEOUT 30000
#define LDR_THRESHOLD 500
#define LEFT_MOTOR_SPEED  255
#define RIGHT_MOTOR_SPEED 255

enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

DHT dht(DHT_PIN, DHT_TYPE);
Direction currentDirection = Direction::Stop;

// Hard-coded remote MAC address
uint8_t remoteAddress[] = {0xF8, 0xB3, 0xB7, 0x20, 0x35, 0x08};

struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
    int lightLevel;
};

struct CommandData {
    Direction direction;
};

SensorData sensorData;

struct LEDControl {
    unsigned long lastToggleTime;
    bool blink;
    bool state;
};

std::map<uint8_t, LEDControl> ledMap;

void onDataReceived(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    memcpy(remoteAddress, esp_now_info->src_addr, 6);
    if (data_len == sizeof(CommandData)) {
        CommandData command;
        memcpy(&command, data, sizeof(command));
        processCommand(command.direction);
    }
}


void handleLED(uint8_t pin, bool condition, bool blink, std::pair<unsigned long, unsigned long> pattern = {500, 500}) {
    unsigned long currentMillis = millis();
    auto &led = ledMap[pin];

    if (condition) {
        if (blink) {
            unsigned long interval = led.state ? pattern.first : pattern.second; // ON duration or OFF duration
            if (currentMillis - led.lastToggleTime >= interval) {
                led.state = !led.state;
                digitalWrite(pin, led.state);
                led.lastToggleTime = currentMillis;
            }
        } else {
            digitalWrite(pin, HIGH);
            led.state = true;
        }
    } else {
        digitalWrite(pin, LOW);
        led.state = false;
    }
}

void readDHT11() {
    sensorData.temperature = dht.readTemperature() + 20;
    sensorData.humidity = dht.readHumidity() + 40;
}

void readMQ135() {
    sensorData.airQuality = analogRead(MQ135_PIN);
}

void readLDR() {
    sensorData.lightLevel = analogRead(LDR_PIN);
}

void readSensors() {

    checkObstacles();

    static unsigned long lastSensorReadTime = 0;
    if (millis() - lastSensorReadTime < 500) return;

    readDHT11();
    readMQ135();
    readLDR();

    esp_err_t result = esp_now_send(remoteAddress, (uint8_t *)&sensorData, sizeof(sensorData));
    if (result == ESP_OK) {
        Serial.println("Data sent successfully");
    } else {
        Serial.printf("Error sending data: %d\n", result);
    }

    Serial.printf("Temperature: %.2f C, Humidity: %.2f%%, Air Quality: %d, Light Level: %d, Obstacle: %s\n",
            sensorData.temperature, sensorData.humidity, sensorData.airQuality, sensorData.lightLevel,
            sensorData.obstacle ? "Yes" : "No");
    lastSensorReadTime = millis();
}

void checkObstacles() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
    float distance = duration * 0.034 / 2;

    sensorData.obstacle = (distance > 0 && distance < OBSTACLE_THRESHOLD);

    if (sensorData.obstacle) {
        if (currentDirection == Direction::Forward) {
            handleLED(WARNING_LED, true, true);
        } else {
            handleLED(WARNING_LED, true, false);
        }
    } else {
        handleLED(WARNING_LED, false, false);
    }
}

void handleHeadlights() {
    int lightLevel = analogRead(LDR_PIN);
    digitalWrite(HEADLIGHT_PIN, lightLevel < LDR_THRESHOLD ? HIGH : LOW);
}

void processCommand(Direction direction) {
    currentDirection = direction;
    if (sensorData.obstacle && direction == Direction::Forward) {
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
            moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, true, LEFT_MOTOR_SPEED);
            moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, false, RIGHT_MOTOR_SPEED);
            break;
        case Direction::Right:
            moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, false, LEFT_MOTOR_SPEED);
            moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, true, RIGHT_MOTOR_SPEED);
            break;
    }
}

void moveMotor(int pin1, int pin2, int enablePin, bool forward, int speed) {
    digitalWrite(pin1, forward ? HIGH : LOW);
    digitalWrite(pin2, forward ? LOW : HIGH);
    analogWrite(enablePin, speed);
}

void stopMotor(int enablePin) {
    analogWrite(enablePin, 0);
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
    pinMode(MOVEMENT_STATUS_LED, OUTPUT);
    pinMode(MQ135_PIN, INPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(HEADLIGHT_PIN, OUTPUT);

    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_EN, OUTPUT);

    stopMotor(MOTOR_LEFT_EN);
    stopMotor(MOTOR_RIGHT_EN);

    dht.begin();

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onDataReceived);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, remoteAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer added successfully");
    } else {
        Serial.println("Failed to add peer");
    }

    // Initialize LEDs in the map
    ledMap[WARNING_LED] = {0, false, false};
    ledMap[MOVEMENT_STATUS_LED] = {0, false, false};
}



void loop() {
    
    readSensors();

    handleLED(MOVEMENT_STATUS_LED, true, currentDirection != Direction::Stop, {100, 800});

    handleHeadlights();
}