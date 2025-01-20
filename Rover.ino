#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

namespace MotorControl {

// Pin Definitions
constexpr int MOTOR_LEFT_FWD = 18;
constexpr int MOTOR_LEFT_BWD = 19;
constexpr int MOTOR_LEFT_EN = 32;
constexpr int MOTOR_RIGHT_FWD = 22;
constexpr int MOTOR_RIGHT_BWD = 23;
constexpr int MOTOR_RIGHT_EN = 33;
constexpr int DHT_PIN = 21;
constexpr int MQ135_PIN = 34;
constexpr int TRIG_PIN = 13;
constexpr int ECHO_PIN = 14;
constexpr int WARNING_LED = 27;
constexpr int LDR_PIN = 35;
constexpr int HEADLIGHT_PIN = 25;
constexpr int CONN_LED = 2;

// Constants
constexpr int OBSTACLE_THRESHOLD = 15;
constexpr int BLINK_INTERVAL = 500;
constexpr int ULTRASONIC_TIMEOUT = 30000;
constexpr int LDR_THRESHOLD = 60;
constexpr int LEFT_MOTOR_SPEED = 255;
constexpr int RIGHT_MOTOR_SPEED = 240;

enum class Direction : uint8_t {
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right
};

// Command types
enum class CommandType : uint8_t {
    Movement = 0,
    CheckConnection
};

struct SensorData {
    float temperature;
    float humidity;
    int airQuality;
    bool obstacle;
    int lightLevel;
};

// Command data structure
struct CommandData {
    CommandType type;
    Direction direction;
};

class Controller {
public:
    Controller() : dht(DHT_PIN, DHT11), lastBlinkTime(0), ledState(false), forwardCommandReceived(false) {}

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
        pinMode(MOTOR_LEFT_EN, OUTPUT);
        pinMode(MOTOR_RIGHT_EN, OUTPUT);

        stopMotor(MOTOR_LEFT_EN);
        stopMotor(MOTOR_RIGHT_EN);

        dht.begin();

        // Initialize ESP-NOW
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK) {
            Serial.println("ESP-NOW init failed");
            return;
        }
        esp_now_register_recv_cb(onDataReceived);

        // Add peer
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
        readSensors();
        handleWarningLED();
        handleHeadlights();
        //print sensor data
        Serial.printf("Temperature: %.2f\n", sensorData.temperature);
        Serial.printf("Humidity: %.2f\n", sensorData.humidity);
        Serial.printf("Air Quality: %d\n", sensorData.airQuality);
        Serial.printf("Obstacle: %s\n", sensorData.obstacle ? "Yes" : "No");
        Serial.printf("Light Level: %d\n", sensorData.lightLevel);
    }

private:
    static void onDataReceived(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
        memcpy(remoteAddress, esp_now_info->src_addr, 6);

        if (data_len == sizeof(CommandData)) {
            CommandData command;
            memcpy(&command, data, sizeof(command));

            if (command.type == CommandType::CheckConnection) {
                //Turn on LED
                digitalWrite(CONN_LED, HIGH);
                return;
            }

            instance().processCommand(command.direction);
        }
    }

    void readSensors() {
        static unsigned long lastSendTime = 0;
        constexpr unsigned long DELAY = 20;

        if (millis() - lastSendTime >= DELAY) {
            checkObstacles();
            readDHT11();
            readMQ135();
            readLDR();
            lastSendTime = millis();

            esp_err_t result = esp_now_send(remoteAddress, reinterpret_cast<uint8_t*>(&sensorData), sizeof(sensorData));
            if (result != ESP_OK) {
                Serial.printf("Error sending data: %d\n", result);
            }
        }
    }

    void processCommand(Direction direction) {
        forwardCommandReceived = (direction == Direction::Forward);

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
                moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, false, LEFT_MOTOR_SPEED);
                moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, true, RIGHT_MOTOR_SPEED);
                break;
            case Direction::Right:
                moveMotor(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_LEFT_EN, true, LEFT_MOTOR_SPEED);
                moveMotor(MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_EN, false, RIGHT_MOTOR_SPEED);
                break;
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

    void checkObstacles() {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
        float distance = duration * 0.034 / 2;
        Serial.printf("Distance: %.2f\n", distance);
        sensorData.obstacle = (distance > 0 && distance < OBSTACLE_THRESHOLD);
    }

    void handleWarningLED() {
        if (sensorData.obstacle) {
            if (forwardCommandReceived) {
                if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
                    ledState = !ledState;
                    digitalWrite(WARNING_LED, ledState);
                    lastBlinkTime = millis();
                }
            } else {
                digitalWrite(WARNING_LED, HIGH);
            }
        } else {
            digitalWrite(WARNING_LED, LOW);
        }
    }

    void handleHeadlights() {
        int lightLevel = analogRead(LDR_PIN);
        digitalWrite(HEADLIGHT_PIN, lightLevel < LDR_THRESHOLD ? HIGH : LOW);
    }

    void moveMotor(int pin1, int pin2, int enablePin, bool forward, int speed) {
        digitalWrite(pin1, forward ? HIGH : LOW);
        digitalWrite(pin2, forward ? LOW : HIGH);
        analogWrite(enablePin, speed);
    }

    void stopMotor(int enablePin) {
        analogWrite(enablePin, 0);
    }

    static Controller& instance() {
        static Controller instance;
        return instance;
    }

    DHT dht;
    SensorData sensorData;
    unsigned long lastBlinkTime;
    bool ledState;
    bool forwardCommandReceived;
    static uint8_t remoteAddress[6];
};

uint8_t Controller::remoteAddress[] = {0xF8, 0xB3, 0xB7, 0x20, 0x35, 0x08};

} // namespace MotorControl

MotorControl::Controller controller;

void setup() {
    controller.setup();
}

void loop() {
    controller.loop();
}
