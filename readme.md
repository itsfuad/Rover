# Rover Project

This README outlines the components, pin connections, and functionality of the rover system implemented using an ESP32 microcontroller.

---

## Components

1. **ESP32 Microcontroller** - Controls the rover's logic and communication.
2. **L293D Motor Driver** - Drives the left and right motors.
3. **DHT11 Sensor** - Measures temperature and humidity.
4. **MQ135 Gas Sensor** - Detects air quality.
5. **HC-SR04 Ultrasonic Sensor** - Measures distance to detect obstacles.
6. **LDR (Light Dependent Resistor)** - Measures ambient light levels.
7. **LEDs** - Includes a warning LED and headlights.

---

## Pin Connections

### Motor Driver (L293D)
- **Motor Left Forward**: GPIO 18 (L293D Pin 2)
- **Motor Left Backward**: GPIO 19 (L293D Pin 7)
- **Motor Left Enable**: GPIO 32 (L293D Pin 1)
- **Motor Right Forward**: GPIO 22 (L293D Pin 10)
- **Motor Right Backward**: GPIO 23 (L293D Pin 15)
- **Motor Right Enable**: GPIO 33 (L293D Pin 9)

### Sensors
- **DHT11 Sensor**: GPIO 21 (Data Pin)
- **MQ135 Gas Sensor**: GPIO 34 (Analog Input)
- **HC-SR04 Ultrasonic Sensor**:
  - Trigger: GPIO 13
  - Echo: GPIO 14
- **LDR (Light Dependent Resistor)**: GPIO 35 (Analog Input)

### LEDs
- **Warning LED**: GPIO 27
- **Headlight LED**: GPIO 25

---

## Constants
- **Obstacle Threshold**: 15 cm (for ultrasonic sensor)
- **Blink Interval**: 500 ms (for warning LED)
- **LDR Threshold**: 30 (adjust based on calibration)
- **Left Motor Speed**: 255 (maximum speed)
- **Right Motor Speed**: 140 (calibrated for straight movement)

---

## Functionality

### 1. **Sensor Data Collection**
The system periodically reads data from the DHT11, MQ135, LDR, and HC-SR04 sensors:
- **DHT11**: Measures temperature and humidity.
- **MQ135**: Reads air quality levels.
- **HC-SR04**: Detects obstacles by measuring distance.
- **LDR**: Detects ambient light levels to control headlights.

### 2. **Obstacle Detection**
- Uses HC-SR04 to measure distance.
- If an obstacle is detected within 15 cm, the rover stops moving.

### 3. **Warning LED Behavior**
- Blinks if an obstacle is detected while moving forward.
- Remains solid if the rover is stationary near an obstacle.

### 4. **Headlight Control**
- Turns on headlights if ambient light levels fall below the LDR threshold.

### 5. **Motor Control**
- Motors are controlled based on commands received (Forward, Backward, Left, Right, or Stop).

### 6. **ESP-NOW Communication**
- **Send Sensor Data**: Periodically sends sensor data to a paired remote device.
- **Receive Commands**: Processes movement commands from a paired remote device.

---

## How to Use

1. **Setup Connections**: Wire all components as per the pin connections listed above.
2. **Upload Code**: Flash the provided code to the ESP32 using Arduino IDE or PlatformIO.
3. **Calibrate Sensors**: Adjust thresholds for LDR and MQ135 as needed.
4. **Pair Devices**: Pair the ESP32 rover with the remote device using ESP-NOW.
5. **Test Functionality**: Verify motor movements, sensor readings, and communication.

---

## Notes
- Ensure the ESP32 is powered adequately to drive all components.
- Calibrate the MQ135 and LDR for optimal performance.
- For better obstacle detection, ensure HC-SR04 has a clear field of view.

---

## Troubleshooting

1. **Motors Not Moving**:
   - Check motor connections to the L293D driver.
   - Verify motor enable pins are configured correctly.

2. **Sensors Not Reading**:
   - Ensure proper wiring and pin configurations.
   - Use a multimeter to check power supply to the sensors.

3. **ESP-NOW Communication Issues**:
   - Verify the MAC address of the paired remote device.
   - Check Wi-Fi settings and reinitialize ESP-NOW.

---

## Future Improvements
- Add camera for video streaming.
- Implement GPS for location tracking.
- Add a solar panel for extended outdoor use.

---

Enjoy building and customizing your rover! 🚀


Controller repo: https://github.com/itsfuad/Rover-Controller