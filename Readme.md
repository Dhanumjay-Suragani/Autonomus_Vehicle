# Autonomous Vehicle – V2

## Project Overview

An autonomous vehicle designed to navigate predefined routes using line-following technology, RFID-based location identification, and obstacle detection.

The vehicle identifies its current location through RFID tags, follows a predefined path, and navigates towards the destination selected by the passenger.

## Key Features

- Autonomous line following using IR sensors.
- RFID-based location identification using RC522.
- Predefined route navigation.
- Multiple destination selection.
- Bluetooth-based communication and destination commands.
- Ultrasonic obstacle detection.
- Motor control using an H-bridge motor driver.
- Return-to-start navigation.
- Relay-based external device control.

## Hardware Components

| Component | Purpose |
|---|---|
| STM32H563ZIT6 | Main controller |
| RC522 RFID | Location identification |
| IR Sensors | Line detection |
| Ultrasonic Sensor | Obstacle detection |
| Motor Driver | Motor control |
| DC Motors | Vehicle movement |
| Bluetooth Module | Wireless communication |
| Relay Module | External device control |

## Software and Technologies

- STM32CubeIDE
- STM32 HAL Library
- Embedded C
- UART Communication
- SPI Communication
- PWM Motor Control
- RFID Communication

## Navigation Architecture

1. The vehicle identifies its initial position using RFID.
2. The passenger selects a destination.
3. The controller determines the appropriate route.
4. IR sensors continuously track the navigation line.
5. RFID checkpoints identify the vehicle's location.
6. Ultrasonic sensing monitors obstacles.
7. The vehicle follows the route until the selected destination is reached.

## Project Structure

The repository contains the embedded firmware, peripheral drivers, configuration files, and project documentation.

## Development Status

Version: V2

Project: Autonomous Vehicle

Development focuses on reliable autonomous navigation, RFID checkpoint detection, obstacle avoidance, and integrated embedded control.

## Future Improvements

- Improved navigation accuracy.
- Enhanced obstacle avoidance.
- Real-time vehicle monitoring.
- Advanced route optimization.
- Improved sensor reliability.