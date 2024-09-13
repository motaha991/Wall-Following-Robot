# Wall-Following Robot System

This project involves designing a wall-following robot using an ESP32 microcontroller, range sensors, motor drivers, and other components. The robot autonomously follows walls by using sensor data to adjust its speed and direction. Below is an overview of the system, including block diagram, schematic design, PCB layout, and programming details.

## Block Diagram

![image](https://github.com/user-attachments/assets/bdacd95b-e01b-483f-b754-36c61e1d1324)

The brain of the robot is the microcontroller (ESP32), responsible for processing input from the range sensors and controlling the motors accordingly to follow walls accurately.

- **Microcontroller (MCU)**: Powered by a small battery, receives input from range sensors, and controls the motor drivers.
- **Range Sensors**: Time-of-Flight (ToF) sensors measure the distance to walls, providing data to the MCU.
- **Motor Drivers**: Receive control signals from the MCU and drive the motors, controlling speed and direction to follow the wall.

## Schematic Design

We used the **EasyEDA** online editor to design the schematic, which provides a comprehensive library of components and can be accessed from any device.

### Main Circuit
- **ESP32-WROOM-32D**: The main MCU with connections for power, sensors, and motor drivers.
- **Charging Module**: A step-up module for charging the small battery connected to the ESP32.
- **Calibration Buttons**: Two buttons used for calibrating the sensors to maintain proper wall distance.
- **Debugging Components**: A buzzer and two LEDs for testing and debugging purposes.
- **Power Switch**: For turning the ESP32 on and off.

### Range Sensors
The range sensors, connected via an **I2C multiplexer** (TCA9548A), allow the ESP32 to read data from multiple sensors simultaneously to accurately follow the wall.

- **Headers for Sensors**: Six headers are used to connect the range sensors, which communicate via the SDA and SCL pins of the I2C multiplexer.

### Motor Drivers
The motor driver circuit consists of **two ZK-5AD motor drivers**, with connections to the ESP32, motors, and power supply via **boost converter modules** (XL6009). These modules ensure the motors adjust based on sensor feedback to follow walls precisely.

## PCB Layout

The PCB layout was designed using **EasyEDA**. The PCB has two layers, top and bottom, and measures **94.3mm x 147.6mm**.

### Top Layer
The top layer contains the main components:
- ESP32
- ZK-5AD motor drivers
- TCA9548A I2C multiplexer
- Buzzer, buttons, switches, and LEDs
- Eight range sensors (strategically placed for accurate wall sensing)

### Bottom Layer
The bottom layer is designed for compactness and easier wiring management:
- Two **XL6009 modules** for power management
- **TP4056 charging module** for battery charging
- Connectors for motors and encoder pins

This layout ensures the robot remains compact and organized, with motors placed beneath the PCB to reduce wiring clutter and ensure smooth operation.

## Programming Details

The robot is programmed in **C++** and developed using the **Arduino IDE**. The Arduino IDE is used to compile the code and upload it to the ESP32 microcontroller.

- **Programming Language**: C++
- **Development Environment**: Arduino IDE

The program files are located in this repository and are designed to control the wall-following behavior, using sensor feedback to adjust motor speeds for wall detection and avoidance.

## Tools Used
- **Microcontroller**: ESP32-WROOM-32D
- **Motor Drivers**: ZK-5AD
- **Range Sensors**: ToF sensors
- **PCB Design Software**: EasyEDA online editor
- **Programming**: C++ using Arduino IDE

**Note:** The complete working, PCB layout and Schematic can be found in the **PDF attached**.

## License
This project is open-source. Feel free to modify and contribute.
