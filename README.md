# Robot5A Raspberry Pi - ROS2 Control and Hardware Integration

This repository contains the code and configuration necessary for the Raspberry Pi to act as the hardware control interface for the Robot5A robotic arm. The Raspberry Pi handles the **ROS2 Control** framework and integrates with the **SlushEngine** to drive the stepper motors of the arm. This setup is designed to work with **ROS2 Humble** on Ubuntu 22.04.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Prerequisites](#software-prerequisites)
- [Setup Guide](#setup-guide)
  - [Clone the Repository](#clone-the-repository)
  - [Install Dependencies](#install-dependencies)
  - [Configure the Raspberry Pi](#configure-the-raspberry-pi)
  - [Connect to the SlushEngine](#connect-to-the-slushengine)
  - [Test the Setup](#test-the-setup)
- [Directory Structure](#directory-structure)
- [Usage](#usage)
  - [Launch Files](#launch-files)
  - [Testing the Hardware Interface](#testing-the-hardware-interface)
- [Contributing](#contributing)
- [License](#license)

---

## Project Overview

This repository is part of the Robot5A project and focuses on the **Raspberry Pi hardware interface**. The Raspberry Pi communicates with the **SlushEngine**, which controls the stepper motors, and runs the ROS2 Control stack to receive and execute commands from the central controller on a remote Ubuntu machine.

### Key Components
- **SlushEngine**: Hardware driver board for stepper motors.
- **ROS2 Control**: Framework to manage hardware abstraction and control commands.
- **Visual Servoing**: The Raspberry Pi relies on visual joint state feedback from a remote system.

---

## Features

- **Integration with SlushEngine** for precise stepper motor control.
- **ROS2 hardware interface** to bridge ROS2 controllers and the SlushEngine.
- **Modular configuration** with YAML files for controllers and URDF descriptions.
- Support for **networked ROS2 communication** between the Raspberry Pi and a central controller.

---

## Hardware Requirements

- **Raspberry Pi 4B** with Ubuntu 22.04 (64-bit).
- **SlushEngine Model D** for stepper motor control.
- **Stepper Motors** (compatible with the SlushEngine).
- Ethernet or Wi-Fi connectivity for ROS2 communication.

---

## Software Prerequisites

Before setting up this project, ensure the following are installed on your Raspberry Pi:

1. **ROS2 Humble**  
   Follow the [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html) for Ubuntu 22.04.

2. **Python 3**  
   Install Python and its dependencies:
```bash
sudo apt update
sudo apt install -y python3 python3-pip
```

3. **Git**
Install Git for version control:
```bash
sudo apt install -y git
```

## Setup Guide
### Clone the Repository

Start by cloning this repository onto your Raspberry Pi:
```bash
cd ~
git clone https://github.com/Eliottfrhl/Robot5A-RaspberryPi.git
cd Robot5A-RaspberryPi
```
Install Dependencies

    Install ROS2 Dependencies
    Run the following to install ROS2 dependencies:
```bash
sudo apt install -y python3-colcon-common-extensions ros-humble-ros2-control ros-humble-ros2-controllers
```
Install SlushEngine Dependencies
Follow these steps to set up the SlushEngine:
```bash
sudo apt install -y python3-pip i2c-tools
pip3 install spidev
git clone https://github.com/Roboteurs/slushengine.git
cd slushengine
python3 setup.py install
```
Enable I2C and SPI Interfaces
Enable I2C and SPI on the Raspberry Pi:
```bash
sudo raspi-config
```
    Navigate to Interface Options and enable I2C and SPI.

Configure the Raspberry Pi

    Build the ROS2 Workspace
    Build the workspace after cloning the repository:
```bash
cd ~/Robot5A-RaspberryPi
colcon build --packages-select R5A_hardware
source install/setup.bash
```
Verify ROS2 Installation
Test that ROS2 is working by running:
```bash
ros2 topic list
```
Connect to the SlushEngine

    Connect the SlushEngine to the Raspberry Pi via GPIO.
    Test the Connection
    Run the following test script:
```bash
python3 src/R5A_hardware/scripts/slush_test.py
```
    This script will initialize the motors and execute basic movement commands.

Network Setup for ROS2 Communication

    Assign Static IPs
    Ensure the Raspberry Pi and your Ubuntu computer are on the same network. Assign static IPs if necessary.

    Enable ROS2 Multicast
    Configure ROS2 to use multicast for discovery:
```bash
export ROS_DOMAIN_ID=1
```
Test ROS2 Talker-Listener
On the Raspberry Pi, run:
```bash
ros2 run demo_nodes_py talker
```
On your Ubuntu machine, run:
```bash
ros2 run demo_nodes_py listener
```
Directory Structure

Robot5A-RaspberryPi/
```bash
├── config/
│   ├── controllers.yaml           # Controller configuration
│   ├── robot_description.urdf     # Robot description
│   └── ros2_control_config.yaml   # Hardware interface configuration
├── launch/
│   ├── controller_manager.launch.py
│   ├── hardware_interface.launch.py
│   └── robot_state_publisher.launch.py
├── src/
│   ├── R5A_hardware/
│   │   ├── __init__.py
│   │   └── slush_engine_hardware.py
├── scripts/
│   └── slush_test.py               # Test script for SlushEngine
├── README.md                       # This README file
├── setup.py                        # Package setup file
└── package.xml                     # ROS2 package metadata
```
Usage
Launch Files

    Start Hardware Interface:
```bash
ros2 launch R5A_hardware hardware_interface.launch.py
```
Start Controller Manager:
```bash
ros2 launch R5A_hardware controller_manager.launch.py
```
Publish Robot State:
```bash
ros2 launch R5A_hardware robot_state_publisher.launch.py
```
Testing the Hardware Interface

Run the SlushEngine test script to verify motor control:
```bash
python3 src/R5A_hardware/scripts/slush_test.py
```
Contributing

Contributions are welcome! Please follow these steps:

    Fork the repository.
    Create a feature branch.
    Submit a pull request.

License

This project is licensed under the MIT License.


You can copy and paste this markdown file directly into your repository's `README.md`. Let me know if you want any additional sections or modifications!
