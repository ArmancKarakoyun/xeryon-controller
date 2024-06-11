# Xeryon Controller Package

## Overview
The `xeryon_controller` package is a ROS2 node designed to control linear and rotary motors using the Xeryon controller. The package subscribes to specific topics to receive displacement data and updates the actuator's position accordingly.

## Features
- Controls linear and rotary motors using the Xeryon controller.
- Subscribes to ROS2 topics to receive displacement data.
- Sets the actuator's position based on the received data.
- Provides robust error handling and logging.

## Prerequisites
Before you begin, ensure you have met the following requirements:
- ROS2 (Foxy, Galactic, Humble, etc.)
- Python 3.6 or later
- Xeryon controller SDK

## Installation
To install the `xeryon_controller` package, follow these steps:

1. **Clone the repository:**
    ```sh
    git clone https://github.com/ArmancKarakoyun/xeryon_controller.git
    ```

2. **Navigate to the package directory:**
    ```sh
    cd xeryon_controller
    ```

3. **Install dependencies:**
    Ensure you have all the necessary dependencies installed for ROS2 and the Xeryon controller SDK.

4. **Build the package:**
    ```sh
    colcon build
    ```

## Usage
To use the `xeryon_controller` package, follow these steps:

### Running the Rotary Motor Controller Node
1. **Launch the ROS2 node:**
    ```sh
    ros2 run xeryon_controller launch.py
    ```

2. **Publish displacement data to the `rotary_data` topic:**
    ```sh
    ros2 topic pub /rotary_data std_msgs/msg/Float64 "{data: 10.0}"
    ```

### Running the Linear Motor Controller Node
1. **Launch the ROS2 node:**
    ```sh
    ros2 run xeryon_controller linear_motor_controller
    ```

2. **Publish displacement data to the `linear_data` topic:**
    ```sh
    ros2 topic pub /linear_data std_msgs/msg/Float64 "{data: 5000.0}"
    ```

## Node Details

### Rotary Motor Controller
- **Node Name**: `rotary_node`
- **Subscribed Topic**: `rotary_data` (std_msgs/Float64)
- **Functionality**: Controls the rotary motor based on received displacement data.

### Linear Motor Controller
- **Node Name**: `linear_node`
- **Subscribed Topic**: `linear_data` (std_msgs/Float64)
- **Functionality**: Controls the linear motor based on received displacement data.

## Error Handling and Logging
The nodes are equipped with robust error handling to manage initialization and runtime issues. Logs are generated to help with debugging and monitoring.

## Contributors
- **Armanc Karakoyun** - [@ArmancKarakoyun](https://github.com/ArmancKarakoyun)

## License
This project is licensed under the Apache License, Version 2.0 - see the [LICENSE](LICENSE) file for details.

