# SMART VIO Workflow

## Project Overview
This project provides a modular workflow for testing, visualizing, and correcting IMU & Camera data synchronization issues in Visual-Inertial Odometry (VIO) systems. It consists of four interconnected components that simulate sensor misalignment, visualize motion data, and enable hardware-level timestamp synchronization.

### Part 1: Simulating IMU–Camera Timestamp Misalignment
A ROS2 node subscribes to ZED camera images and IMU data, computes optical flow on the images, and introduces a natural processing delay. This creates intentional timestamp misalignment between image motion vectors and IMU measurements, mimicking real-world sensor processing delays. This dataset is used to test synchronization correction algorithms for VIO workflows.

### Part 2: Motion Data Visualization Script
A Python script loads the logged image motion data and IMU angular velocity data, sorts them by timestamp, standardizes their scales, and plots them together. This allows visual inspection of temporal misalignments between image-based motion estimation and IMU rotations, which is essential for debugging synchronization issues.

### Part 3: IMU Serial ROS2 Node (Synchronization Interface)
A ROS2 Python node reads IMU measurements and camera trigger timestamps from a serial-connected microcontroller. Data is structured using Protocol Buffers (protobuf), with message headers indicating whether it’s IMU data or a camera trigger event. The node publishes parsed IMU data as ROS2 sensor_msgs/Imu messages and logs camera timestamps, enabling accurate timestamp ingestion into the ROS2 ecosystem.

### Part 4: Arduino IMU & Camera Timestamp Publisher
An Arduino sketch interfaces with an MPU6050 IMU sensor to read acceleration and angular velocity data. It also generates camera trigger timestamps using the Arduino’s microsecond timer. Both data types are encoded using Nanopb (protobuf for embedded) and sent via Serial to a connected ROS2 machine. This acts as the hardware synchronization source, ensuring precise timing between IMU measurements and camera events.

### Purpose and Motivation
The purpose of this project is to create a hardware-accurate workflow for synchronizing IMU and camera data in Visual-Inertial Odometry (VIO) systems, while also providing tools to simulate, visualize, and correct timestamp misalignments between these sensors.

## Installation Guide
This guide walks you through setting up the entire workflow, including ROS2 nodes, Arduino code, and visualization scripts.

### System Requirements
| Component          | Requirement                      |
| ------------------ | -------------------------------- |
| OS                 | Ubuntu 22.04                     |
| ROS2               | Humble Hawksbill (or compatible) |
| Python             | Python 3.8+                      |
| C++ Compiler       | GCC >= 9                         |
| Arduino IDE        | Version >= 1.8.x                 |
| ZED SDK (Optional) | For live ZED camera topics       |

### Clone Repository & Setup ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/JesseMasselink/SMART_VIO_Workflow.git
```

### Install ROS2 & Python Dependencies
```bash
sudo apt update
# Install ROS2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-rclcpp
# Install Python dependencies
sudo apt install libopencv-dev python3-opencv python3-pip python3-numpy python3-matplotlib
# Install pip dependencies
pip3 install protobuf pyserial
```

### Build ROS2 Workspace
Make sure ament_cmake_python is enabled for mixed C++ and Python packages:
```bash
cd ~/ros2_ws
colcon build --packages-select zed_sub imu_camera_hardware_sync --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```


## Usage Workflow (using bagfile)
This workflow shows how to use a pre-recorded ROS2 bagfile containing IMU and Camera data to simulate sensor misalignment, log data, and visualize synchronization issues using this repository.

### Source Environment
Before you start, source ROS2 environment:
```bash
source ~/ros2_ws/install/setup.bash
```

### Step 1: Ensure ROS2 Bagfile Contains IMU and Camera Data
Prepare a ROS2 bagfile that includes:
- Camera image topic: zed/zed_node/left_raw/image_raw_color
- IMU data topic: /zed/zed_node/imu/data

### Step 2: Launch Optical Flow Node
This ROS2 C++ node will subscribe to the image and IMU topics from the bagfile, compute optical flow, and log the image motion vectors and IMU data.
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run zed_sub zed_sub
```
What happens:
- Node subscribes to the image and IMU topics from the bagfile (next step).
- Computes optical flow between image frames.
- Logs to CSV files:
    - Image motion vectors to IMG_output.txt
    - IMU angular velocities to IMU_output.txt

### Step 3: Play the Bagfile (With Slowed Playback Rate)
To ensure that the Optical Flow Node can keep up with the data, play the bagfile at a reduced rate. This prevents message drops and ensures all data is processed.

```bash
ros2 bag play /path/to/your/data.bag -r 0.2
```
What happens:
- Bagfile publishes IMU and camera data to ROS2 topics.
- The Optical Flow Node processes incomming data and records motion values into CSV files (previous step).
- The misalignment effect occurs naturally as image processing is slower than IMU data recording.

### Step 4: Stop Playback
Once the bagfile has fully played, stop the Optical Flow Node after it finishes processing.
Make sure that:
- Bagfile has stopped playing.
- IMG_output.txt and IMU_output.txt are generated in the src/zed_sub/data directory.

### Step 5: Visualize Data Misalignment
Run the visualization Python script to compare image motion vectors and IMU angular velocities on a shared timeline.

```bash
python3 src/SMART_VIO_Workflow/src/zed_sub/visualization.py
```
What Happens:
- Reads the logged IMG_output.txt and IMU_output.txt.
- Standardizes data for visual comparison.
- Plots image X-motion (optical flow) vs IMU Z-angular velocity.
- Reveals timestamp alignment (or misalignment) for analysis.


# Hardware Synchronization
## Overview
This section explains how to set up hardware-level synchronization between an MPU6050 IMU sensor and an Arduino, which is intended to provide precise timestamps for both IMU readings and camera trigger events. This setup enables low-level synchronization by using the Arduino as a data relay and timestamp generator, ensuring accurate alignment between camera images and IMU data streams.

Note: As of now, the ZED Camera trigger integration is not yet fully implemented. The Arduino provides IMU data and camera trigger timestamps, but the ZED camera hardware trigger connection remains a future task.

## Hardware Components Required
| Component              | Details                             |
| ---------------------- | ----------------------------------- |
| Microcontroller        | Arduino (Arduino UNO/MEGA)          |
| IMU Sensor             | MPU6050                             |
| Connection Interface   | I2C (SDA/SCL), Serial USB to PC     |
| Optional (Future Work) | ZED Camera GPIO trigger integration |

## Wiring
| MPU6050 Pin | Arduino Pin               | Description    |
| ----------- | ------------------------- | -------------- |
| VCC         | 3.3V or 5V                | Power supply   |
| GND         | GND                       | Ground         |
| SDA         | A4 (or dedicated SDA pin) | I2C Data Line  |
| SCL         | A5 (or dedicated SCL pin) | I2C Clock Line |

Additional Notes:
- Connect the Arduino to your PC via USB.
- Ensure the correct Serial Port is selected in the Arduino IDE (/dev/ttyACM0 on Linux).
- No additional hardware wiring for the ZED Camera trigger has been implemented yet

## Setup Arduino Environment
#### Install Arduino IDE
```bash
sudo apt install arduino
```

#### Install Required Libraries
1. Navigate to Library Manager.
2. Search: "Adafruit MPU6050" and install.

#### Upload to Arduino Board
3. Open "IMU_Reader.ino" in Arduino IDE.
4. Upload to correct port (/dev/ttyACM0).


## Current State
- The Arduino reads IMU data (acceleration & angular velocity) at a high frequency.
- Each reading is timestamped using the Arduino’s internal microsecond timer.
- Camera trigger timestamps are simulated by the Arduino’s internal timer as placeholder values.
- The data is serialized using Protocol Buffers (Nanopb) and sent to the host PC over Serial USB.
- A ROS2 node (imu_serial_node.py) reads and parses these messages, publishing IMU data to /imu/data_serial.

## Limitations and Pending work:
- The hardware trigger connection to the ZED Camera (via GPIO pins) is NOT implemented yet.
- Therefore, the actual hardware-synchronized timestamping of camera frames is currently simulated.
- This part of the workflow is intended for future development, where the Arduino will detect real camera triggers and generate accurate frame timestamps.

## Purpose of Current Prototype
Even though the ZED camera trigger is not active, this hardware setup allows:
- Testing hardware-level timestamp acquisition from the IMU.
- Developing and validating the serial communication pipeline (Protobuf streaming to ROS2).
- Preparing the system for when camera hardware synchronization is added.

## Troubleshooting
- Serial Port Settings: Ensure Arduino is connected to /dev/ttyACM0. If not, adjust the port in imu_serial_node.py. 
This allows for real time imu and camera synchronization.


# Future work
To achieve the systems original goal of fully synchronized IMU and Camera data, the following steps are required to finalize this project.

## Execution Flow Diagram
Currently, the project is divided into two separate parts: (1) IMU-Camera Misalignment Simulation and (2) Hardware Synchronization Prototype. Below is the intended data flow for the complete system:

1. Arduino:
    - Reads IMU data at 100Hz, encodes it using Protocol Buffers, and sends it to the PC via Serial.
    - Triggers the camera hardware at 20Hz, aligned with IMU readings. (To be implemented)

2. Camera (ZED):
- Receives hardware trigger signals from Arduino.
- Captures images and sends data to the PC via ROS2 topics. (To be implemented)

3. ROS2 IMU Serial Node:
- Reads incoming Protobuf messages from Arduino.
- Parses IMU data and publishes it to the topic /imu/data_serial.

4. Optical Flow Node (ROS2 C++):
- Subscribes to camera image data. (Timestamp correction logic to be added)
- Subscribes to /imu/data_serial for synchronized IMU data.
- Computes optical flow between consecutive image frames.
- Logs image motion vectors and IMU angular velocities to CSV files.

5. Visualization Script (Python):
- Loads logged motion data from CSV files.
- Plots image motion vectors and IMU angular velocities on a shared timeline.
- Highlights timestamp misalignment and helps evaluate synchronization accuracy. (Full alignment verification still to be developed)

## Optional Additions:
- Implement ZED GPIO trigger connection.
- Add timestamp correction logic in Optical Flow Node.
- Automate Protobuf compilation with CMake.
- Optimize Optical Flow processing speed for real-time performance.


# Troubleshooting & Common Issues
This section lists common problems encountered during the development of this repository, along with solutions and workarounds.

## Protobuf & Serial Communication Issues
### Nanopb Compatibility Issues on Arduino Mega
- Problem: Arduino UNO does not support certain libraries used by Nanopb (limits.h).
- Solution:
Use a more advanced microcontroller like the Arduino MEGA, which has full support for Nanopb.

### Corrupt or Unreadable Serial Data
- Problem: Serial data stream from Arduino appears as unusable values.
- Solution:
    - Add start bytes to every message:
        - 0xAA 0x55 IMU Data
        - 0xAA 0x54 Camera Trigger
    - This signals the receiving script where each packet begins.

### Python Can't Find Generated Protobuf .py File
- Problem: ImportError when using generated .py Protobuf files.
- Solution: 
    Run the generated file once manually to ensure Python caches it:
```bash
python3 img_imu_data_pb2.py
```

## ROS2 and Data Processing Issues
### Missing Output Directories
- Problem: The directories where logs are saved (src/zed_sub/data/) might not exist. This wil result in not saving the output files.
- Solution: Create directory where files can be saved
```bash
mkdir -p src/zed_sub/data/
```

### IMU and Image CSV Data Starts from 0
- Problem: Unwanted datapoint (0.0, 0.0, 0.0) is present in IMG_output.txt and/or IMU_output.txt. This can cause troubles when visualising the data.
- Solution: 
    - Open text file and search (ctrl+f) for 0.0 value and remove line.    
    - Don't forget to save (ctrl+s).
