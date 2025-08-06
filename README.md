# SMART VIO Workflow

## Project Overview
This project consists of three parts:

Part1: Simulation of misalignment between IMU data and camera images:
This ROS2 node is designed to simulate timestamp misalignment between IMU data and camera images for testing and validation purposes. By subscribing to real-time image and IMU topics from a ZED stereo camera, the node introduces processing delays through optical flow computation, which naturally causes a timing offset between the incoming IMU data and the processed image data.

The purpose of this node is to generate a dataset where image motion vectors and IMU measurements are not perfectly aligned in time, simulating real-world scenarios where sensor processing pipelines introduce latency. This misalignment is useful for developing and testing synchronization algorithms or correction tools in Visual-Inertial Odometry (VIO) workflows.

Part2: 
This Python script reads the image motion data and IMU angular velocity data logged by the Optical Flow ROS2 node and visualizes them in a single graph for comparison.

The script:

Loads motion data from IMG_output.txt and IMU_output.txt.

Sorts the data by timestamp to ensure chronological order.

Standardizes the motion values (removes mean and scales by standard deviation) for better visual comparison.

Plots the X-axis optical flow motion (from images) against the Z-axis angular velocity (from IMU) on a shared timeline.

Displays a simple graph to visualize potential misalignment or correlation between image and IMU data.

This tool helps verify how well image motion vectors and IMU rotations align (or misalign) in time, which is essential for testing synchronization algorithms.

Part3:

This ROS2 Python node reads IMU and camera timestamp data from a serial-connected microcontroller (e.g., Arduino) that packages its data using Protocol Buffers (protobuf).

What it does:
Reads serial data stream from /dev/ttyACM0 at 115200 baud rate.

Detects message start bytes to differentiate between:

IMU data messages (0x55)

Image timestamp messages (0x54)

For each message:

Reads the length-prefixed protobuf payload.

Parses it into either an IMU data message or Image timestamp message.

Converts parsed IMU data into a standard ROS2 sensor_msgs/Imu message and publishes it to /imu/data_serial.

Logs and prints incoming camera timestamps (used for synchronization/debugging).

Purpose:
This node ensures that IMU measurements and camera trigger timestamps are accurately captured and passed into the ROS2 ecosystem for further processing, enabling hardware-level synchronization between IMU and camera data streams.




### Purpose and Motivation

High-Level Architecture

🗂 Repository Structure

Package Breakdown

Directory Tree Overview

⚙️ Installation Guide

System Requirements

ROS2 Workspace Setup

Dependencies Installation

Building the Workspace

📡 IMU-Camera Hardware Synchronization

Overview of Synchronization Method

IMU Serial Data Handling

Generating Synchronized Image-IMU Messages

Running the IMU_CAMERA_Hardware_Sync Node

🔍 Optical Flow and Motion Tracking Node

Optical Flow Computation Pipeline

Real-Time Motion Graph Visualization

Running the ROS2_IMU_OpticalFlow_Node

🖥 Visualization Tools

Visualization Scripts and Output Files

Example Visual Outputs

📝 Data Logging

Image Motion Data (CSV Output)

IMU Angular Velocity Data (CSV Output)

File Structure and Formats

🔧 Usage Workflow

Step-by-Step Execution Flow

Recording & Evaluating Data Sessions

Recommended Playback & Visualization Settings

📊 Data Post-Processing (Optional)

Exporting Data for Analysis

Potential Integration with VIO Algorithms

🧠 Background Theory (Optional Reading)

IMU-Camera Synchronization Challenges

Optical Flow Fundamentals

Why Motion Graphs?

❗ Troubleshooting

Common Errors and Fixes

Platform-Specific Notes (Jetson, Ubuntu, etc.)

🗒️ Future Improvements & TODOs

Planned Features

Known Limitations

📞 Contact & Contributors