#!/usr/bin/env python3

import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

# --- Read and parse motion data from file ---
def read_motion_data(filename):
    timestamps = []
    x_values = []
    y_values = []
    z_values = []  # Optional Z values (used for IMU)

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()  # Remove leading/trailing whitespaces
            if line:
                parts = line.split('|')
                # Handle lines with 3 or 4 values (IMG or IMU data)
                if len(parts) == 3:
                    timestamps.append(float(parts[0]))
                    x_values.append(float(parts[1]))
                    y_values.append(float(parts[2]))
                elif len(parts) == 4:
                    timestamps.append(float(parts[0]))
                    x_values.append(float(parts[1]))
                    y_values.append(float(parts[2]))
                    z_values.append(float(parts[3]))
                else:
                    print("Warning: Line format not recognized:", line)
    return timestamps, x_values, y_values, z_values

# --- File paths for IMG and IMU data ---
img_file = 'src/zed_sub/data/IMG_output.txt'
imu_file = 'src/zed_sub/data/IMU_output.txt'

# --- Read IMG and IMU data from files ---
img_timestamps, img_x, img_y, img_z = read_motion_data(img_file)
imu_timestamps, imu_x, imu_y, imu_z = read_motion_data(imu_file)

# --- Sort IMG data by timestamp ---
if img_timestamps:
    combined_img = list(zip(img_timestamps, img_x, img_y))
    combined_img.sort(key=lambda x: x[0])  # Sort by timestamp
    img_timestamps_sorted, img_x_sorted, img_y_sorted = zip(*combined_img)

# --- Sort IMU data by timestamp ---
if imu_timestamps:
    combined_imu = list(zip(imu_timestamps, imu_x, imu_y, imu_z))
    combined_imu.sort(key=lambda x: x[0])  # Sort by timestamp
    imu_timestamps_sorted, imu_x_sorted, imu_y_sorted, imu_z_sorted = zip(*combined_imu)

# --- Standardize IMG X-axis motion data ---
if img_timestamps:
    img_x_mean = np.mean(img_x_sorted)
    img_x_std = np.std(img_x_sorted)
    img_x_standardized = (np.array(img_x_sorted) - img_x_mean) / img_x_std

# --- Standardize IMU Z-axis angular velocity data ---
if imu_timestamps:
    imu_z_mean = np.mean(imu_z_sorted)
    imu_z_std = np.std(imu_z_sorted)
    imu_z_standardized = (np.array(imu_z_sorted) - imu_z_mean) / imu_z_std

# --- Plot the standardized data ---
if img_timestamps and imu_timestamps:
    plt.figure()

    plt.plot(img_timestamps_sorted, img_x_standardized, label='IMG_X-axis (Standardized)')
    # plt.plot(img_timestamps_sorted, img_y_sorted, label='IMG_Y-axis')  # Optional Y-axis plot

    plt.plot(imu_timestamps_sorted, imu_z_standardized, label='IMU_Z-axis (Standardized)')
    # plt.plot(imu_timestamps_sorted, imu_y_sorted, label='IMU_Y-axis')  # Optional Y-axis plot

    plt.xlabel("Timestamp (seconds)")
    plt.ylabel("Motion Measurement (Standardized)")
    plt.title("Motion Data Visualization (IMG vs IMU)")

    plt.legend()
    plt.grid(True)
    plt.show()

else:
    print("Error: Failed to load data from IMG or IMU file.")
