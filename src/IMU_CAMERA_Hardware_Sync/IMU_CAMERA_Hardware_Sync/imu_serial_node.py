#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial
import struct

import sys
import os
sys.path.append(os.path.dirname(__file__))
import img_imu_data_pb2 # Generated Protobuf file

from std_msgs.msg import String  # or a custom message type
from sensor_msgs.msg import Imu

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_serial', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.01, self.read_serial)
        self.get_logger().info("test1")


    def read_serial(self):
        try:
            self.get_logger().info("test2")

            # Wait until startbytes are found
            while True:
                byte1 = self.serial_port.read(1)
                if byte1 == b'\xAA':
                    self.get_logger().info("byte1 read")
                else:
                    continue
                byte2 = self.serial_port.read(1)
                if byte2 == b'\x55' or byte2 == b'\x54':
                    self.get_logger().info("byte2 read")
                else: 
                    continue

                #IMU data message
                if(byte2 == b'\x55'):
                    try:
                        self.get_logger().info("test_IMU")
                        
                        # Read 4-byte length prefix little endian (0->255)
                        length_bytes = self.serial_port.read(4)
                        if len(length_bytes) < 4:
                            return
                        
                        length = struct.unpack('<I', length_bytes)[0]

                        # Read message
                        data = self.serial_port.read(length)
                        if len(data) < length:  # Message is not correct
                            self.get_logger().info(f"Incomplete message: {len(data)}/{length} bytes")
                            return
                        
                        

                        # Decode protobuf
                        imu_data = img_imu_data_pb2.ImuData()
                        imu_data.ParseFromString(data)

                        # Convert to IMU data
                        imu_msg = Imu()             # Create IMU message
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"

                        imu_msg.linear_acceleration.x = imu_data.acc_x
                        imu_msg.linear_acceleration.y = imu_data.acc_y
                        imu_msg.linear_acceleration.z = imu_data.acc_z

                        imu_msg.angular_velocity.x = imu_data.ang_x
                        imu_msg.angular_velocity.y = imu_data.ang_y
                        imu_msg.angular_velocity.z = imu_data.ang_z

                        float_timestamp = imu_data.imu_timestamp
                        
                        seconds = int(float_timestamp)
                        nanoseconds = int((float_timestamp - seconds) * 1e9)
                        nanoseconds = min(max(nanoseconds, 0), 4294967295)  # Ensure it's within nanosec limits
                        imu_msg.header.stamp.sec = seconds
                        imu_msg.header.stamp.nanosec = nanoseconds

                        # Orientation and angular velocity
                        self.publisher_.publish(imu_msg)
                        self.get_logger().info("Published IMU data")
                    except Exception as e:
                        self.get_logger().error(f"parsing ImuData failed: {e}")
                
                #IMG timestamp message
                elif(byte2 == b'\x54'):
                    self.get_logger().info("test_IMG")

                    # Read 4-byte length prefix little endian (0->255)
                    length_bytes = self.serial_port.read(4)
                    if len(length_bytes) < 4:
                        self.get_logger().error("Length bytes too short")
                        return
                    
                    length = struct.unpack('<I', length_bytes)[0]

                    # Read message
                    data = self.serial_port.read(length)
                    if len(data) < length:  # Message is not correct
                        self.get_logger().info(f"Incomplete message: {len(data)}/{length} bytes")
                        return
                    try:
                        # Decode protobuf
                        img_data = img_imu_data_pb2.ImgData()
                        img_data.ParseFromString(data)

                        #Convert to IMG timestamp
                        img_timestamp = img_data.camera_timestamp

                        self.get_logger().info(str(img_timestamp))
                    except Exception as e:
                        self.get_logger().error(f"Parsing ImgData failed: {e}")
                        self.get_logger().info(f"Raw length: {length}")
                        self.get_logger().info(f"Raw data (hex): {data.hex()}")
                else:
                    self.get_logger().info("Invalid second start byte")

        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    serial_node = IMUSerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()