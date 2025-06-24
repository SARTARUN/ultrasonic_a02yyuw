#!/usr/bin/env python3

import serial
import time
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
import json
from rclpy.qos import QoSProfile, DurabilityPolicy

import ament_index_python.packages

class A02YYUWUltrasonicSensor:
    STA_OK = 0x00
    STA_ERR_CHECKSUM = 0x01
    STA_ERR_SERIAL = 0x02
    STA_ERR_CHECK_OUT_LIMIT = 0x03
    STA_ERR_CHECK_LOW_LIMIT = 0x04
    STA_ERR_DATA = 0x05

    last_operate_status = STA_OK

    distance = 0.0
    distance_max = 4500.0
    distance_min = 0.0
    range_max = 4500.0

    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        try:
            self.ser = serial.Serial(
                port=port, 
                baudrate=baudrate, 
                timeout=10
            )
            print('serial:', self.ser)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            sys.exit(1)

    def validate_checksum(self, header, data_h, data_l, sum_byte):
        calculated_sum = (header + data_h + data_l) & 0xFF
        return calculated_sum == sum_byte

    def read_distance(self):
        data = [0]*4
        i = 0
        timenow = time.time()

        while (self.ser.inWaiting() < 4):
            time.sleep(0.01)
            if ((time.time() - timenow) > 1):
                break

        rlt = self.ser.read(self.ser.inWaiting())
        index = len(rlt)
        if(len(rlt) >= 4):
            index = len(rlt) - 4
            while True:
                try:
                    data[0] = ord(rlt[index])
                except:
                    data[0] = rlt[index]
                if(data[0] == 0xFF):
                    break
                elif (index > 0):
                    index = index - 1
                else:
                    break
        if (data[0] == 0xFF):
            try:
                data[1] = ord(rlt[index + 1])
                data[2] = ord(rlt[index + 2])
                data[3] = ord(rlt[index + 3])
            except:
                data[1] = rlt[index + 1]
                data[2] = rlt[index + 2]
                data[3] = rlt[index + 3]
            i = 4
        if i == 4:
            if not self.validate_checksum(data[0], data[1], data[2], data[3]):
                self.last_operate_status = self.STA_ERR_CHECKSUM
            else:
                self.distance = data[1] * 256 + data[2]
                self.last_operate_status = self.STA_OK
            if self.distance > self.distance_max:
                self.last_operate_status = self.STA_ERR_CHECK_OUT_LIMIT
                self.distance = self.distance_max
            elif self.distance < self.distance_min:
                self.last_operate_status = self.STA_ERR_CHECK_LOW_LIMIT
                self.distance = self.distance_min
        else:
            self.last_operate_status = self.STA_ERR_DATA
        return self.distance

class SensorInfoPublisher(Node):
    def __init__(self):
        super().__init__('sensor_info_publisher')
        self.sensor = A02YYUWUltrasonicSensor()
        self.range_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.info_pub = self.create_publisher(String, 'ultrasonic_sensor_info', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        distance = self.sensor.read_distance()
        # Publish sensor info as JSON
        info = {
            'port': self.sensor.ser.port,
            'baudrate': self.sensor.ser.baudrate,
            'distance_cm': distance,
            'distance_max': self.sensor.distance_max,
            'distance_min': self.sensor.distance_min,
            'range_max': self.sensor.range_max
        }
        info_msg = String()
        info_msg.data = json.dumps(info)
        self.info_pub.publish(info_msg)
        # Publish as Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = "ultra_link"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26
        range_msg.min_range = self.sensor.distance_min
        range_msg.max_range = self.sensor.distance_max
        range_msg.range = float(distance) if distance is not None else 0.0
        self.range_pub.publish(range_msg)

class URDFPublisher(Node):
    def __init__(self, urdf_path):
        super().__init__('ultra_description_publisher')
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(String, 'ultra_description', qos_profile)
        self.urdf_path = urdf_path
        self.timer = self.create_timer(1.0, self.publish_urdf)

    def publish_urdf(self):
        try:
            with open(self.urdf_path, 'r') as urdf_file:
                urdf_content = urdf_file.read()
            msg = String()
            msg.data = urdf_content
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read or publish URDF: {e}')

def main(args=None):
    rclpy.init(args=args)
    # Use ament_index_python to find the package share directory
    pkg_share = ament_index_python.packages.get_package_share_directory('ultrasonic_a02yyuw')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ultrasonic_a02yyuw_description.urdf')

    sensor_node = SensorInfoPublisher()
    urdf_node = URDFPublisher(urdf_path)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(urdf_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    sensor_node.destroy_node()
    urdf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()