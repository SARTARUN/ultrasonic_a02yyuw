# Ultrasonic A02YYUW sensor configuration using the TTL convertor

This package connects an ultrasonic sensor using the TTL convertor and provide the data in ROS2 topic. 

## Ultrasonic A02YYUW sensor

- Waterproof and dustproof (IP67)
- Operating range: 3.3~5V
- Average current rate: 8mA
- Standby current rate: <5mA
- Detection range: 3~450cm
- Blind zone distance: 3cm
- Sensing angle: 60°
- Output: UART
- Response Time: 100ms
- Probe center frequency: 40K(U+00B1)1.0K
- Operating temperature: -15~60°C
- Storage temperature: -25~80°C
- Integrated signal processing
- Band rate: 9600bit/s

## Features
- Publishes distance data from the sensor
- Configurable sensor pins
- Works with FT232H or microcontroller interface

## Usage

```bash
ros2 run ultrasonic_ros2 ultrasonic_node
