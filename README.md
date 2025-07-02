# Ultrasonic A02YYUW sensor configuration using FT232-AZ USB to TTL Adapter

This package connects A02YYUW sensor using FT232-AZ USB to TTL Adapter and provide the data in ROS2 topic. 

## Ultrasonic A02YYUW sensor

### Features

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

### Dimension
![A02YYUW Sensor Dimension](images/A02YYUW_Dimension.png)

### Pinout
![A02YYUW Sensor Pinout](images/A02YYUW_pinout.png)

|Label  |Name   |Description        |
|------:|:-----:|:------------------|
|1      |VCC    |Power              |   
|2      |GND    |Ground             |
|3      |RX     |Processes Vales    |
|4      |TX     |UART Output        |

For more information about the sensor, please refer to official wiki page for [A02YYUW Sensor](https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311) on DFRobot website. 

## FT232-AZ USB to TTL Adapter

### Features

- Operating range: 3.3V or 5V (can be set using the jumper on the board)
- Average current rate: 24mA
- Operating temperature: -40~85°C
- Standby temperature: -65~150°C
- Interface: UART
- Chip: FT232-AZ
- Data connection: Mini-USB-B cable

### Dimension
![TTL Adapter Dimension](images/TTLConverter_Dimension.png)

### Pinout
![TTL Adapter Pinout](images/TTLConverter_pinout_2.png)

For more information about the adapter, please refer to [FT232-AZ USB to TTL Serial Adapter](https://www.az-delivery.de/en/products/ftdi-adapter-ft232rl?variant=36542809426) on AZ-Delivery website. 

## Connection between A02YYUW and FT232-AZ USB to TTL Adapter

|Sr. No.|A02YYUW|Adapter|    
|------:|:-----:|:-----:|
|1      |VCC    |VCC    |
|2      |GND    |GND    |
|3      |RX     |TX     |
|4      |TX     |RX     |

## Building
Clone this repositroy in your ROS2 workspace:
```bash
git clone https://github.com/SARTARUN/ultrasonic_a02yyuw
```
Then build the package and source the workspace:
```bash
colcon build
source install/local_setub.bash
```

## Running the library
To the launch the package write down the command in terminal:
```bash
ros2 launch ultrasonic_a02yyuw ultrasonic_a02yyuw.launch.py
```

<!--## References

1. [Ultrasonic A02YYUW Sensor Datasheet](https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311)

-->