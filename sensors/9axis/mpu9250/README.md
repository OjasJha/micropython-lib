# 9-Axis Inertial Measurement Unit (IMU) Sensor Driver

MIT License

Copyright (c) 2025 Ojas Jha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

================================================================================

## Overview

This directory contains a comprehensive MicroPython library for 9-axis Inertial Measurement Unit (IMU) sensors, specifically the MPU9250 module and its components. The library provides high-level handler classes for easy integration with Raspberry Pi Pico projects, along with low-level drivers for direct hardware access.

### What is a 9-Axis IMU?

A 9-axis IMU combines three types of sensors to provide complete motion and orientation data:
- **3-axis Accelerometer**: Measures linear acceleration (including gravity)
- **3-axis Gyroscope**: Measures angular velocity (rotation rate)
- **3-axis Magnetometer**: Measures magnetic field strength and direction (compass)

### MPU9250 Architecture

The MPU9250 is a System in Package (SiP) that combines:
- **MPU6500**: 6-axis IMU (accelerometer + gyroscope) at I2C address 0x68
- **AK8963**: 3-axis magnetometer at I2C address 0x0C
- Both sensors share the same I2C bus but have different addresses

## File Structure

### MPU9250 Directory (`mpu9250/`)

#### High-Level Handler Classes (by Ojas Jha)

1. **`mpu9250_handler.py`**
   - High-level orchestrator for the complete 9-axis IMU system
   - Combines MPU6500 and AK8963 for unified 9-axis interface
   - Provides heading calculation using magnetometer data
   - Handles I2C bypass configuration for AK8963 access
   - Comprehensive status monitoring and error handling
   - Author: Ojas Jha
   - License: MIT License

2. **`mpu6500_handler.py`**
   - High-level handler for the MPU6500 6-axis IMU
   - Implements complementary filter for smooth angle estimation
   - Provides tilt angle calculations (pitch, roll, yaw)
   - Temperature compensation support
   - Configurable full-scale ranges for accelerometer and gyroscope
   - Author: Ojas Jha
   - License: MIT License

3. **`ak8963_handler.py`**
   - High-level handler for the AK8963 3-axis magnetometer
   - Hard/soft iron calibration for accurate compass readings
   - Magnetic field measurement in microTesla (μT)
   - Status monitoring and error handling
   - Author: Ojas Jha
   - License: MIT License

#### Test and Demonstration Scripts (by Ojas Jha)

4. **`mpu9250_test.py`**
   - Comprehensive test suite for 9-axis IMU functionality
   - Demonstrates sensor initialization and data reading
   - Includes continuous monitoring and heading calculation
   - System health and status monitoring examples
   - Author: Ojas Jha
   - License: MIT License

5. **`mpu6500_test.py`**
   - Test suite for 6-axis IMU functionality
   - Demonstrates accelerometer and gyroscope data reading
   - Shows complementary filter usage and configuration
   - Includes tilt angle calculation examples
   - Author: Ojas Jha
   - License: MIT License

6. **`ak8963_test.py`**
   - Test suite for magnetometer functionality
   - Demonstrates basic magnetic field reading
   - Shows calibration process for compass accuracy
   - Includes continuous monitoring examples
   - Author: Ojas Jha
   - License: MIT License

#### Low-Level Driver Classes (by Mika Tuupola)

7. **`mpu9250.py`**
   - Low-level driver for MPU9250 9-axis IMU
   - Direct hardware interface for all 9 axes
   - I2C bypass configuration for magnetometer access
   - Author: Mika Tuupola
   - License: MIT License
   - Source: https://github.com/tuupola/micropython-mpu9250

8. **`mpu6500.py`**
   - Low-level driver for MPU6500 6-axis IMU
   - Direct register access for accelerometer and gyroscope
   - Configurable sensitivity ranges
   - Gyroscope calibration support
   - Author: Mika Tuupola
   - License: MIT License
   - Source: https://github.com/tuupola/micropython-mpu9250

9. **`ak8963.py`**
   - Low-level driver for AK8963 magnetometer
   - Direct register access for magnetic field data
   - Factory sensitivity adjustment support
   - Hard/soft iron calibration algorithms
   - Author: Mika Tuupola
   - License: MIT License
   - Source: https://github.com/tuupola/micropython-mpu9250

## Hardware Requirements

### Raspberry Pi Pico Specifications
- RP2040 dual-core ARM Cortex-M0+ processor
- 264KB RAM, 2MB flash memory
- I2C communication interface (GPIO 20/21 by default)
- 3.3V logic levels

### MPU9250 Module Specifications
- Operating voltage: 3.3V (or 5V with level shifter)
- I2C interface (SCL and SDA)
- I2C addresses: 0x68 (MPU6500), 0x0C (AK8963)
- Pull-up resistors: 4.7kΩ on SCL and SDA lines

### Wiring Connections

```
MPU9250 Module  →  Raspberry Pi Pico
─────────────────────────────────────
VCC             →  3.3V
GND             →  GND
SCL             →  GPIO 21 (I2C SCL) + 4.7kΩ pull-up to 3.3V
SDA             →  GPIO 20 (I2C SDA) + 4.7kΩ pull-up to 3.3V
```

## Quick Start Guide

### 1. Using the High-Level 9-Axis Handler

```python
from mpu9250_handler import MPU9250NineAxisInertialMeasurementUnitHandler

# Initialize the 9-axis IMU
imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler()

# Initialize both sensors (MPU6500 + AK8963)
if imu_9axis.initialize_sensor():
    print("9-axis IMU initialized successfully!")
    
    # Get comprehensive sensor data
    data = imu_9axis.get_comprehensive_sensor_data()
    
    # Display all measurements
    print(f"Temperature: {data['temperature_celsius']:.1f}°C")
    print(f"Acceleration: X={data['accelerometer_data']['x_axis']:.2f} m/s²")
    print(f"Gyroscope: X={data['gyroscope_data']['x_axis']:.2f} °/s")
    print(f"Magnetometer: X={data['magnetometer_data']['x_axis']:.2f} μT")
    print(f"Pitch: {data['filtered_angles']['pitch_angle']:.1f}°")
    print(f"Roll: {data['filtered_angles']['roll_angle']:.1f}°")
    print(f"Heading: {data['heading_degrees']:.1f}°")
else:
    print("Failed to initialize 9-axis IMU")
```

### 2. Using the 6-Axis IMU Handler (MPU6500)

```python
from mpu6500_handler import MPU6500InertialMeasurementUnitHandler

# Initialize the 6-axis IMU
imu_6axis = MPU6500InertialMeasurementUnitHandler()

if imu_6axis.initialize_sensor():
    # Read accelerometer data
    accel_data = imu_6axis.get_accelerometer_data()
    print(f"Acceleration: X={accel_data['x_axis']:.2f} m/s²")
    
    # Read gyroscope data
    gyro_data = imu_6axis.get_gyroscope_data()
    print(f"Gyroscope: X={gyro_data['x_axis']:.2f} °/s")
    
    # Get filtered angles using complementary filter
    filtered_angles = imu_6axis.update_complementary_filtered_angles()
    print(f"Pitch: {filtered_angles['pitch_angle']:.1f}°")
    print(f"Roll: {filtered_angles['roll_angle']:.1f}°")
```

### 3. Using the Magnetometer Handler (AK8963)

```python
from ak8963_handler import AK8963MagnetometerHandler

# Initialize the magnetometer
magnetometer = AK8963MagnetometerHandler()

if magnetometer.initialize_sensor():
    # Read magnetic field data
    mag_data = magnetometer.read_magnetic_field_data()
    print(f"Magnetic Field: X={mag_data['x_axis']:.2f} μT")
    
    # Perform calibration (rotate sensor in all directions)
    print("Rotate sensor in figure-8 patterns...")
    calibration = magnetometer.calibrate_magnetometer(
        calibration_samples=256,
        sample_delay_ms=200
    )
    print(f"Calibration offset: {calibration['offset']}")
    print(f"Calibration scale: {calibration['scale']}")
```

## Key Features

### High-Level Handler Classes

1. **Sensor Fusion**
   - Complementary filter for smooth angle estimation
   - Combines accelerometer and gyroscope data
   - Configurable filter coefficient (0.0-1.0)

2. **Heading Calculation**
   - Uses magnetometer data for compass functionality
   - Calculates heading in degrees (0-360°)
   - Provides magnetic north reference

3. **Calibration Support**
   - Gyroscope bias calibration
   - Hard/soft iron magnetometer calibration
   - Factory sensitivity adjustment application

4. **Comprehensive Status Monitoring**
   - Sensor initialization verification
   - Connection health checks
   - Error message tracking
   - Configuration parameter reporting

5. **Flexible Configuration**
   - Configurable I2C pins and frequency
   - Multiple full-scale ranges
   - Adjustable output units (m/s², g, °/s, rad/s)
   - Filter coefficient customization

## Applications

### Robotics
- Balance control and stabilization
- Navigation and path planning
- Orientation tracking

### Drones and UAVs
- Flight stabilization
- Attitude control
- Heading maintenance
- GPS-denied navigation

### Virtual/Augmented Reality
- Head tracking
- Motion sensing
- Gesture recognition

### Navigation Systems
- Compass functionality
- Tilt compensation
- Dead reckoning

### Gaming
- Motion controllers
- 3D movement detection
- Gesture-based controls

### Industrial
- Vibration monitoring
- Equipment orientation
- Tilt sensing

## Technical Specifications

### Accelerometer (MPU6500)
- Measurement Range: ±2g, ±4g, ±8g, ±16g (configurable)
- Resolution: 16-bit ADC
- Output Units: m/s² or g
- Typical Use: Tilt sensing, motion detection

### Gyroscope (MPU6500)
- Measurement Range: ±250, ±500, ±1000, ±2000 °/s (configurable)
- Resolution: 16-bit ADC
- Output Units: °/s or rad/s
- Typical Use: Rotation detection, angular velocity

### Magnetometer (AK8963)
- Measurement Range: ±4800 μT
- Resolution: 14-bit or 16-bit (configurable)
- Output Units: microTesla (μT)
- Earth's Field: 25-65 μT (location dependent)
- Typical Use: Compass, heading calculation

### Temperature Sensor (MPU6500)
- Range: -40°C to +85°C
- Resolution: ~1°C
- Purpose: Thermal compensation

## Complementary Filter Algorithm

The complementary filter combines accelerometer and gyroscope data for smooth, accurate angle estimates:

```
filtered_angle = α × (angle + gyro × dt) + (1-α) × accel_angle
```

Where:
- **α** = filter coefficient (0.0 to 1.0)
- **α = 0.0**: Use only accelerometer (stable, slow response)
- **α = 1.0**: Use only gyroscope (fast, drifts over time)
- **α = 0.98**: Recommended balance for most applications

## Troubleshooting

### Sensor Not Detected
1. Check wiring connections (SCL, SDA, VCC, GND)
2. Verify power supply (3.3V required)
3. Ensure pull-up resistors are present (4.7kΩ recommended)
4. Test I2C bus with `i2c.scan()` to see detected devices
5. Verify correct I2C addresses (0x68 for MPU6500, 0x0C for AK8963)

### Inaccurate Readings
1. Perform gyroscope calibration when stationary
2. Perform magnetometer calibration (hard/soft iron)
3. Keep sensor away from magnetic interference
4. Adjust complementary filter coefficient
5. Check temperature compensation

### Connection Issues
1. Use shorter wire connections for I2C communication
2. Reduce I2C frequency if experiencing errors
3. Ensure proper pull-up resistor values
4. Avoid electrical noise sources

## Code Examples

All handler files include comprehensive test scripts:
- `mpu9250_test.py`: Complete 9-axis IMU testing
- `mpu6500_test.py`: 6-axis IMU testing and calibration
- `ak8963_test.py`: Magnetometer testing and calibration

Run any test script to verify hardware setup:
```bash
# On Raspberry Pi Pico with MicroPython
python mpu9250_test.py
```

## Credits and Attribution

### High-Level Handler Classes
- **Author**: Ojas Jha
- **License**: MIT License
- **Date**: October 19, 2025
- **Files**: `*_handler.py`, `*_test.py`

### Low-Level Driver Classes
- **Author**: Mika Tuupola
- **License**: MIT License
- **Copyright**: 2018-2023
- **Source**: https://github.com/tuupola/micropython-mpu9250
- **Files**: `mpu9250.py`, `mpu6500.py`, `ak8963.py`

Special thanks to Mika Tuupola for the excellent low-level MicroPython drivers that form the foundation of this library.

## References

### Datasheets
- [MPU-9250 Product Specification](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/)
- [MPU-6500 Register Map](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6500/)
- [AK8963C Datasheet](https://www.akm.com/akm/en/file/datasheet/AK8963C.pdf)

### Raspberry Pi Pico Documentation
- [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [MicroPython Documentation](https://docs.micropython.org/en/latest/rp2/quickref.html)

### Related Resources
- [I2C Protocol Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- [Complementary Filter Tutorial](https://www.pieter-jan.com/node/11)

## Version History

- **Version 2.0** (October 19, 2025)
  - Added comprehensive handler classes by Ojas Jha
  - Implemented complementary filter for angle estimation
  - Added heading calculation using magnetometer
  - Comprehensive documentation and test scripts
  - MIT License applied to all handler and test files

- **Version 0.4.0** (Original low-level drivers by Mika Tuupola)
  - Basic MPU9250, MPU6500, and AK8963 drivers
  - I2C communication implementation
  - Calibration support

## License

This library is released under the MIT License. See individual file headers for specific copyright information.

---

**Author**: Ojas Jha  
**Date**: October 19, 2025  
**License**: MIT License

