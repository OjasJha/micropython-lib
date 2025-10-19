# MPU6050 6-Axis IMU Sensor - MicroPython Driver

A comprehensive MicroPython library for the MPU6050 6-axis Inertial Measurement Unit (IMU) designed for Raspberry Pi Pico 2 W.

## License

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

---

## Features

- **Complete 6-Axis Motion Tracking**
  - 3-axis accelerometer (linear acceleration measurement)
  - 3-axis gyroscope (angular velocity measurement)
  - Built-in temperature sensor

- **Advanced Sensor Fusion**
  - Complementary filter implementation
  - Real-time angle estimation (pitch, roll, yaw)
  - Drift correction for stable orientation tracking

- **Object-Oriented Design**
  - High-level handler class for easy integration
  - Clean API with comprehensive error handling
  - Configurable parameters for different applications

- **Comprehensive Data Access**
  - Raw sensor data (accelerometer, gyroscope, temperature)
  - Calculated tilt angles from accelerometer
  - Filtered orientation angles with sensor fusion
  - Timestamped readings for data logging

- **Flexible Configuration**
  - Configurable I2C pins and frequency
  - Adjustable complementary filter coefficient
  - Support for both I2C addresses (0x68, 0x69)

## Hardware Requirements

- Raspberry Pi Pico 2 W (or Pico/Pico W)
- MPU6050 IMU sensor module
- MicroPython firmware (v1.26.1 or later)
- I2C connections (SDA, SCL, VCC, GND)

## Wiring

| MPU6050 Pin | Raspberry Pi Pico Pin | Description |
|-------------|----------------------|-------------|
| VCC | 3.3V (Pin 36) | Power supply (3.3V) |
| GND | GND (Pin 38) | Ground |
| SCL | GPIO 21 (Pin 27) | I2C Clock Line (configurable) |
| SDA | GPIO 20 (Pin 26) | I2C Data Line (configurable) |
| AD0 | GND or VCC | I2C Address Select (GND=0x68, VCC=0x69) |

**Note:** Most MPU6050 modules have built-in pull-up resistors for I2C. If yours doesn't, add 4.7kΩ pull-up resistors on both SDA and SCL lines to 3.3V.

## Files Included

### 1. `mpu6050.py` - Low-Level Driver Library
The core library providing low-level functions for MPU6050 communication:
- Sensor initialization and configuration
- Raw data reading from registers
- Data conversion to physical units
- Register address constants

**Key Functions:**
- `initialize_mpu6050_sensor(i2c_bus, device_address)` - Initialize sensor
- `read_sensor_data(i2c_bus)` - Read all sensor data
- `read_16bit_signed_data(i2c_bus, register_address, device_address)` - Read raw register data

### 2. `mpu6050_handler.py` - High-Level Handler Class
Object-oriented wrapper providing easy-to-use interface:
- Automatic sensor initialization
- Real-time sensor fusion with complementary filter
- Comprehensive data access methods
- Built-in error handling and status monitoring

**Key Class: `MPU6050Handler`**

### 3. `mpu6050_test.py` - Demonstration and Test Program
Comprehensive test suite demonstrating all features:
- Basic sensor reading examples
- Angle calculation demonstrations
- Sensor fusion filtering examples
- Configuration and customization examples
- Real-time monitoring demonstration

## Quick Start

### Basic Usage

```python
from mpu6050_handler import MPU6050Handler
import utime

# Create handler with default settings
mpu = MPU6050Handler()

# Initialize the sensor
if not mpu.initialize():
    print("Sensor initialization failed!")
else:
    # Read sensor data
    temperature = mpu.get_temperature()
    acceleration = mpu.get_acceleration()
    angular_velocity = mpu.get_angular_velocity()
    
    print(f"Temperature: {temperature:.2f} °C")
    print(f"Acceleration: X={acceleration['x']:.3f}g, "
          f"Y={acceleration['y']:.3f}g, Z={acceleration['z']:.3f}g")
    print(f"Angular Velocity: X={angular_velocity['x']:.2f}°/s, "
          f"Y={angular_velocity['y']:.2f}°/s, Z={angular_velocity['z']:.2f}°/s")
```

### Getting Tilt Angles

```python
from mpu6050_handler import MPU6050Handler

mpu = MPU6050Handler()
mpu.initialize()

# Get tilt angles from accelerometer
tilt_angles = mpu.get_tilt_angles()
print(f"Pitch: {tilt_angles['pitch']:.2f}°")
print(f"Roll:  {tilt_angles['roll']:.2f}°")
print(f"Yaw:   {tilt_angles['yaw']:.2f}°")
```

### Using Sensor Fusion (Complementary Filter)

```python
from mpu6050_handler import MPU6050Handler
import utime

mpu = MPU6050Handler(filter_coefficient=0.98)  # 98% gyro, 2% accel
mpu.initialize()

# Continuous filtered angle readings
while True:
    filtered_angles = mpu.get_filtered_angles()
    print(f"Filtered Pitch: {filtered_angles['pitch']:.2f}°, "
          f"Roll: {filtered_angles['roll']:.2f}°")
    utime.sleep(0.1)
```

### Comprehensive Data Access

```python
from mpu6050_handler import MPU6050Handler

mpu = MPU6050Handler()
mpu.initialize()

# Get all data in one call
data = mpu.get_comprehensive_data()

print(f"Timestamp: {data['timestamp_ms']} ms")
print(f"Temperature: {data['temperature']:.2f} °C")
print(f"Accelerometer: {data['accelerometer']}")
print(f"Gyroscope: {data['gyroscope']}")
print(f"Tilt Angles: {data['tilt_angles']}")
print(f"Filtered Angles: {data['filtered_angles']}")
```

### Custom Configuration

```python
from mpu6050_handler import MPU6050Handler

# Create handler with custom I2C settings
mpu = MPU6050Handler(
    scl_pin=21,                    # Custom SCL pin
    sda_pin=20,                    # Custom SDA pin
    i2c_freq=400000,               # 400kHz I2C frequency
    device_address=0x68,           # I2C address
    filter_coefficient=0.95,       # 95% gyro, 5% accel (more stable)
    i2c_id=0                       # I2C controller ID (0 or 1)
)

if mpu.initialize():
    # Change filter coefficient during runtime
    mpu.set_filter_coefficient(0.90)  # Even more stable
    
    # Reset filter state
    mpu.reset_filter()
    
    # Check status
    status = mpu.get_status()
    print(f"Connected: {status['is_connected']}")
    print(f"Filter Coefficient: {status['filter_coefficient']}")
```

## API Reference

### MPU6050Handler Class

#### Constructor
```python
MPU6050Handler(scl_pin=21, sda_pin=20, i2c_freq=400000, 
               device_address=0x68, filter_coefficient=0.98, i2c_id=0)
```

**Parameters:**
- `scl_pin` (int): GPIO pin for I2C clock line (default: 21)
- `sda_pin` (int): GPIO pin for I2C data line (default: 20)
- `i2c_freq` (int): I2C frequency in Hz (default: 400000)
- `device_address` (int): I2C address of MPU6050 (default: 0x68)
- `filter_coefficient` (float): Complementary filter weight 0.0-1.0 (default: 0.98)
- `i2c_id` (int): I2C controller ID, 0 or 1 (default: 0)

#### Methods

##### `initialize()`
Initialize and configure the MPU6050 sensor.

**Returns:** `bool` - True if successful, False otherwise

##### `is_connected()`
Check if sensor is responding on I2C bus.

**Returns:** `bool` - True if connected, False otherwise

##### `read_raw_data()`
Read raw sensor data from MPU6050.

**Returns:** `dict` - Dictionary with temperature, accelerometer, and gyroscope data

##### `get_temperature()`
Get current temperature reading.

**Returns:** `float` - Temperature in Celsius

##### `get_acceleration()`
Get current acceleration data.

**Returns:** `dict` - Dictionary with x, y, z acceleration values in g-force

##### `get_angular_velocity()`
Get current gyroscope data.

**Returns:** `dict` - Dictionary with x, y, z angular velocity in degrees/second

##### `get_tilt_angles()`
Get calculated tilt angles from accelerometer.

**Returns:** `dict` - Dictionary with pitch, roll, yaw angles in degrees

##### `get_filtered_angles()`
Get sensor fusion filtered angles.

**Returns:** `dict` - Dictionary with filtered pitch and roll in degrees

##### `get_comprehensive_data()`
Get all sensor data including raw readings, calculated angles, and filtered angles.

**Returns:** `dict` - Comprehensive data structure with all information

##### `calculate_tilt_angles(accelerometer_data)`
Calculate tilt angles from accelerometer data.

**Parameters:**
- `accelerometer_data` (dict): Accelerometer data dictionary

**Returns:** `tuple` - (pitch, roll, yaw) in degrees

##### `update_filtered_angles(gyroscope_data, delta_time_seconds=None)`
Update filtered angles using complementary filter.

**Parameters:**
- `gyroscope_data` (dict): Gyroscope data dictionary
- `delta_time_seconds` (float, optional): Time elapsed since last update

**Returns:** `tuple` - (filtered_pitch, filtered_roll) in degrees

##### `reset_filter()`
Reset the sensor fusion filter state to zero.

##### `set_filter_coefficient(coefficient)`
Set the complementary filter coefficient.

**Parameters:**
- `coefficient` (float): Filter coefficient 0.0-1.0
  - 0.98: 98% gyro + 2% accel (default, smooth)
  - 0.95: 95% gyro + 5% accel (more stable)
  - 0.90: 90% gyro + 10% accel (very stable)

##### `get_status()`
Get current handler status information.

**Returns:** `dict` - Status dictionary with initialization, connection, and configuration info

## Understanding Sensor Data

### Accelerometer
Measures linear acceleration in g-force units (1g = 9.81 m/s²):
- **X-axis:** Left/Right tilt
- **Y-axis:** Forward/Backward tilt  
- **Z-axis:** Up/Down acceleration
- **Range:** ±2g (configured for maximum sensitivity)
- **Sensitivity:** 16384 LSB/g

### Gyroscope
Measures angular velocity (rotation rate) in degrees per second:
- **X-axis:** Pitch rate (rotation around X-axis)
- **Y-axis:** Roll rate (rotation around Y-axis)
- **Z-axis:** Yaw rate (rotation around Z-axis)
- **Range:** ±250°/s (configured for maximum sensitivity)
- **Sensitivity:** 131 LSB/°/s

### Tilt Angles
Calculated from accelerometer using trigonometry:
- **Pitch:** Rotation around X-axis (forward/backward tilt)
- **Roll:** Rotation around Y-axis (left/right tilt)
- **Yaw:** Rotation around Z-axis (compass heading approximation)

### Sensor Fusion (Complementary Filter)
Combines gyroscope and accelerometer data for stable angle estimation:
- **Gyroscope:** Good short-term accuracy, but drifts over time
- **Accelerometer:** Accurate long-term, but noisy with vibration
- **Filter:** Blends both sensors for best of both worlds

**Filter Coefficient Guidelines:**
- `0.98` (default): Very smooth, slight drift correction
- `0.95`: Balanced smoothness and stability
- `0.90`: More stable, responds faster to corrections
- `0.80`: Very stable, but may feel sluggish

## Applications

The MPU6050 sensor and this library are suitable for:

- **Robotics**
  - Self-balancing robots
  - Quadcopter/drone flight control
  - Robot arm stabilization
  - Line-following robots with tilt sensing

- **Motion Tracking**
  - Gesture recognition systems
  - Gaming controllers with motion input
  - Virtual Reality (VR) headset tracking
  - Fitness tracking devices

- **Industrial Applications**
  - Equipment vibration monitoring
  - Tilt detection for safety systems
  - Platform leveling systems
  - Orientation monitoring

- **Education**
  - Physics experiments (acceleration, rotation)
  - Signal processing demonstrations
  - Sensor fusion algorithm learning
  - Embedded systems programming

## Troubleshooting

### Sensor Not Detected
- Check wiring connections (VCC, GND, SDA, SCL)
- Verify power supply is 3.3V
- Ensure I2C address is correct (0x68 or 0x69)
- Check if pull-up resistors are present on I2C lines

### Erratic Readings
- Ensure stable power supply
- Check for loose connections
- Keep sensor away from electromagnetic interference
- Allow sensor to stabilize after power-on (100ms)

### Drift in Angles
- Adjust filter coefficient (lower value = less drift)
- Reset filter when orientation is known
- Ensure sensor is not subject to constant acceleration
- Check for sensor mounting vibrations

### I2C Communication Errors
- Reduce I2C frequency if using long wires
- Add or check pull-up resistors (4.7kΩ typical)
- Verify GPIO pins are correct for your board
- Try different I2C controller (i2c_id=1)

## Technical Specifications

### MPU6050 Sensor
- **Communication:** I2C (up to 400kHz)
- **Power Supply:** 2.375V - 3.46V (3.3V recommended)
- **Operating Current:** 3.9mA (normal mode)
- **Sleep Current:** 5µA
- **Operating Temperature:** -40°C to +85°C

### Accelerometer
- **Full Scale Range:** ±2g, ±4g, ±8g, ±16g (±2g used in this library)
- **Sensitivity:** 16384 LSB/g (at ±2g)
- **Non-linearity:** ±0.5% (best fit straight line)

### Gyroscope
- **Full Scale Range:** ±250°/s, ±500°/s, ±1000°/s, ±2000°/s (±250°/s used)
- **Sensitivity:** 131 LSB/°/s (at ±250°/s)
- **Non-linearity:** ±0.2% (best fit straight line)

### Temperature Sensor
- **Range:** -40°C to +85°C
- **Sensitivity:** 340 LSB/°C
- **Accuracy:** ±1°C (typical)

## Performance Notes

### Raspberry Pi Pico 2 W Specific
- Supports I2C frequencies up to 1MHz (400kHz recommended)
- Hardware I2C with DMA support for efficient transfers
- 520KB SRAM allows for extensive data buffering
- Dual-core operation possible for parallel sensor processing

### Update Rates
- Maximum sensor data rate: 1kHz (when DLPF enabled)
- Configured sample rate: 125Hz (good balance)
- Recommended read rate: 10-100Hz for most applications
- Filter update requires consistent timing for best results

### Memory Usage
- Handler class: ~200 bytes
- Frame buffer per reading: ~100 bytes
- Total library code: ~15KB

## Examples and Tests

Run the comprehensive test program to verify functionality:

```python
# Upload all three files to your Pico
# Run the test program
import mpu6050_test
mpu6050_test.main()
```

The test program demonstrates:
1. Basic sensor reading
2. Angle calculations
3. Comprehensive data access
4. Configuration options
5. Real-time monitoring (10 seconds)

## References

- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- [MicroPython I2C Documentation](https://docs.micropython.org/en/latest/library/machine.I2C.html)

## Contributing

This library is part of a larger MicroPython project for Raspberry Pi Pico. Improvements and bug fixes are welcome.

## Version History

- **Version 1.0** (October 19, 2025)
  - Initial release
  - Complete MPU6050 driver implementation
  - Handler class with sensor fusion
  - Comprehensive test suite
  - Full documentation

## Author

**Ojas Jha**  
License: MIT License  
Date: October 19, 2025

---

For more information about MicroPython on Raspberry Pi Pico, visit:
- [MicroPython Official Documentation](https://docs.micropython.org/)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)

