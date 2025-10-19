"""
MPU6050 6-Axis Motion Sensor Library for MicroPython on Raspberry Pi Pico 2 W

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

OVERVIEW:
=========
This library provides a complete interface for the MPU6050 Inertial Measurement Unit (IMU),
a popular 6-axis motion tracking device that combines:
- 3-axis accelerometer (measures linear acceleration in g-force)
- 3-axis gyroscope (measures angular velocity in degrees per second)
- Built-in temperature sensor

HARDWARE COMMUNICATION:
======================
The MPU6050 communicates via I2C (Inter-Integrated Circuit) protocol:
- I2C is a 2-wire serial communication protocol (SDA for data, SCL for clock)
- Default I2C address: 0x68 (can be changed to 0x69 with AD0 pin high)
- Raspberry Pi Pico 2 W has two I2C controllers (I2C0 and I2C1)
- This library uses configurable pins for SDA/SCL connections

SENSOR DATA FORMAT:
==================
Raw sensor data is stored in 16-bit signed integers (two's complement format):
- Range: -32,768 to +32,767
- High byte and low byte are stored in consecutive registers
- Conversion factors are applied to get real-world units

REGISTER MAP:
============
The MPU6050 uses memory-mapped registers for configuration and data access:
- Each register has a unique 8-bit address
- Data registers store sensor readings
- Configuration registers control sensor behavior

Author: Ojas Jha
License: MIT License (see above)
Date: October 19, 2025
Target: Raspberry Pi Pico 2 W with MicroPython
Reference: MPU6050 Register Map and Descriptions Rev 4.2
"""

# ============================================================================
# IMPORT STATEMENTS
# ============================================================================

# MicroPython Hardware Interface Modules
from machine import Pin, I2C  # Pin: GPIO pin control, I2C: Inter-Integrated Circuit communication
import utime                  # MicroPython time utilities for delays and timing

# ============================================================================
# MPU6050 REGISTER ADDRESS CONSTANTS
# ============================================================================

"""
REGISTER ADDRESSING EXPLANATION:
Each register in the MPU6050 has a unique 8-bit address (0x00 to 0xFF).
These constants define the memory locations for different sensor functions.
The 0x prefix indicates hexadecimal notation (base-16 numbering system).
"""

# Power Management Registers
POWER_MANAGEMENT_1_REGISTER = 0x6B      # Controls sleep mode, clock source, and device reset
                                         # Bit 6: SLEEP (0=wake up, 1=sleep mode)
                                         # Default: 0x40 (sleep mode enabled)

# Sample Rate Control Registers
SAMPLE_RATE_DIVIDER_REGISTER = 0x19      # Controls the sensor sampling frequency
                                         # Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
                                         # Default gyro rate: 1kHz, so 0x07 gives 125Hz

# Digital Low Pass Filter Configuration
CONFIGURATION_REGISTER = 0x1A            # Configures external frame sync and digital low pass filter
                                         # DLPF_CFG bits control bandwidth and delay
                                         # 0x00 = 260Hz bandwidth, 0ms delay

# Sensor Configuration Registers
GYROSCOPE_CONFIGURATION_REGISTER = 0x1B  # Configures gyroscope full-scale range and self-test
                                         # FS_SEL bits: 00=±250°/s, 01=±500°/s, 10=±1000°/s, 11=±2000°/s
                                         # 0x00 = ±250°/s (most sensitive setting)

ACCELEROMETER_CONFIGURATION_REGISTER = 0x1C  # Configures accelerometer full-scale range and self-test
                                             # AFS_SEL bits: 00=±2g, 01=±4g, 10=±8g, 11=±16g
                                             # 0x00 = ±2g (most sensitive setting)

# Sensor Data Output Registers (High Byte Addresses)
TEMPERATURE_OUTPUT_HIGH_REGISTER = 0x41       # Temperature sensor high byte
                                              # 16-bit temperature data starts here (0x41-0x42)

ACCELEROMETER_X_OUTPUT_HIGH_REGISTER = 0x3B   # Accelerometer X-axis high byte
                                              # 6 bytes total: ACCEL_XOUT_H/L, ACCEL_YOUT_H/L, ACCEL_ZOUT_H/L
                                              # Registers 0x3B through 0x40

GYROSCOPE_X_OUTPUT_HIGH_REGISTER = 0x43       # Gyroscope X-axis high byte
                                              # 6 bytes total: GYRO_XOUT_H/L, GYRO_YOUT_H/L, GYRO_ZOUT_H/L
                                              # Registers 0x43 through 0x48

# ============================================================================
# SENSOR INITIALIZATION FUNCTIONS
# ============================================================================

def initialize_mpu6050_sensor(i2c_bus, device_address=0x68):
    """
    Initialize the MPU6050 sensor with optimal configuration for general motion sensing.

    WHAT THIS FUNCTION DOES:
    =======================
    1. Wakes up the MPU6050 from sleep mode (default state after power-on)
    2. Configures sample rate for 125Hz data collection
    3. Sets up digital filtering to reduce noise
    4. Configures sensor ranges for maximum sensitivity

    I2C COMMUNICATION EXPLANATION:
    =============================
    - i2c_bus.writeto_mem() writes data directly to device memory registers
    - Format: writeto_mem(device_address, register_address, data_bytes)
    - data_bytes must be in bytes format (b'\x00' means byte value 0)

    RASPBERRY PI PICO I2C SETUP:
    ============================
    Before calling this function, create I2C bus like:
    i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)

    Args:
        i2c_bus (machine.I2C): I2C communication object created with machine.I2C()
                               Example: I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
        device_address (int): 7-bit I2C address of MPU6050 sensor
                             Default: 0x68 (when AD0 pin is grounded)
                             Alternative: 0x69 (when AD0 pin is high)

    Returns:
        None: Function configures the sensor but returns no value

    Raises:
        OSError: If I2C communication fails (sensor not connected or wrong address)

    Example Usage:
        >>> from machine import Pin, I2C
        >>> i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
        >>> initialize_mpu6050_sensor(i2c)
    """

    # STEP 1: Wake up the MPU6050 from sleep mode
    # ==========================================
    # The MPU6050 starts in sleep mode to save power after reset/power-on
    # Writing 0x00 to PWR_MGMT_1 clears the SLEEP bit and uses internal 8MHz oscillator
    print("Initializing MPU6050 sensor...")
    i2c_bus.writeto_mem(device_address, POWER_MANAGEMENT_1_REGISTER, b'\x00')
    utime.sleep_ms(100)  # Allow internal oscillator and sensor to stabilize (100ms recommended)

    # STEP 2: Configure sample rate divider
    # ====================================
    # Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    # Gyro output rate = 1kHz when DLPF is enabled
    # 0x07 = 7, so sample rate = 1000Hz / (1 + 7) = 125Hz
    # 125Hz provides good balance between data rate and processing load
    i2c_bus.writeto_mem(device_address, SAMPLE_RATE_DIVIDER_REGISTER, b'\x07')

    # STEP 3: Configure digital low pass filter (DLPF)
    # ===============================================
    # 0x00 sets DLPF to 260Hz bandwidth with 0ms delay
    # This reduces high-frequency noise while maintaining responsiveness
    # Lower values provide more filtering but add delay
    i2c_bus.writeto_mem(device_address, CONFIGURATION_REGISTER, b'\x00')

    # STEP 4: Configure gyroscope sensitivity
    # ======================================
    # 0x00 sets full-scale range to ±250°/s (most sensitive setting)
    # This provides 131 LSB/°/s sensitivity (131 counts per degree per second)
    # Other options: ±500°/s, ±1000°/s, ±2000°/s (less sensitive but wider range)
    i2c_bus.writeto_mem(device_address, GYROSCOPE_CONFIGURATION_REGISTER, b'\x00')

    # STEP 5: Configure accelerometer sensitivity
    # ==========================================
    # 0x00 sets full-scale range to ±2g (most sensitive setting)
    # This provides 16384 LSB/g sensitivity (16384 counts per g-force)
    # Other options: ±4g, ±8g, ±16g (less sensitive but wider range)
    i2c_bus.writeto_mem(device_address, ACCELEROMETER_CONFIGURATION_REGISTER, b'\x00')

    print("MPU6050 initialization complete!")

# ============================================================================
# LOW-LEVEL DATA READING FUNCTIONS
# ============================================================================

def read_16bit_signed_data(i2c_bus, register_address, device_address=0x68):
    """
    Read 16-bit signed integer data from consecutive MPU6050 registers.

    BINARY DATA REPRESENTATION EXPLANATION:
    ======================================
    The MPU6050 stores sensor data as 16-bit signed integers using two's complement format:
    - Positive numbers: 0 to 32,767 (0x0000 to 0x7FFF)
    - Negative numbers: -32,768 to -1 (0x8000 to 0xFFFF)
    - Each 16-bit value is split across two 8-bit registers (high byte, low byte)

    REGISTER LAYOUT:
    ===============
    High Byte Register: Contains bits 15-8 (most significant byte)
    Low Byte Register:  Contains bits 7-0 (least significant byte)
    Example: Value 1000 (0x03E8) = High:0x03, Low:0xE8

    BIT MANIPULATION EXPLANATION:
    ============================
    - Left shift (<<): Moves bits left, high_byte << 8 moves byte to upper 8 bits
    - Bitwise OR (|): Combines bits, creates 16-bit value from two 8-bit values
    - Two's complement conversion: Values > 32767 are negative numbers

    Args:
        i2c_bus (machine.I2C): I2C communication object for reading data
        register_address (int): Starting register address (high byte location)
                               Low byte will be read from register_address + 1
        device_address (int): 7-bit I2C address of MPU6050 sensor (default: 0x68)

    Returns:
        int: Signed 16-bit integer value (-32,768 to +32,767)
             Raw sensor data that needs conversion to physical units

    Raises:
        OSError: If I2C communication fails (sensor disconnected, wrong address)

    Example Usage:
        >>> # Read temperature sensor raw data
        >>> temp_raw = read_16bit_signed_data(i2c, 0x41)  # 0x41 = TEMP_OUT_H
        >>> print(f"Raw temperature: {temp_raw}")
    """

    # STEP 1: Read high byte (most significant 8 bits)
    # ===============================================
    # readfrom_mem() returns a bytes object, [0] gets first (and only) byte
    # register_address points to the high byte of the 16-bit value
    high_byte = i2c_bus.readfrom_mem(device_address, register_address, 1)[0]

    # STEP 2: Read low byte (least significant 8 bits)
    # ===============================================
    # Low byte is always stored in the next consecutive register
    low_byte = i2c_bus.readfrom_mem(device_address, register_address + 1, 1)[0]

    # STEP 3: Combine bytes into 16-bit unsigned value
    # ==============================================
    # Left shift high byte 8 positions to occupy bits 15-8
    # Bitwise OR with low byte to fill bits 7-0
    # Example: high=0x03, low=0xE8 → (0x03 << 8) | 0xE8 = 0x03E8 = 1000
    raw_value = high_byte << 8 | low_byte

    # STEP 4: Convert from unsigned to signed (two's complement)
    # ========================================================
    # In 16-bit two's complement:
    # - Values 0-32767 (0x0000-0x7FFF) are positive
    # - Values 32768-65535 (0x8000-0xFFFF) represent negative numbers
    # - To convert: subtract 65536 from values > 32767
    if raw_value > 32767:  # Check if MSB (sign bit) is set
        signed_value = raw_value - 65536  # Convert to negative value
    else:
        signed_value = raw_value  # Keep positive value

    return signed_value

# ============================================================================
# HIGH-LEVEL SENSOR DATA READING FUNCTIONS
# ============================================================================

def read_sensor_data(i2c_bus):
    """
    Read all sensor data from MPU6050 and convert to physical units.

    SENSOR DATA CONVERSION EXPLANATION:
    ==================================
    The MPU6050 provides raw 16-bit integer values that must be converted to meaningful units:

    TEMPERATURE SENSOR:
    - Raw value is proportional to temperature
    - Formula: Temperature(°C) = (Raw_Value / 340.0) + 36.53
    - This formula is from the MPU6050 datasheet

    ACCELEROMETER (measures linear acceleration):
    - Configured for ±2g full-scale range (most sensitive)
    - Sensitivity: 16384 LSB/g (Least Significant Bits per g-force)
    - Formula: Acceleration(g) = Raw_Value / 16384.0
    - 1g = 9.81 m/s² (Earth's gravitational acceleration)

    GYROSCOPE (measures angular velocity/rotation rate):
    - Configured for ±250°/s full-scale range (most sensitive)
    - Sensitivity: 131 LSB/°/s (Least Significant Bits per degree per second)
    - Formula: Angular_Velocity(°/s) = Raw_Value / 131.0

    COORDINATE SYSTEM:
    =================
    MPU6050 uses right-hand coordinate system:
    - X-axis: Points toward the "right" side of the chip
    - Y-axis: Points toward the "front" of the chip
    - Z-axis: Points "up" from the chip surface
    - Positive rotation follows right-hand rule

    Args:
        i2c_bus (machine.I2C): I2C communication object for sensor communication

    Returns:
        dict: Nested dictionary containing all sensor data in physical units
              Structure:
              {
                  'temperature': float,           # Temperature in Celsius (°C)
                  'accelerometer': {              # Linear acceleration in g-force
                      'x': float,                 # X-axis acceleration (-2g to +2g)
                      'y': float,                 # Y-axis acceleration (-2g to +2g)
                      'z': float                  # Z-axis acceleration (-2g to +2g)
                  },
                  'gyroscope': {                  # Angular velocity in degrees/second
                      'x': float,                 # X-axis rotation (-250°/s to +250°/s)
                      'y': float,                 # Y-axis rotation (-250°/s to +250°/s)
                      'z': float                  # Z-axis rotation (-250°/s to +250°/s)
                  }
              }

    Raises:
        OSError: If I2C communication fails during any sensor reading

    Example Usage:
        >>> data = read_sensor_data(i2c)
        >>> print(f"Temperature: {data['temperature']:.1f}°C")
        >>> print(f"Accel X: {data['accelerometer']['x']:.3f}g")
        >>> print(f"Gyro Z: {data['gyroscope']['z']:.1f}°/s")
    """

    # READ TEMPERATURE SENSOR
    # ======================
    # Temperature calculation from MPU6050 datasheet
    # Raw value is from internal temperature sensor, not ambient temperature
    temperature_raw = read_16bit_signed_data(i2c_bus, TEMPERATURE_OUTPUT_HIGH_REGISTER)
    temperature_celsius = temperature_raw / 340.0 + 36.53

    # READ ACCELEROMETER DATA (3-axis linear acceleration)
    # ==================================================
    # Each axis is stored in consecutive register pairs (high byte, low byte)
    # Register layout: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L
    acceleration_x_raw = read_16bit_signed_data(i2c_bus, ACCELEROMETER_X_OUTPUT_HIGH_REGISTER)
    acceleration_y_raw = read_16bit_signed_data(i2c_bus, ACCELEROMETER_X_OUTPUT_HIGH_REGISTER + 2)
    acceleration_z_raw = read_16bit_signed_data(i2c_bus, ACCELEROMETER_X_OUTPUT_HIGH_REGISTER + 4)

    # Convert raw values to g-force units (1g = Earth's gravity)
    acceleration_x_g = acceleration_x_raw / 16384.0  # ±2g range: 16384 LSB/g
    acceleration_y_g = acceleration_y_raw / 16384.0
    acceleration_z_g = acceleration_z_raw / 16384.0

    # READ GYROSCOPE DATA (3-axis angular velocity)
    # ============================================
    # Each axis is stored in consecutive register pairs (high byte, low byte)
    # Register layout: GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    gyroscope_x_raw = read_16bit_signed_data(i2c_bus, GYROSCOPE_X_OUTPUT_HIGH_REGISTER)
    gyroscope_y_raw = read_16bit_signed_data(i2c_bus, GYROSCOPE_X_OUTPUT_HIGH_REGISTER + 2)
    gyroscope_z_raw = read_16bit_signed_data(i2c_bus, GYROSCOPE_X_OUTPUT_HIGH_REGISTER + 4)

    # Convert raw values to degrees per second
    gyroscope_x_dps = gyroscope_x_raw / 131.0  # ±250°/s range: 131 LSB/°/s
    gyroscope_y_dps = gyroscope_y_raw / 131.0
    gyroscope_z_dps = gyroscope_z_raw / 131.0

    # RETURN STRUCTURED DATA
    # =====================
    # Organize all sensor data into a nested dictionary for easy access
    return {
        'temperature': temperature_celsius,
        'accelerometer': {
            'x': acceleration_x_g,
            'y': acceleration_y_g,
            'z': acceleration_z_g,
        },
        'gyroscope': {
            'x': gyroscope_x_dps,
            'y': gyroscope_y_dps,
            'z': gyroscope_z_dps,
        }
    }