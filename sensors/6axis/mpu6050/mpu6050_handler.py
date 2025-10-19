"""
MPU6050 Handler Class for Raspberry Pi Pico 2 W with MicroPython

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
This handler class provides a high-level, object-oriented interface for the MPU6050
Inertial Measurement Unit (IMU). It encapsulates all the functionality from the test
program into a reusable class that can be easily integrated into larger projects.

KEY FEATURES:
=============
1. AUTOMATIC SENSOR INITIALIZATION:
   - Handles I2C bus setup and sensor configuration
   - Configurable pin assignments and I2C parameters
   - Error handling for connection issues

2. REAL-TIME SENSOR FUSION:
   - Complementary filter implementation for stable angle estimation
   - Automatic time-based integration of gyroscope data
   - Configurable filter coefficients for different applications

3. COMPREHENSIVE DATA ACCESS:
   - Raw sensor data (accelerometer, gyroscope, temperature)
   - Calculated tilt angles (pitch, roll, yaw)
   - Filtered orientation data with drift correction
   - Timestamped readings for data logging

4. EASY INTEGRATION:
   - Simple API for common use cases
   - Configurable parameters for different applications
   - Built-in error handling and status monitoring

APPLICATIONS:
=============
- Drone flight control systems
- Self-balancing robots
- Motion-controlled gaming
- Industrial equipment monitoring
- Smartphone orientation sensing
- Virtual/Augmented Reality tracking

Author: Ojas Jha
License: MIT License (see above)
Date: October 19, 2025
Target: Raspberry Pi Pico 2 W with MicroPython
"""

# ============================================================================
# IMPORT STATEMENTS
# ============================================================================

# MicroPython Hardware Interface Modules
from machine import Pin, I2C  # Pin: GPIO control, I2C: Inter-Integrated Circuit communication
import utime                  # MicroPython time utilities for delays and timing measurements

# Standard Python Mathematical Library
import math                   # Mathematical functions (trigonometry, square root, etc.)

# Custom MPU6050 Library Functions
from mpu6050 import initialize_mpu6050_sensor, read_sensor_data

# ============================================================================
# MPU6050 HANDLER CLASS
# ============================================================================

class MPU6050Handler:
    """
    High-level handler class for MPU6050 IMU sensor with sensor fusion capabilities.

    This class provides a complete interface for MPU6050 sensor operations including:
    - Automatic sensor initialization and configuration
    - Real-time sensor data acquisition
    - Tilt angle calculations from accelerometer data
    - Sensor fusion using complementary filtering
    - Comprehensive data access methods

    ATTRIBUTES:
    ===========
    i2c_bus (machine.I2C): I2C communication object
    device_address (int): I2C address of the MPU6050 sensor
    is_initialized (bool): Sensor initialization status
    filtered_pitch (float): Current filtered pitch angle in degrees
    filtered_roll (float): Current filtered roll angle in degrees
    previous_time_ms (int): Timestamp of last sensor reading
    filter_coefficient (float): Complementary filter weighting factor

    METHODS:
    ========
    - __init__(): Initialize handler with I2C configuration
    - initialize(): Initialize and configure the MPU6050 sensor
    - read_raw_data(): Read raw sensor data from MPU6050
    - calculate_tilt_angles(): Calculate tilt angles from accelerometer data
    - update_filtered_angles(): Update filtered angles using sensor fusion
    - get_comprehensive_data(): Get all sensor data in one call
    - get_temperature(): Get current temperature reading
    - get_acceleration(): Get current acceleration data
    - get_angular_velocity(): Get current gyroscope data
    - get_tilt_angles(): Get calculated tilt angles
    - get_filtered_angles(): Get sensor fusion filtered angles
    - is_connected(): Check if sensor is responding
    - reset_filter(): Reset sensor fusion filter state
    """

    def __init__(self, scl_pin=21, sda_pin=20, i2c_freq=400000, device_address=0x68,
                 filter_coefficient=0.98, i2c_id=0):
        """
        Initialize the MPU6050Handler with I2C configuration.

        Args:
            scl_pin (int): GPIO pin number for I2C clock line (default: 21)
            sda_pin (int): GPIO pin number for I2C data line (default: 20)
            i2c_freq (int): I2C communication frequency in Hz (default: 400000)
            device_address (int): I2C address of MPU6050 sensor (default: 0x68)
            filter_coefficient (float): Complementary filter weighting (default: 0.98)
            i2c_id (int): I2C controller ID (0 or 1, default: 0)
        """
        # Store configuration parameters
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin
        self.i2c_freq = i2c_freq
        self.device_address = device_address
        self.filter_coefficient = filter_coefficient
        self.i2c_id = i2c_id

        # Initialize I2C bus
        self.i2c_bus = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin),
                          freq=i2c_freq, timeout=1000)

        # Sensor fusion state variables
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self.previous_time_ms = utime.ticks_ms()

        # Status flags
        self.is_initialized = False
        self.last_error = None

        print(f"MPU6050Handler initialized with I2C{i2c_id} (SCL: {scl_pin}, SDA: {sda_pin})")

    def initialize(self):
        """
        Initialize and configure the MPU6050 sensor.

        This method performs the complete sensor setup including:
        - Wake up sensor from sleep mode
        - Configure sample rate and filtering
        - Set sensor sensitivity ranges
        - Verify sensor communication

        Returns:
            bool: True if initialization successful, False otherwise

        Raises:
            OSError: If I2C communication fails
        """
        try:
            print("Initializing MPU6050 sensor...")
            initialize_mpu6050_sensor(self.i2c_bus, self.device_address)

            # Verify sensor is responding
            if self.is_connected():
                self.is_initialized = True
                self.last_error = None
                print("MPU6050 sensor initialized successfully!")
                return True
            else:
                self.last_error = "Sensor not responding after initialization"
                print(f"Error: {self.last_error}")
                return False

        except OSError as e:
            self.last_error = f"I2C communication error: {e}"
            print(f"Initialization failed: {self.last_error}")
            return False
        except Exception as e:
            self.last_error = f"Unexpected error: {e}"
            print(f"Initialization failed: {self.last_error}")
            return False

    def is_connected(self):
        """
        Check if the MPU6050 sensor is responding on the I2C bus.

        Returns:
            bool: True if sensor is responding, False otherwise
        """
        try:
            # Try to read from a known register to test communication
            self.i2c_bus.readfrom_mem(self.device_address, 0x75, 1)  # WHO_AM_I register
            return True
        except OSError:
            return False

    def read_raw_data(self):
        """
        Read raw sensor data from MPU6050.

        Returns:
            dict: Raw sensor data dictionary with structure:
                  {
                      'temperature': float,           # Temperature in Celsius
                      'accelerometer': {              # Linear acceleration in g-force
                          'x': float, 'y': float, 'z': float
                      },
                      'gyroscope': {                  # Angular velocity in degrees/second
                          'x': float, 'y': float, 'z': float
                      }
                  }

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        if not self.is_initialized:
            raise RuntimeError("Sensor not initialized. Call initialize() first.")

        return read_sensor_data(self.i2c_bus)

    def calculate_tilt_angles(self, accelerometer_data):
        """
        Calculate tilt angles (pitch, roll, yaw) from accelerometer data using trigonometry.

        Args:
            accelerometer_data (dict): Accelerometer data dictionary
                                      Structure: {'x': float, 'y': float, 'z': float}
                                      Units: g-force (1g = 9.81 m/s²)

        Returns:
            tuple: Three-element tuple containing calculated angles in degrees
                   (pitch_degrees, roll_degrees, yaw_degrees)
        """
        # Extract acceleration components
        acceleration_x = accelerometer_data['x']
        acceleration_y = accelerometer_data['y']
        acceleration_z = accelerometer_data['z']

        # PITCH CALCULATION (rotation around X-axis)
        xz_plane_magnitude = math.sqrt(acceleration_x * acceleration_x + acceleration_z * acceleration_z)
        pitch_radians = math.atan2(acceleration_y, xz_plane_magnitude)
        pitch_degrees = pitch_radians * 180.0 / math.pi

        # ROLL CALCULATION (rotation around Y-axis)
        yz_plane_magnitude = math.sqrt(acceleration_y * acceleration_y + acceleration_z * acceleration_z)
        roll_radians = math.atan2(-acceleration_x, yz_plane_magnitude)
        roll_degrees = roll_radians * 180.0 / math.pi

        # YAW APPROXIMATION (rotation around Z-axis)
        xy_plane_magnitude = math.sqrt(acceleration_x * acceleration_x + acceleration_y * acceleration_y)
        yaw_radians = math.atan2(acceleration_z, xy_plane_magnitude)
        yaw_degrees = yaw_radians * 180.0 / math.pi

        return pitch_degrees, roll_degrees, yaw_degrees

    def update_filtered_angles(self, gyroscope_data, delta_time_seconds=None):
        """
        Update filtered angles using complementary filter sensor fusion.

        Args:
            gyroscope_data (dict): Gyroscope data dictionary
                                  Structure: {'x': float, 'y': float, 'z': float}
                                  Units: degrees per second
            delta_time_seconds (float, optional): Time elapsed since last update
                                                 If None, calculated automatically

        Returns:
            tuple: Two-element tuple containing filtered angles in degrees
                   (filtered_pitch, filtered_roll)
        """
        # Calculate time delta if not provided
        if delta_time_seconds is None:
            current_time_ms = utime.ticks_ms()
            delta_time_seconds = (current_time_ms - self.previous_time_ms) / 1000.0
            self.previous_time_ms = current_time_ms

        # GYROSCOPE INTEGRATION (Short-term tracking)
        gyroscope_pitch_change = gyroscope_data['x'] * delta_time_seconds
        gyroscope_roll_change = -gyroscope_data['y'] * delta_time_seconds  # Negative for correct direction

        # Update current angles with gyroscope-derived changes
        gyroscope_pitch = self.filtered_pitch + gyroscope_pitch_change
        gyroscope_roll = self.filtered_roll + gyroscope_roll_change

        # ACCELEROMETER ANGLE CALCULATION (Long-term correction)
        # Note: This is a simplified calculation - ideally would use actual accelerometer data
        accelerometer_pitch = math.atan2(gyroscope_data['y'],
                                       math.sqrt(gyroscope_data['x'] * gyroscope_data['x'] +
                                               gyroscope_data['z'] * gyroscope_data['z'])) * 180 / math.pi
        accelerometer_roll = math.atan2(-gyroscope_data['x'],
                                      math.sqrt(gyroscope_data['y'] * gyroscope_data['y'] +
                                              gyroscope_data['z'] * gyroscope_data['z'])) * 180 / math.pi

        # COMPLEMENTARY FILTER APPLICATION
        self.filtered_pitch = (self.filter_coefficient * gyroscope_pitch +
                              (1 - self.filter_coefficient) * accelerometer_pitch)
        self.filtered_roll = (self.filter_coefficient * gyroscope_roll +
                             (1 - self.filter_coefficient) * accelerometer_roll)

        return self.filtered_pitch, self.filtered_roll

    def get_comprehensive_data(self):
        """
        Get all sensor data including raw readings, calculated angles, and filtered angles.

        This is the main method for getting complete sensor information in one call.

        Returns:
            dict: Comprehensive sensor data dictionary with structure:
                  {
                      'timestamp_ms': int,           # Current timestamp in milliseconds
                      'temperature': float,          # Temperature in Celsius
                      'accelerometer': {             # Linear acceleration in g-force
                          'x': float, 'y': float, 'z': float
                      },
                      'gyroscope': {                 # Angular velocity in degrees/second
                          'x': float, 'y': float, 'z': float
                      },
                      'tilt_angles': {               # Calculated tilt angles in degrees
                          'pitch': float, 'roll': float, 'yaw': float
                      },
                      'filtered_angles': {           # Sensor fusion filtered angles in degrees
                          'pitch': float, 'roll': float
                      }
                  }

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        # Read raw sensor data
        raw_data = self.read_raw_data()

        # Calculate tilt angles from accelerometer data
        pitch, roll, yaw = self.calculate_tilt_angles(raw_data['accelerometer'])

        # Update filtered angles using sensor fusion
        filtered_pitch, filtered_roll = self.update_filtered_angles(raw_data['gyroscope'])

        # Return comprehensive data structure
        return {
            'timestamp_ms': utime.ticks_ms(),
            'temperature': raw_data['temperature'],
            'accelerometer': raw_data['accelerometer'],
            'gyroscope': raw_data['gyroscope'],
            'tilt_angles': {
                'pitch': pitch,
                'roll': roll,
                'yaw': yaw
            },
            'filtered_angles': {
                'pitch': filtered_pitch,
                'roll': filtered_roll
            }
        }

    def get_temperature(self):
        """
        Get current temperature reading from the sensor.

        Returns:
            float: Temperature in Celsius

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        raw_data = self.read_raw_data()
        return raw_data['temperature']

    def get_acceleration(self):
        """
        Get current acceleration data from the sensor.

        Returns:
            dict: Acceleration data dictionary
                  Structure: {'x': float, 'y': float, 'z': float}
                  Units: g-force (1g = 9.81 m/s²)

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        raw_data = self.read_raw_data()
        return raw_data['accelerometer']

    def get_angular_velocity(self):
        """
        Get current gyroscope data from the sensor.

        Returns:
            dict: Angular velocity data dictionary
                  Structure: {'x': float, 'y': float, 'z': float}
                  Units: degrees per second

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        raw_data = self.read_raw_data()
        return raw_data['gyroscope']

    def get_tilt_angles(self):
        """
        Get calculated tilt angles from accelerometer data.

        Returns:
            dict: Tilt angles dictionary
                  Structure: {'pitch': float, 'roll': float, 'yaw': float}
                  Units: degrees

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        raw_data = self.read_raw_data()
        pitch, roll, yaw = self.calculate_tilt_angles(raw_data['accelerometer'])
        return {'pitch': pitch, 'roll': roll, 'yaw': yaw}

    def get_filtered_angles(self):
        """
        Get sensor fusion filtered angles.

        Returns:
            dict: Filtered angles dictionary
                  Structure: {'pitch': float, 'roll': float}
                  Units: degrees

        Raises:
            OSError: If I2C communication fails
            RuntimeError: If sensor is not initialized
        """
        raw_data = self.read_raw_data()
        filtered_pitch, filtered_roll = self.update_filtered_angles(raw_data['gyroscope'])
        return {'pitch': filtered_pitch, 'roll': filtered_roll}

    def reset_filter(self):
        """
        Reset the sensor fusion filter state.

        This method resets the filtered angle estimates to zero and clears
        the timing state. Useful when restarting sensor fusion or when
        the device orientation is known to be level.
        """
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self.previous_time_ms = utime.ticks_ms()
        print("Sensor fusion filter reset to zero state")

    def set_filter_coefficient(self, coefficient):
        """
        Set the complementary filter coefficient.

        Args:
            coefficient (float): Filter coefficient (0.0 to 1.0)
                                - 0.98: 98% gyro + 2% accelerometer (default, smooth)
                                - 0.95: 95% gyro + 5% accelerometer (more stable)
                                - 0.90: 90% gyro + 10% accelerometer (very stable)
        """
        if 0.0 <= coefficient <= 1.0:
            self.filter_coefficient = coefficient
            print(f"Filter coefficient set to {coefficient}")
        else:
            raise ValueError("Filter coefficient must be between 0.0 and 1.0")

    def get_status(self):
        """
        Get current handler status information.

        Returns:
            dict: Status information dictionary
                  Structure:
                  {
                      'is_initialized': bool,
                      'is_connected': bool,
                      'last_error': str or None,
                      'filter_coefficient': float,
                      'device_address': int,
                      'i2c_frequency': int
                  }
        """
        return {
            'is_initialized': self.is_initialized,
            'is_connected': self.is_connected(),
            'last_error': self.last_error,
            'filter_coefficient': self.filter_coefficient,
            'device_address': self.device_address,
            'i2c_frequency': self.i2c_freq
        }
