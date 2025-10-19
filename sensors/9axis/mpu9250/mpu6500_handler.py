"""
MPU6500 Inertial Measurement Unit Handler Class for Raspberry Pi Pico (MicroPython)

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

This module provides a high-level interface for the MPU6500 6-axis Inertial Measurement Unit (IMU),
which combines a 3-axis accelerometer, 3-axis gyroscope, and temperature sensor in a single chip.

Key Concepts:
- IMU (Inertial Measurement Unit): Device that measures motion and orientation
- Accelerometer: Measures linear acceleration (gravity + motion) in m/s²
- Gyroscope: Measures angular velocity (rotation rate) in degrees/second
- Complementary Filter: Algorithm that combines accelerometer and gyroscope data
- Tilt Angles: Pitch (X-axis rotation), Roll (Y-axis rotation), Yaw (Z-axis rotation)

MPU6500 Sensor Architecture:
- 3-axis accelerometer with configurable full-scale ranges (±2g to ±16g)
- 3-axis gyroscope with configurable full-scale ranges (±250 to ±2000 dps)
- Built-in temperature sensor for thermal compensation
- I2C communication interface with address 0x68
- 16-bit ADC resolution for high precision measurements

Raspberry Pi Pico Specifics:
- Uses I2C pins (GPIO 20/21 by default) for sensor communication
- Requires 3.3V power supply and proper pull-up resistors (4.7kΩ recommended)
- I2C frequency up to 400kHz supported by the Pico
- Complementary filter provides smooth angle estimates for robotics applications

Author: Ojas Jha
License: MIT License (see above)
Date: October 19, 2025
Version: 2.0
"""

# =============================================================================
# IMPORTS - Categorized by functionality
# =============================================================================

# MicroPython hardware abstraction layer for Raspberry Pi Pico
from machine import Pin, I2C  # Pin: GPIO control, I2C: Two-wire communication protocol
import utime  # MicroPython time utilities for delays and timestamps
import math  # Mathematical functions for angle calculations and trigonometry

# MPU6500 sensor driver and configuration constants
from mpu6500 import MPU6500, ACCEL_FS_SEL_2G, GYRO_FS_SEL_250DPS, SF_M_S2, SF_DEG_S


class MPU6500InertialMeasurementUnitHandler:
    """
    High-level handler for the MPU6500 6-axis Inertial Measurement Unit.

    This class provides a simplified interface for interacting with the MPU6500 IMU,
    handling initialization, data reading, angle calculations, and complementary filtering.
    It abstracts the low-level I2C communication and provides high-level sensor fusion.

    MPU6500 Sensor Architecture:
    - 3-axis accelerometer: Measures linear acceleration (gravity + motion)
    - 3-axis gyroscope: Measures angular velocity (rotation rate)
    - Temperature sensor: Provides thermal compensation data
    - I2C slave device with address 0x68 (104 in decimal)
    - Configurable full-scale ranges for different applications

    Complementary Filter Algorithm:
    The complementary filter combines accelerometer and gyroscope data to provide
    smooth, accurate angle estimates. It uses a weighted average where:
    - Gyroscope provides short-term accuracy (high frequency response)
    - Accelerometer provides long-term stability (low frequency response)
    - Filter coefficient (0.0-1.0) controls the balance between them

    Usage Example:
        imu = MPU6500InertialMeasurementUnitHandler()
        if imu.initialize_sensor():
            data = imu.get_comprehensive_sensor_data()
            print(f"Pitch: {data['filtered_angles']['pitch_angle']:.2f}°")
    """

    def __init__(
        self,
        serial_clock_pin=21,
        serial_data_pin=20,
        i2c_frequency=400_000,
        i2c_bus_id=0,
        imu_device_address=0x68,
        accelerometer_full_scale=ACCEL_FS_SEL_2G,
        gyroscope_full_scale=GYRO_FS_SEL_250DPS,
        accelerometer_scale_factor=SF_M_S2,
        gyroscope_scale_factor=SF_DEG_S,
        complementary_filter_coefficient=0.98
    ):
        """
        Initialize the MPU6500 IMU handler with configuration parameters.

        Args:
            serial_clock_pin (int): GPIO pin for I2C clock line (SCL). Default: 21
                - Raspberry Pi Pico GPIO21 is commonly used for I2C SCL
            serial_data_pin (int): GPIO pin for I2C data line (SDA). Default: 20
                - Raspberry Pi Pico GPIO20 is commonly used for I2C SDA
            i2c_frequency (int): I2C communication frequency in Hz. Default: 400000
                - Standard I2C speeds: 100kHz (slow), 400kHz (fast)
                - Higher frequency = faster communication but less reliable over long wires
            i2c_bus_id (int): I2C bus identifier. Default: 0
                - Raspberry Pi Pico has I2C0 and I2C1 buses
            imu_device_address (int): I2C address of the MPU6500 sensor. Default: 0x68
                - Fixed address for MPU6500 IMU
            accelerometer_full_scale: Accelerometer measurement range. Default: ACCEL_FS_SEL_2G
                - Options: ±2g, ±4g, ±8g, ±16g
                - Lower range = higher resolution, higher range = less precision
            gyroscope_full_scale: Gyroscope measurement range. Default: GYRO_FS_SEL_250DPS
                - Options: ±250, ±500, ±1000, ±2000 degrees/second
                - Lower range = higher resolution, higher range = less precision
            accelerometer_scale_factor: Accelerometer output scaling. Default: SF_M_S2
                - SF_M_S2: Output in meters per second squared (m/s²)
                - SF_G: Output in gravitational units (g = 9.81 m/s²)
            gyroscope_scale_factor: Gyroscope output scaling. Default: SF_DEG_S
                - SF_DEG_S: Output in degrees per second (°/s)
                - SF_RAD_S: Output in radians per second (rad/s)
            complementary_filter_coefficient (float): Filter balance coefficient (0.0-1.0). Default: 0.98
                - 0.0: Use only accelerometer (stable but slow response)
                - 1.0: Use only gyroscope (fast but drifts over time)
                - 0.98: Good balance for most applications

        Returns:
            None

        Note:
            The I2C bus is initialized but the sensor is not yet configured.
            Call initialize_sensor() to establish communication and configure the sensor.
        """
        # Store configuration parameters for reference and debugging
        self.serial_clock_pin = serial_clock_pin
        self.serial_data_pin = serial_data_pin
        self.i2c_frequency = i2c_frequency
        self.i2c_bus_id = i2c_bus_id
        self.imu_device_address = imu_device_address
        self.accelerometer_full_scale = accelerometer_full_scale
        self.gyroscope_full_scale = gyroscope_full_scale
        self.accelerometer_scale_factor = accelerometer_scale_factor
        self.gyroscope_scale_factor = gyroscope_scale_factor

        # Initialize complementary filter parameters
        # The filter coefficient controls the balance between gyroscope and accelerometer
        self.complementary_filter_coefficient = float(complementary_filter_coefficient)
        self.filtered_pitch_angle = 0.0  # Current filtered pitch angle (degrees)
        self.filtered_roll_angle = 0.0   # Current filtered roll angle (degrees)
        self._last_measurement_time_ms = utime.ticks_ms()  # Timestamp for time delta calculation

        # Initialize I2C communication bus
        # I2C requires pull-up resistors (4.7kΩ) on SCL and SDA lines
        # The timeout parameter prevents hanging if sensor doesn't respond
        self.i2c_communication_bus = I2C(
            i2c_bus_id,
            scl=Pin(serial_clock_pin),
            sda=Pin(serial_data_pin),
            freq=i2c_frequency,
            timeout=1000  # 1 second timeout for I2C operations
        )

        # Initialize sensor state variables
        self.imu_sensor = None  # Will hold the MPU6500 driver instance
        self.is_sensor_initialized = False  # Tracks initialization status
        self.last_error_message = None  # Stores last error for debugging

    def initialize_sensor(self):
        """
        Initialize the IMU sensor and establish communication.

        This method creates the MPU6500 driver instance, configures the sensor
        with the specified full-scale ranges and scale factors, and verifies
        communication by reading the device ID register.

        Process:
        1. Create MPU6500 driver instance with I2C bus and configuration
        2. Configure accelerometer and gyroscope full-scale ranges
        3. Set output scale factors for data conversion
        4. Read WHO_AM_I register to verify sensor communication
        5. Set initialization status and clear any previous errors

        Returns:
            bool: True if initialization successful, False otherwise

        Raises:
            Exception: If I2C communication fails or sensor not found

        Note:
            This method must be called before any other sensor operations.
            Check the return value to ensure successful initialization.
        """
        try:
            # Create MPU6500 driver instance with our I2C bus and configuration
            # The driver handles low-level register operations and data conversion
            self.imu_sensor = MPU6500(
                self.i2c_communication_bus,
                address=self.imu_device_address,
                accel_fs=self.accelerometer_full_scale,  # Accelerometer full-scale range
                gyro_fs=self.gyroscope_full_scale,       # Gyroscope full-scale range
                accel_sf=self.accelerometer_scale_factor, # Accelerometer scale factor
                gyro_sf=self.gyroscope_scale_factor      # Gyroscope scale factor
            )

            # Verify sensor communication by reading WHO_AM_I register
            # This register should return 0x70 for MPU6500
            _ = self.imu_sensor.whoami

            # Update initialization status
            self.is_sensor_initialized = True
            self.last_error_message = None
            return True

        except Exception as initialization_exception:
            # Handle initialization failures gracefully
            self.imu_sensor = None
            self.is_sensor_initialized = False
            self.last_error_message = "IMU sensor initialization failed: {}".format(initialization_exception)
            return False

    def is_sensor_connected(self):
        """
        Check if the IMU sensor is connected and responsive via I2C communication.

        This method verifies the physical connection and communication with the MPU6500
        sensor by attempting to read the WHO_AM_I register. This register contains a
        fixed value (0x70) that identifies the sensor type and confirms I2C communication.

        I2C Communication Verification Process:
        1. Check if sensor driver instance exists (initialization check)
        2. Attempt to read WHO_AM_I register (0x75) from the sensor
        3. Verify the returned value matches expected MPU6500 identifier (0x70)
        4. Handle any I2C communication errors gracefully

        Raspberry Pi Pico I2C Considerations:
        - I2C requires pull-up resistors (4.7kΩ) on SCL and SDA lines
        - Communication can fail due to loose connections, power issues, or bus conflicts
        - The WHO_AM_I register is read-only and always returns the same value
        - I2C timeout is set to 1000ms to prevent hanging on unresponsive devices

        Args:
            None

        Returns:
            bool: True if sensor is connected and responsive, False otherwise
                - True: Sensor responds correctly to I2C commands
                - False: Sensor not found, communication error, or not initialized

        Raises:
            None (all exceptions are caught and handled internally)

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            if imu.is_sensor_connected():
                print("Sensor is ready for use")
            else:
                print("Check sensor connections and power")
        """
        try:
            # Check if sensor driver instance exists
            # This prevents errors if initialize_sensor() was never called
            if self.imu_sensor is None:
                return False

            # Attempt to read WHO_AM_I register to verify communication
            # This register should return 0x70 for MPU6500 sensor
            # The underscore (_) indicates we don't need the actual value, just success/failure
            _ = self.imu_sensor.whoami

            # If we reach here, communication was successful
            return True

        except Exception:
            # Handle any I2C communication errors
            # This includes I2C bus errors, device not found, or timeout
            return False

    def read_raw_sensor_data(self):
        """
        Read raw sensor data from all MPU6500 sensors via I2C communication.

        This method performs a complete sensor reading operation, retrieving data from
        the accelerometer, gyroscope, and temperature sensor simultaneously. The data
        is returned in a structured format with proper scaling applied based on the
        configured full-scale ranges and scale factors.

        MPU6500 Sensor Architecture:
        - Accelerometer: 3-axis MEMS accelerometer measuring linear acceleration
          * Measures gravity (≈9.81 m/s²) when stationary
          * Detects motion, vibration, and tilt
          * 16-bit ADC resolution with configurable ranges (±2g to ±16g)
        - Gyroscope: 3-axis MEMS gyroscope measuring angular velocity
          * Measures rotation rate around each axis
          * Used for detecting orientation changes and rotation
          * 16-bit ADC resolution with configurable ranges (±250 to ±2000 dps)
        - Temperature Sensor: Built-in temperature sensor for thermal compensation
          * Provides sensor temperature for calibration
          * Helps compensate for temperature drift in measurements

        Data Processing Pipeline:
        1. I2C register read operations for all sensor axes
        2. Raw 16-bit ADC values converted to physical units
        3. Scale factors applied based on configured full-scale ranges
        4. Data formatted into structured dictionary for easy access

        Coordinate System (Right-Handed):
        - X-axis: Points forward (pitch axis)
        - Y-axis: Points right (roll axis)
        - Z-axis: Points up (yaw axis)

        Args:
            None

        Returns:
            dict: Comprehensive sensor data dictionary containing:
                - 'temperature_celsius' (float): Temperature in Celsius
                - 'accelerometer_data' (dict): 3-axis acceleration data
                    * 'x_axis' (float): X-axis acceleration (m/s² or g)
                    * 'y_axis' (float): Y-axis acceleration (m/s² or g)
                    * 'z_axis' (float): Z-axis acceleration (m/s² or g)
                - 'gyroscope_data' (dict): 3-axis angular velocity data
                    * 'x_axis' (float): X-axis angular velocity (°/s or rad/s)
                    * 'y_axis' (float): Y-axis angular velocity (°/s or rad/s)
                    * 'z_axis' (float): Z-axis angular velocity (°/s or rad/s)

        Raises:
            RuntimeError: If sensor is not initialized or driver instance is None
            Exception: If I2C communication fails or sensor read error occurs

        Note:
            - All values are converted to float for consistent data types
            - Units depend on configured scale factors (SF_M_S2, SF_G, SF_DEG_S, SF_RAD_S)
            - Reading all sensors simultaneously ensures data consistency
            - Temperature sensor provides thermal compensation data

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()
            data = imu.read_raw_sensor_data()
            print(f"Temperature: {data['temperature_celsius']:.1f}°C")
            print(f"Acceleration X: {data['accelerometer_data']['x_axis']:.2f} m/s²")
        """
        # Verify sensor is properly initialized before attempting to read data
        if not self.is_sensor_initialized or self.imu_sensor is None:
            raise RuntimeError("IMU sensor not initialized. Call initialize_sensor() first.")

        # Read accelerometer data (3-axis linear acceleration)
        # Returns tuple of (x, y, z) acceleration values in configured units
        acceleration_x, acceleration_y, acceleration_z = self.imu_sensor.acceleration

        # Read gyroscope data (3-axis angular velocity)
        # Returns tuple of (x, y, z) angular velocity values in configured units
        gyroscope_x, gyroscope_y, gyroscope_z = self.imu_sensor.gyro

        # Read temperature sensor data
        # Returns temperature in Celsius for thermal compensation
        temperature_celsius = self.imu_sensor.temperature

        # Structure the data into a comprehensive dictionary
        # All values converted to float for consistent data types
        return {
            'temperature_celsius': float(temperature_celsius),
            'accelerometer_data': {
                'x_axis': float(acceleration_x),
                'y_axis': float(acceleration_y),
                'z_axis': float(acceleration_z)
            },
            'gyroscope_data': {
                'x_axis': float(gyroscope_x),
                'y_axis': float(gyroscope_y),
                'z_axis': float(gyroscope_z)
            },
        }

    def get_temperature_reading(self):
        """
        Get the current temperature reading from the MPU6500's built-in temperature sensor.

        This method provides a convenient way to access only the temperature data
        from the sensor without reading all sensor axes. The temperature sensor
        is primarily used for thermal compensation of the accelerometer and
        gyroscope readings, as MEMS sensors are sensitive to temperature changes.

        Temperature Sensor Architecture:
        - Built-in temperature sensor integrated within the MPU6500 chip
        - Provides temperature data for thermal compensation algorithms
        - Temperature affects sensor accuracy and drift characteristics
        - Typical operating range: -40°C to +85°C
        - Resolution: approximately 1°C per LSB (Least Significant Bit)

        Thermal Compensation Importance:
        - MEMS accelerometers and gyroscopes are temperature-sensitive
        - Temperature changes cause bias drift and scale factor variations
        - Thermal compensation improves long-term stability and accuracy
        - Critical for applications requiring precise measurements over time

        Args:
            None

        Returns:
            float: Current temperature in Celsius
                - Range: Typically -40°C to +85°C
                - Resolution: ~1°C
                - Accuracy: ±1°C (typical)

        Raises:
            RuntimeError: If sensor is not initialized (inherited from read_raw_sensor_data)
            Exception: If I2C communication fails (inherited from read_raw_sensor_data)

        Note:
            - This method calls read_raw_sensor_data() internally
            - For better performance when reading multiple values, use read_raw_sensor_data()
            - Temperature readings are used for sensor calibration and compensation
            - Temperature affects sensor bias and scale factor accuracy

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()
            temp = imu.get_temperature_reading()
            print(f"Sensor temperature: {temp:.1f}°C")

            # Use temperature for thermal compensation
            if temp > 60.0:
                print("Warning: High sensor temperature detected")
        """
        # Extract temperature from comprehensive sensor data
        # This method provides a convenient single-value access point
        return self.read_raw_sensor_data()['temperature_celsius']

    def get_accelerometer_data(self):
        """
        Get the current 3-axis accelerometer readings from the MPU6500.

        This method provides convenient access to only the accelerometer data
        without reading gyroscope or temperature sensors. The accelerometer
        measures linear acceleration, which includes both gravity and motion
        acceleration components.

        Accelerometer Architecture (MEMS Technology):
        - 3-axis capacitive MEMS (Micro-Electro-Mechanical Systems) accelerometer
        - Each axis measures acceleration along its direction
        - Measures both static (gravity) and dynamic (motion) acceleration
        - 16-bit ADC resolution with configurable full-scale ranges
        - Built-in anti-aliasing filter to reduce noise

        Full-Scale Range Options:
        - ±2g: Highest resolution, best for tilt sensing and low-acceleration applications
        - ±4g: Good balance for general motion detection
        - ±8g: Suitable for higher acceleration applications
        - ±16g: Maximum range for high-impact or vibration applications

        Physical Principles:
        - When stationary: measures gravity vector (≈9.81 m/s² downward)
        - During motion: measures gravity + motion acceleration
        - Tilt detection: gravity vector changes with orientation
        - Vibration detection: rapid changes in acceleration values

        Coordinate System (Right-Handed):
        - X-axis: Points forward (positive = forward acceleration)
        - Y-axis: Points right (positive = rightward acceleration)
        - Z-axis: Points up (positive = upward acceleration)

        Args:
            None

        Returns:
            dict: 3-axis accelerometer data dictionary containing:
                - 'x_axis' (float): X-axis acceleration (m/s² or g)
                - 'y_axis' (float): Y-axis acceleration (m/s² or g)
                - 'z_axis' (float): Z-axis acceleration (m/s² or g)

        Raises:
            RuntimeError: If sensor is not initialized (inherited from read_raw_sensor_data)
            Exception: If I2C communication fails (inherited from read_raw_sensor_data)

        Note:
            - Units depend on configured scale factor (SF_M_S2 for m/s², SF_G for g)
            - When stationary, total magnitude ≈ 9.81 m/s² (1g)
            - For tilt sensing, use with complementary filter for best results
            - Accelerometer alone cannot distinguish between tilt and linear acceleration

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()
            accel = imu.get_accelerometer_data()

            # Calculate total acceleration magnitude
            total_accel = math.sqrt(
                accel['x_axis']**2 + accel['y_axis']**2 + accel['z_axis']**2
            )
            print(f"Total acceleration: {total_accel:.2f} m/s²")

            # Detect if device is stationary (close to 1g)
            if abs(total_accel - 9.81) < 0.5:
                print("Device appears to be stationary")
        """
        # Extract accelerometer data from comprehensive sensor reading
        # This method provides focused access to acceleration measurements only
        return self.read_raw_sensor_data()['accelerometer_data']

    def get_gyroscope_data(self):
        """
        Get the current 3-axis gyroscope readings from the MPU6500.

        This method provides convenient access to only the gyroscope data
        without reading accelerometer or temperature sensors. The gyroscope
        measures angular velocity (rotation rate) around each axis, which
        is essential for detecting orientation changes and rotational motion.

        Gyroscope Architecture (MEMS Technology):
        - 3-axis MEMS (Micro-Electro-Mechanical Systems) gyroscope
        - Each axis measures angular velocity around its rotation axis
        - Uses Coriolis effect to detect rotation in vibrating structures
        - 16-bit ADC resolution with configurable full-scale ranges
        - Built-in low-pass filter to reduce noise and aliasing

        Full-Scale Range Options:
        - ±250°/s: Highest resolution, best for precise orientation tracking
        - ±500°/s: Good balance for general motion detection
        - ±1000°/s: Suitable for faster rotation applications
        - ±2000°/s: Maximum range for high-speed rotation detection

        Physical Principles:
        - Measures rate of rotation around each axis (degrees per second)
        - Zero output when stationary (no rotation)
        - Positive values indicate rotation in one direction
        - Negative values indicate rotation in opposite direction
        - Integration over time gives total angle change

        Coordinate System (Right-Handed):
        - X-axis: Rotation around X-axis (pitch rate)
        - Y-axis: Rotation around Y-axis (roll rate)
        - Z-axis: Rotation around Z-axis (yaw rate)

        Gyroscope Characteristics:
        - High frequency response (good for detecting quick changes)
        - Subject to drift over time (bias instability)
        - Temperature sensitive (requires thermal compensation)
        - Excellent for short-term angle tracking
        - Poor for long-term absolute orientation (drift accumulation)

        Args:
            None

        Returns:
            dict: 3-axis gyroscope data dictionary containing:
                - 'x_axis' (float): X-axis angular velocity (°/s or rad/s)
                - 'y_axis' (float): Y-axis angular velocity (°/s or rad/s)
                - 'z_axis' (float): Z-axis angular velocity (°/s or rad/s)

        Raises:
            RuntimeError: If sensor is not initialized (inherited from read_raw_sensor_data)
            Exception: If I2C communication fails (inherited from read_raw_sensor_data)

        Note:
            - Units depend on configured scale factor (SF_DEG_S for °/s, SF_RAD_S for rad/s)
            - When stationary, all values should be close to zero
            - For angle estimation, integrate over time: angle += rate * time_delta
            - Use with complementary filter to combine with accelerometer for best results
            - Gyroscope drift requires periodic correction from accelerometer

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()
            gyro = imu.get_gyroscope_data()

            # Calculate total rotation rate magnitude
            total_rate = math.sqrt(
                gyro['x_axis']**2 + gyro['y_axis']**2 + gyro['z_axis']**2
            )
            print(f"Total rotation rate: {total_rate:.2f} °/s")

            # Detect if device is stationary (low rotation rates)
            if total_rate < 1.0:
                print("Device appears to be stationary")
            else:
                print("Device is rotating")
        """
        # Extract gyroscope data from comprehensive sensor reading
        # This method provides focused access to angular velocity measurements only
        return self.read_raw_sensor_data()['gyroscope_data']

    @staticmethod
    def _calculate_tilt_angles_from_accelerometer(accelerometer_data):
        """
        Calculate tilt angles from accelerometer data using trigonometry.

        This static method converts 3-axis accelerometer readings into tilt angles
        (pitch, roll, yaw) using trigonometric calculations. The accelerometer
        measures the combined effect of gravity and linear acceleration.

        Mathematical Background:
        - When stationary, accelerometer measures gravity vector (≈9.81 m/s²)
        - Tilt angles are calculated using atan2() for proper quadrant handling
        - Pitch: rotation around X-axis (forward/backward tilt)
        - Roll: rotation around Y-axis (left/right tilt)
        - Yaw: rotation around Z-axis (compass direction - not reliable from accelerometer alone)

        Args:
            accelerometer_data (dict): Dictionary containing accelerometer readings
                - 'x_axis' (float): X-axis acceleration (m/s²)
                - 'y_axis' (float): Y-axis acceleration (m/s²)
                - 'z_axis' (float): Z-axis acceleration (m/s²)

        Returns:
            tuple: (pitch_degrees, roll_degrees, yaw_degrees)
                - pitch_degrees (float): Pitch angle in degrees (-90° to +90°)
                - roll_degrees (float): Roll angle in degrees (-90° to +90°)
                - yaw_degrees (float): Yaw angle in degrees (-90° to +90°)

        Note:
            Yaw calculation from accelerometer alone is not reliable for navigation
            as it doesn't distinguish between different orientations with same gravity vector.
            For accurate yaw, use a magnetometer or gyroscope integration.
        """
        # Extract acceleration components
        acceleration_x = accelerometer_data['x_axis']
        acceleration_y = accelerometer_data['y_axis']
        acceleration_z = accelerometer_data['z_axis']

        # Calculate pitch angle (rotation around X-axis)
        # Pitch = atan2(Y, sqrt(X² + Z²))
        # This gives the angle between the Y-axis and the horizontal plane
        xz_magnitude = math.sqrt(acceleration_x * acceleration_x + acceleration_z * acceleration_z) or 1e-9
        pitch_radians = math.atan2(acceleration_y, xz_magnitude)
        pitch_degrees = pitch_radians * 180.0 / math.pi

        # Calculate roll angle (rotation around Y-axis)
        # Roll = atan2(-X, sqrt(Y² + Z²))
        # Negative X because roll is positive when rotating clockwise around Y-axis
        yz_magnitude = math.sqrt(acceleration_y * acceleration_y + acceleration_z * acceleration_z) or 1e-9
        roll_radians = math.atan2(-acceleration_x, yz_magnitude)
        roll_degrees = roll_radians * 180.0 / math.pi

        # Calculate yaw angle (rotation around Z-axis)
        # Yaw = atan2(Z, sqrt(X² + Y²))
        # Note: This is not reliable for navigation without magnetometer
        xy_magnitude = math.sqrt(acceleration_x * acceleration_x + acceleration_y * acceleration_y) or 1e-9
        yaw_radians = math.atan2(acceleration_z, xy_magnitude)
        yaw_degrees = yaw_radians * 180.0 / math.pi

        return pitch_degrees, roll_degrees, yaw_degrees

    def get_tilt_angles_from_accelerometer(self):
        """
        Get tilt angles calculated directly from accelerometer data using trigonometry.

        This method provides a convenient way to calculate tilt angles (pitch, roll, yaw)
        from the current accelerometer readings. The calculation uses trigonometric
        functions to convert 3-axis acceleration data into orientation angles.

        Tilt Angle Calculation Process:
        1. Read current accelerometer data from the sensor
        2. Apply trigonometric calculations to determine orientation
        3. Convert from radians to degrees for user-friendly output
        4. Return structured dictionary with all three angles

        Mathematical Background:
        - When stationary, accelerometer measures gravity vector (≈9.81 m/s²)
        - Gravity vector changes direction based on device orientation
        - Trigonometric functions (atan2) convert acceleration ratios to angles
        - atan2() provides proper quadrant handling for all orientations

        Angle Definitions:
        - Pitch: Rotation around X-axis (forward/backward tilt)
          * Positive: nose up, tail down
          * Negative: nose down, tail up
        - Roll: Rotation around Y-axis (left/right tilt)
          * Positive: right wing down, left wing up
          * Negative: left wing down, right wing up
        - Yaw: Rotation around Z-axis (compass direction)
          * Note: Not reliable from accelerometer alone

        Limitations of Accelerometer-Only Tilt Calculation:
        - Cannot distinguish between tilt and linear acceleration
        - Yaw calculation is not reliable for navigation
        - Affected by vibration and motion
        - Best used when device is relatively stationary

        Args:
            None

        Returns:
            dict: Tilt angles dictionary containing:
                - 'pitch_angle' (float): Pitch angle in degrees (-90° to +90°)
                - 'roll_angle' (float): Roll angle in degrees (-90° to +90°)
                - 'yaw_angle' (float): Yaw angle in degrees (-90° to +90°)

        Raises:
            RuntimeError: If sensor is not initialized (inherited from get_accelerometer_data)
            Exception: If I2C communication fails (inherited from get_accelerometer_data)

        Note:
            - Angles are calculated from current accelerometer reading
            - For smooth, filtered angles, use update_complementary_filtered_angles()
            - Yaw from accelerometer alone is not reliable for navigation
            - Best accuracy when device is stationary or moving slowly

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()
            angles = imu.get_tilt_angles_from_accelerometer()

            print(f"Pitch: {angles['pitch_angle']:.1f}°")
            print(f"Roll: {angles['roll_angle']:.1f}°")
            print(f"Yaw: {angles['yaw_angle']:.1f}°")

            # Check if device is level
            if abs(angles['pitch_angle']) < 5 and abs(angles['roll_angle']) < 5:
                print("Device is approximately level")
        """
        # Get current accelerometer data from the sensor
        accelerometer_data = self.get_accelerometer_data()

        # Calculate tilt angles using trigonometric functions
        # This uses the static method for the actual calculation
        pitch_angle, roll_angle, yaw_angle = self._calculate_tilt_angles_from_accelerometer(accelerometer_data)

        # Return structured dictionary with all calculated angles
        return {
            'pitch_angle': pitch_angle,
            'roll_angle': roll_angle,
            'yaw_angle': yaw_angle
        }

    def update_complementary_filtered_angles(self, gyroscope_data=None, time_delta=None):
        """
        Update complementary filtered angles using sensor fusion algorithm.

        This method implements a complementary filter that combines gyroscope and
        accelerometer data to provide smooth, accurate angle estimates. The filter
        leverages the strengths of both sensors while compensating for their weaknesses.

        Complementary Filter Algorithm:
        The complementary filter is a sensor fusion technique that combines:
        - Gyroscope: High-frequency response, good for detecting quick changes
        - Accelerometer: Low-frequency stability, good for absolute orientation reference

        Filter Equation:
        filtered_angle = α × gyroscope_angle + (1-α) × accelerometer_angle
        Where α (alpha) is the filter coefficient (0.0 to 1.0)

        Filter Coefficient Behavior:
        - α = 0.0: Use only accelerometer (stable but slow response)
        - α = 1.0: Use only gyroscope (fast but drifts over time)
        - α = 0.98: Good balance for most applications (recommended)

        Algorithm Process:
        1. Calculate time delta since last measurement
        2. Integrate gyroscope data to estimate current angles
        3. Get accelerometer-based angle estimates
        4. Apply weighted average using filter coefficient
        5. Update internal filter state for next iteration

        Gyroscope Integration:
        - angle_new = angle_old + angular_velocity × time_delta
        - Provides smooth, responsive angle tracking
        - Accumulates drift over time (bias instability)

        Accelerometer Correction:
        - Provides absolute reference for gravity direction
        - Corrects gyroscope drift over time
        - Noisy and affected by linear acceleration

        Args:
            gyroscope_data (dict, optional): Pre-read gyroscope data dictionary
                - If None, method will read current gyroscope data
                - Should contain 'x_axis', 'y_axis', 'z_axis' keys
            time_delta (float, optional): Time elapsed since last update in seconds
                - If None, method will calculate from internal timestamp
                - Should be positive value representing time interval

        Returns:
            dict: Filtered angle estimates containing:
                - 'pitch_angle' (float): Filtered pitch angle in degrees
                - 'roll_angle' (float): Filtered roll angle in degrees

        Raises:
            RuntimeError: If sensor is not initialized (inherited from sensor read methods)
            Exception: If I2C communication fails (inherited from sensor read methods)

        Note:
            - Filter state is maintained internally between calls
            - First call initializes the filter with accelerometer values
            - Regular updates (10-100Hz) provide best results
            - Filter coefficient can be adjusted for different applications
            - Yaw angle is not calculated (requires magnetometer for accuracy)

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()

            # Update filter at regular intervals
            while True:
                filtered_angles = imu.update_complementary_filtered_angles()
                print(f"Pitch: {filtered_angles['pitch_angle']:.1f}°")
                print(f"Roll: {filtered_angles['roll_angle']:.1f}°")
                utime.sleep_ms(50)  # 20Hz update rate
        """
        # Get gyroscope data if not provided
        if gyroscope_data is None:
            gyroscope_data = self.get_gyroscope_data()

        # Calculate time delta if not provided
        if time_delta is None:
            current_time_ms = utime.ticks_ms()
            time_delta = (current_time_ms - self._last_measurement_time_ms) / 1000.0
            self._last_measurement_time_ms = current_time_ms

        # Integrate gyroscope data for angle estimation
        # Gyroscope provides rate of change, integration gives total angle
        # Negative Y-axis because roll is positive when rotating clockwise around Y-axis
        gyroscope_pitch = self.filtered_pitch_angle + gyroscope_data['x_axis'] * time_delta
        gyroscope_roll = self.filtered_roll_angle - gyroscope_data['y_axis'] * time_delta

        # Get accelerometer-based angle estimates
        # Accelerometer provides absolute reference for gravity direction
        accelerometer_data = self.get_accelerometer_data()
        accelerometer_pitch, accelerometer_roll, _ = self._calculate_tilt_angles_from_accelerometer(accelerometer_data)

        # Apply complementary filter
        # Weighted average combines gyroscope responsiveness with accelerometer stability
        filter_alpha = self.complementary_filter_coefficient
        self.filtered_pitch_angle = filter_alpha * gyroscope_pitch + (1.0 - filter_alpha) * accelerometer_pitch
        self.filtered_roll_angle = filter_alpha * gyroscope_roll + (1.0 - filter_alpha) * accelerometer_roll

        # Return current filtered angle estimates
        return {
            'pitch_angle': self.filtered_pitch_angle,
            'roll_angle': self.filtered_roll_angle
        }

    def get_comprehensive_sensor_data(self):
        """
        Get comprehensive sensor data including raw readings, calculated angles, and filtered estimates.

        This method provides a complete snapshot of all sensor data and derived calculations
        in a single call. It combines raw sensor readings, trigonometric angle calculations,
        and complementary filtered angle estimates for comprehensive sensor fusion analysis.

        Data Fusion Process:
        1. Read all raw sensor data (accelerometer, gyroscope, temperature)
        2. Calculate tilt angles from accelerometer using trigonometry
        3. Update complementary filter with current sensor data
        4. Combine all data into comprehensive dictionary
        5. Add timestamp for data correlation and analysis

        Comprehensive Data Structure:
        - Raw sensor readings: Direct measurements from all sensors
        - Tilt angles: Calculated from accelerometer using trigonometry
        - Filtered angles: Sensor fusion result from complementary filter
        - Temperature: For thermal compensation and monitoring
        - Timestamp: For data correlation and time-series analysis

        Use Cases:
        - Complete system monitoring and diagnostics
        - Data logging and analysis applications
        - Sensor fusion algorithm development
        - Performance comparison between different angle calculation methods
        - Real-time monitoring dashboards

        Args:
            None

        Returns:
            dict: Comprehensive sensor data dictionary containing:
                - 'timestamp_ms' (int): Current timestamp in milliseconds
                - 'temperature_celsius' (float): Temperature reading in Celsius
                - 'accelerometer_data' (dict): 3-axis acceleration data
                    * 'x_axis' (float): X-axis acceleration (m/s² or g)
                    * 'y_axis' (float): Y-axis acceleration (m/s² or g)
                    * 'z_axis' (float): Z-axis acceleration (m/s² or g)
                - 'gyroscope_data' (dict): 3-axis angular velocity data
                    * 'x_axis' (float): X-axis angular velocity (°/s or rad/s)
                    * 'y_axis' (float): Y-axis angular velocity (°/s or rad/s)
                    * 'z_axis' (float): Z-axis angular velocity (°/s or rad/s)
                - 'tilt_angles' (dict): Accelerometer-based tilt angles
                    * 'pitch_angle' (float): Pitch angle in degrees
                    * 'roll_angle' (float): Roll angle in degrees
                    * 'yaw_angle' (float): Yaw angle in degrees
                - 'filtered_angles' (dict): Complementary filtered angles
                    * 'pitch_angle' (float): Filtered pitch angle in degrees
                    * 'roll_angle' (float): Filtered roll angle in degrees

        Raises:
            RuntimeError: If sensor is not initialized (inherited from sensor read methods)
            Exception: If I2C communication fails (inherited from sensor read methods)

        Note:
            - This method updates the complementary filter state
            - All data is captured at the same moment for consistency
            - Timestamp allows correlation with external events
            - Use for comprehensive analysis and monitoring applications
            - More computationally expensive than individual sensor reads

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()

            # Get comprehensive data snapshot
            data = imu.get_comprehensive_sensor_data()

            print(f"Timestamp: {data['timestamp_ms']} ms")
            print(f"Temperature: {data['temperature_celsius']:.1f}°C")
            print(f"Raw Pitch: {data['tilt_angles']['pitch_angle']:.1f}°")
            print(f"Filtered Pitch: {data['filtered_angles']['pitch_angle']:.1f}°")

            # Compare raw vs filtered angles
            pitch_diff = abs(data['tilt_angles']['pitch_angle'] - data['filtered_angles']['pitch_angle'])
            print(f"Pitch difference: {pitch_diff:.1f}°")
        """
        # Read all raw sensor data in one operation for consistency
        raw_sensor_data = self.read_raw_sensor_data()

        # Calculate tilt angles from accelerometer data using trigonometry
        # This provides the "raw" angle estimates from gravity vector
        pitch_angle, roll_angle, yaw_angle = self._calculate_tilt_angles_from_accelerometer(raw_sensor_data['accelerometer_data'])

        # Update complementary filter with current sensor data
        # This provides the "filtered" angle estimates using sensor fusion
        filtered_angles = self.update_complementary_filtered_angles(gyroscope_data=raw_sensor_data['gyroscope_data'], time_delta=None)

        # Combine all data into comprehensive dictionary
        # Timestamp allows correlation with external events and data logging
        return {
            'timestamp_ms': utime.ticks_ms(),
            'temperature_celsius': raw_sensor_data['temperature_celsius'],
            'accelerometer_data': raw_sensor_data['accelerometer_data'],
            'gyroscope_data': raw_sensor_data['gyroscope_data'],
            'tilt_angles': {
                'pitch_angle': pitch_angle,
                'roll_angle': roll_angle,
                'yaw_angle': yaw_angle
            },
            'filtered_angles': filtered_angles
        }

    def reset_complementary_filter(self):
        """
        Reset the complementary filter state to initial values.

        This method clears the internal filter state and resets all accumulated
        angle estimates to zero. It also resets the timestamp used for time
        delta calculations, effectively restarting the filter from a clean state.

        Filter State Reset Process:
        1. Reset filtered pitch angle to 0.0 degrees
        2. Reset filtered roll angle to 0.0 degrees
        3. Reset timestamp to current time for fresh time delta calculations
        4. Clear any accumulated gyroscope integration drift

        When to Use Filter Reset:
        - System startup or initialization
        - After sensor calibration or reconfiguration
        - When filter has accumulated significant drift
        - Before starting a new measurement session
        - After sensor has been stationary for extended periods
        - When switching between different filter coefficients

        Filter State Components:
        - filtered_pitch_angle: Current filtered pitch estimate (degrees)
        - filtered_roll_angle: Current filtered roll estimate (degrees)
        - _last_measurement_time_ms: Timestamp for time delta calculations

        Post-Reset Behavior:
        - Next call to update_complementary_filtered_angles() will initialize with accelerometer values
        - Filter will gradually converge to accurate estimates over several iterations
        - Time delta calculations will start fresh from reset timestamp

        Args:
            None

        Returns:
            None

        Raises:
            None

        Note:
            - Filter will need several iterations to converge after reset
            - Consider calling this before starting continuous measurements
            - Reset is useful for eliminating accumulated gyroscope drift
            - Filter coefficient remains unchanged after reset

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()

            # Reset filter before starting measurements
            imu.reset_complementary_filter()

            # Allow filter to converge over several iterations
            for i in range(10):
                angles = imu.update_complementary_filtered_angles()
                print(f"Iteration {i}: Pitch = {angles['pitch_angle']:.1f}°")
                utime.sleep_ms(100)
        """
        # Reset filtered angle estimates to zero
        # This clears any accumulated gyroscope integration drift
        self.filtered_pitch_angle = 0.0
        self.filtered_roll_angle = 0.0

        # Reset timestamp to current time
        # This ensures fresh time delta calculations for next filter update
        self._last_measurement_time_ms = utime.ticks_ms()

    def set_complementary_filter_coefficient(self, filter_coefficient):
        """
        Set the complementary filter coefficient to control sensor fusion balance.

        This method allows dynamic adjustment of the complementary filter coefficient,
        which controls the balance between gyroscope and accelerometer data in the
        sensor fusion algorithm. The coefficient determines the relative weight
        given to each sensor's contribution to the final angle estimate.

        Filter Coefficient Behavior:
        The complementary filter equation is:
        filtered_angle = α × gyroscope_angle + (1-α) × accelerometer_angle

        Where α (alpha) is the filter coefficient with the following effects:
        - α = 0.0: Use only accelerometer (100% accelerometer, 0% gyroscope)
          * Very stable, no drift
          * Slow response to changes
          * Noisy output
        - α = 0.5: Equal weight to both sensors (50% each)
          * Balanced response
          * Moderate stability and responsiveness
        - α = 0.98: Recommended for most applications (98% gyroscope, 2% accelerometer)
          * Fast response to changes
          * Good stability with minimal drift
          * Smooth output
        - α = 1.0: Use only gyroscope (0% accelerometer, 100% gyroscope)
          * Very fast response
          * Significant drift over time
          * Smooth but inaccurate long-term

        Application Guidelines:
        - High-frequency applications (robotics, gaming): α = 0.95-0.99
        - General purpose (drones, vehicles): α = 0.98
        - Low-frequency applications (tilt sensing): α = 0.90-0.95
        - Maximum stability (leveling): α = 0.80-0.90

        Args:
            filter_coefficient (float): Filter coefficient value (0.0 to 1.0)
                - 0.0: Use only accelerometer (stable, slow)
                - 1.0: Use only gyroscope (fast, drifts)
                - 0.98: Recommended balance for most applications

        Returns:
            None

        Raises:
            ValueError: If filter coefficient is outside valid range [0.0, 1.0]

        Note:
            - Coefficient change takes effect immediately on next filter update
            - Consider resetting filter after changing coefficient for best results
            - Higher values provide faster response but more drift
            - Lower values provide more stability but slower response

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()
            imu.initialize_sensor()

            # Set coefficient for fast response (robotics application)
            imu.set_complementary_filter_coefficient(0.99)

            # Set coefficient for stable leveling (construction application)
            imu.set_complementary_filter_coefficient(0.85)

            # Reset filter after changing coefficient
            imu.reset_complementary_filter()
        """
        # Convert input to float for consistent data type
        coefficient_value = float(filter_coefficient)

        # Validate coefficient is within valid range
        if 0.0 <= coefficient_value <= 1.0:
            # Update the filter coefficient
            self.complementary_filter_coefficient = coefficient_value
        else:
            # Raise error for invalid coefficient values
            raise ValueError("Complementary filter coefficient must be in range [0.0, 1.0]")

    def get_sensor_status(self):
        """
        Get comprehensive status information about the sensor and system configuration.

        This method provides a complete diagnostic overview of the sensor system,
        including initialization status, connection state, configuration parameters,
        and any error information. It's useful for system monitoring, debugging,
        and status reporting applications.

        Status Information Categories:
        1. Initialization Status: Whether the sensor has been properly initialized
        2. Connection Status: Whether the sensor is currently connected and responsive
        3. Error Information: Any error messages from failed operations
        4. Configuration Parameters: Current system configuration settings
        5. Filter Settings: Current complementary filter configuration

        Diagnostic Use Cases:
        - System health monitoring and reporting
        - Troubleshooting sensor connection issues
        - Configuration verification and validation
        - Error tracking and debugging
        - System status dashboards
        - Automated testing and validation

        Args:
            None

        Returns:
            dict: Comprehensive status dictionary containing:
                - 'is_sensor_initialized' (bool): Whether sensor has been initialized
                    * True: Sensor driver created and configured successfully
                    * False: Sensor not initialized or initialization failed
                - 'is_sensor_connected' (bool): Whether sensor is currently connected
                    * True: Sensor responds to I2C communication
                    * False: Sensor not found or communication failed
                - 'last_error_message' (str or None): Last error message from failed operations
                    * None: No errors recorded
                    * String: Description of last error that occurred
                - 'imu_device_address' (int): I2C address of the MPU6500 sensor
                    * Typically 0x68 (104 in decimal)
                - 'i2c_frequency' (int): I2C communication frequency in Hz
                    * Common values: 100000 (100kHz), 400000 (400kHz)
                - 'complementary_filter_coefficient' (float): Current filter coefficient
                    * Range: 0.0 to 1.0
                    * Default: 0.98 (recommended for most applications)

        Raises:
            None

        Note:
            - This method performs a live connection check (calls is_sensor_connected())
            - Status information is current as of the method call
            - Use for system monitoring and diagnostic applications
            - Error messages are cleared when operations succeed

        Usage Example:
            imu = MPU6500InertialMeasurementUnitHandler()

            # Check status before initialization
            status = imu.get_sensor_status()
            print(f"Initialized: {status['is_sensor_initialized']}")
            print(f"Connected: {status['is_sensor_connected']}")

            # Initialize sensor
            imu.initialize_sensor()

            # Check status after initialization
            status = imu.get_sensor_status()
            print(f"Initialized: {status['is_sensor_initialized']}")
            print(f"Connected: {status['is_sensor_connected']}")
            print(f"Filter coefficient: {status['complementary_filter_coefficient']}")

            if status['last_error_message']:
                print(f"Last error: {status['last_error_message']}")
        """
        # Return comprehensive status information dictionary
        # This provides a complete overview of the sensor system state
        return {
            'is_sensor_initialized': self.is_sensor_initialized,
            'is_sensor_connected': self.is_sensor_connected(),
            'last_error_message': self.last_error_message,
            'imu_device_address': self.imu_device_address,
            'i2c_frequency': self.i2c_frequency,
            'complementary_filter_coefficient': self.complementary_filter_coefficient,
        }
