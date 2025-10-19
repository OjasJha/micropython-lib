"""
MPU9250 9-Axis Inertial Measurement Unit Handler for Raspberry Pi Pico (MicroPython)

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

This module provides a high-level interface for the MPU9250 9-axis Inertial Measurement Unit (IMU),
which combines a 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer, and temperature sensor.

Key Concepts:
- 9-Axis IMU: Combines accelerometer, gyroscope, and magnetometer for complete motion sensing
- Sensor Fusion: Combining data from multiple sensors for accurate orientation estimation
- Heading Calculation: Using magnetometer data to determine compass direction
- MPU9250 Architecture: Contains MPU6500 (6-axis) + AK8963 (3-axis magnetometer)

MPU9250 Sensor Architecture:
- MPU6500: 3-axis accelerometer + 3-axis gyroscope + temperature sensor
- AK8963: 3-axis magnetometer for compass functionality
- I2C communication with two separate addresses (0x68 for MPU6500, 0x0C for AK8963)
- Combined package provides complete 9-axis motion sensing capability

Raspberry Pi Pico Specifics:
- Uses I2C pins (GPIO 20/21 by default) for sensor communication
- Requires 3.3V power supply and proper pull-up resistors (4.7kΩ recommended)
- I2C frequency up to 400kHz supported by the Pico
- Provides heading calculation using magnetometer data

Author: Ojas Jha
License: MIT License (see above)
Date: October 19, 2025
Version: 2.0
"""

# =============================================================================
# IMPORTS - Categorized by functionality
# =============================================================================

# MicroPython time utilities for timestamps
import utime  # MicroPython time utilities for delays and timestamps
import math  # Mathematical functions for heading calculations

# Import our custom sensor handlers
from mpu6500_handler import MPU6500InertialMeasurementUnitHandler  # 6-axis IMU handler
from ak8963_handler import AK8963MagnetometerHandler  # 3-axis magnetometer handler


class MPU9250NineAxisInertialMeasurementUnitHandler:
    """
    High-level orchestrator for the MPU9250 9-axis Inertial Measurement Unit.

    This class combines the MPU6500 6-axis IMU (accelerometer + gyroscope + temperature)
    and AK8963 3-axis magnetometer to provide a unified interface for complete
    9-axis motion sensing. It handles initialization of both sensors and provides
    comprehensive data including heading calculation.

    MPU9250 Architecture:
    The MPU9250 is essentially two sensors in one package:
    - MPU6500: 6-axis IMU (accelerometer + gyroscope + temperature)
    - AK8963: 3-axis magnetometer (compass)

    Sensor Fusion Benefits:
    - Accelerometer: Provides gravity reference and linear acceleration
    - Gyroscope: Provides angular velocity and short-term orientation changes
    - Magnetometer: Provides absolute heading reference (compass direction)
    - Temperature: Enables thermal compensation for improved accuracy

    Usage Example:
        imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler()
        if imu_9axis.initialize_sensor():
            data = imu_9axis.get_comprehensive_sensor_data()
            print(f"Heading: {data['heading_degrees']:.1f}°")
    """

    def __init__(
        self,
        serial_clock_pin=21,
        serial_data_pin=20,
        i2c_frequency=400_000,
        i2c_bus_id=0,
        mpu6500_imu_address=0x68,
        ak8963_magnetometer_address=0x0C,
        complementary_filter_coefficient=0.98
    ):
        """
        Initialize the MPU9250 9-axis IMU handler with dual-sensor configuration.

        This constructor sets up the MPU9250 9-axis IMU system by initializing both
        the MPU6500 6-axis IMU (accelerometer + gyroscope + temperature) and the
        AK8963 3-axis magnetometer. The MPU9250 is essentially two sensors in one
        package, requiring separate initialization and management.

        MPU9250 Dual-Sensor Architecture:
        The MPU9250 combines two distinct sensors in a single package:
        - MPU6500: 6-axis IMU with accelerometer, gyroscope, and temperature sensor
          * I2C Address: 0x68 (104 in decimal)
          * Provides motion sensing and orientation tracking
        - AK8963: 3-axis magnetometer for compass functionality
          * I2C Address: 0x0C (12 in decimal)
          * Provides magnetic field sensing and heading calculation

        I2C Bus Configuration:
        - Both sensors share the same I2C bus but have different addresses
        - Single I2C bus reduces wiring complexity and GPIO usage
        - Both sensors must be properly initialized for full 9-axis functionality
        - I2C frequency affects communication speed and reliability

        Raspberry Pi Pico Integration:
        - Uses standard I2C pins (GPIO 20/21) for communication
        - Requires 3.3V power supply and pull-up resistors (4.7kΩ recommended)
        - I2C frequency up to 400kHz supported by the Pico
        - Dual-sensor initialization provides comprehensive motion sensing

        Args:
            serial_clock_pin (int): GPIO pin for I2C clock line (SCL). Default: 21
                - Raspberry Pi Pico GPIO21 is commonly used for I2C SCL
                - Must be connected to MPU9250's SCL pin
            serial_data_pin (int): GPIO pin for I2C data line (SDA). Default: 20
                - Raspberry Pi Pico GPIO20 is commonly used for I2C SDA
                - Must be connected to MPU9250's SDA pin
            i2c_frequency (int): I2C communication frequency in Hz. Default: 400000
                - Standard I2C speeds: 100kHz (slow), 400kHz (fast)
                - Both sensors share the same I2C bus frequency
                - Higher frequency = faster communication but less reliable over long wires
            i2c_bus_id (int): I2C bus identifier. Default: 0
                - Raspberry Pi Pico has I2C0 and I2C1 buses
                - Both sensors will use the same bus ID
            mpu6500_imu_address (int): I2C address of the MPU6500 sensor. Default: 0x68
                - Fixed address for MPU6500 IMU within MPU9250 package
                - Cannot be changed (hardware-defined)
            ak8963_magnetometer_address (int): I2C address of the AK8963 sensor. Default: 0x0C
                - Fixed address for AK8963 magnetometer within MPU9250 package
                - Cannot be changed (hardware-defined)
            complementary_filter_coefficient (float): Filter balance coefficient (0.0-1.0). Default: 0.98
                - Controls the balance between gyroscope and accelerometer data
                - 0.98 provides good balance for most applications
                - Applied to the MPU6500 IMU for sensor fusion

        Returns:
            None

        Note:
            - Both sensors share the same I2C bus but have different addresses
            - The I2C bus is initialized but sensors are not yet configured
            - Call initialize_sensor() to establish communication with both sensors
            - Both sensors must initialize successfully for full 9-axis functionality
            - The handler creates separate instances for each sensor component

        Usage Example:
            # Initialize with default settings
            imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler()

            # Initialize with custom I2C frequency
            imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler(
                i2c_frequency=100_000  # Use slower I2C for better reliability
            )

            # Initialize with custom filter coefficient
            imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler(
                complementary_filter_coefficient=0.95  # More stable, less responsive
            )
        """
        # Store configuration parameters for reference and debugging
        self.serial_clock_pin = serial_clock_pin
        self.serial_data_pin = serial_data_pin
        self.i2c_frequency = i2c_frequency
        self.i2c_bus_id = i2c_bus_id
        self.mpu6500_imu_address = mpu6500_imu_address
        self.ak8963_magnetometer_address = ak8963_magnetometer_address
        self.complementary_filter_coefficient = complementary_filter_coefficient

        # Initialize the 6-axis IMU handler (MPU6500)
        # This handles accelerometer, gyroscope, and temperature sensor
        self.inertial_measurement_unit = MPU6500InertialMeasurementUnitHandler(
            serial_clock_pin=serial_clock_pin,
            serial_data_pin=serial_data_pin,
            i2c_frequency=i2c_frequency,
            i2c_bus_id=i2c_bus_id,
            imu_device_address=mpu6500_imu_address,
            complementary_filter_coefficient=complementary_filter_coefficient
        )

        # Initialize the 3-axis magnetometer handler (AK8963)
        # This handles compass functionality and heading calculation
        self.magnetometer_sensor = AK8963MagnetometerHandler(
            serial_clock_pin=serial_clock_pin,
            serial_data_pin=serial_data_pin,
            i2c_frequency=i2c_frequency,
            i2c_bus_id=i2c_bus_id,
            magnetometer_address=ak8963_magnetometer_address
        )

        # Initialize 9-axis sensor state variables
        self.is_sensor_initialized = False  # Tracks initialization status of both sensors
        self.last_error_message = None  # Stores last error for debugging

    def initialize_sensor(self):
        """
        Initialize the dual-sensor MPU9250 (MPU6500 + AK8963).

        Returns:
            bool: True if both sensors initialized successfully, False otherwise.
        """
        # 1) Initialize the MPU6500 6-axis IMU
        imu_initialization_success = self.inertial_measurement_unit.initialize_sensor()

        # If the IMU did not come up, fail early
        if not imu_initialization_success:
            self.is_sensor_initialized = False
            self.last_error_message = "IMU sensor failed to initialize; cannot continue with 9-axis setup"
            return False

        # 2) Enable I²C BYPASS so the external bus can talk directly to AK8963 inside the 9250
        # Register and bit definitions (same behavior as used by the working simple MPU9250 class)
        _INT_PIN_CFG      = 0x37     # MPU6500 INT Pin / Bypass Enable Configuration
        _I2C_BYPASS_MASK  = 0b00000010
        _I2C_BYPASS_EN    = 0b00000010

        try:
            # Access underlying low-level driver the 6500 handler created
            mpu = self.inertial_measurement_unit.imu_sensor
            if mpu is None:
                raise RuntimeError("Underlying MPU6500 driver instance is not available")

            # Read-modify-write INT_PIN_CFG to set I2C_BYPASS_EN
            current = mpu._register_char(_INT_PIN_CFG)
            current &= ~_I2C_BYPASS_MASK
            current |= _I2C_BYPASS_EN
            mpu._register_char(_INT_PIN_CFG, current)
        except Exception as e:
            # If we cannot enable bypass, the AK8963 will not respond on 0x0C
            self.is_sensor_initialized = False
            self.last_error_message = "Failed to enable AK8963 I2C bypass: {}".format(e)
            return False

        # 3) Now initialize the AK8963 3-axis magnetometer
        magnetometer_initialization_success = self.magnetometer_sensor.initialize_sensor()

        # 4) Determine overall status
        self.is_sensor_initialized = bool(imu_initialization_success and magnetometer_initialization_success)
        self.last_error_message = None if self.is_sensor_initialized else "One or more sub-sensors failed to initialize"

        return self.is_sensor_initialized

    def is_sensor_connected(self):
        """
        Check if both MPU6500 IMU and AK8963 magnetometer sensors are connected and responsive.

        This method verifies the physical connection and communication with both sensors
        in the MPU9250 9-axis IMU system. It performs I2C communication tests on both
        the MPU6500 6-axis IMU and the AK8963 3-axis magnetometer to ensure both
        sensors are properly connected and responsive.

        Dual-Sensor Connection Verification Process:
        1. Check MPU6500 IMU connection (address 0x68)
           - Attempt to read WHO_AM_I register
           - Verify sensor responds to I2C commands
           - Confirm accelerometer, gyroscope, and temperature sensor accessibility
        2. Check AK8963 magnetometer connection (address 0x0C)
           - Attempt to read magnetometer status registers
           - Verify sensor responds to I2C commands
           - Confirm magnetic field data accessibility
        3. Validate both sensors are operational
           - Both sensors must respond for full 9-axis functionality
           - Single sensor failure indicates partial system failure

        I2C Communication Verification:
        - Both sensors share the same I2C bus but have different addresses
        - MPU6500 address: 0x68 (104 in decimal)
        - AK8963 address: 0x0C (12 in decimal)
        - I2C timeout prevents hanging on unresponsive devices
        - Communication failures can indicate wiring or power issues

        Connection Failure Scenarios:
        - Loose or broken I2C connections
        - Power supply issues (3.3V required)
        - Missing pull-up resistors (4.7kΩ recommended)
        - I2C bus conflicts or address collisions
        - Sensor hardware failure

        Args:
            None

        Returns:
            bool: True if both sensors are connected and responsive, False otherwise
                - True: Both MPU6500 and AK8963 sensors respond to I2C commands
                - False: One or both sensors are not connected or not responding

        Raises:
            None (all exceptions are caught and handled internally)

        Note:
            - This method performs live I2C communication tests
            - Both sensors must be connected for full 9-axis functionality
            - Connection status can change during operation
            - Use for system health monitoring and diagnostics

        Usage Example:
            imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler()

            # Check connection status
            if imu_9axis.is_sensor_connected():
                print("Both sensors are connected and responsive")
                # Safe to initialize and use sensors
                imu_9axis.initialize_sensor()
            else:
                print("One or both sensors are not connected")
                print("Check wiring, power supply, and pull-up resistors")
        """
        # Check connection status of both sensors
        # Both sensors must be connected for full 9-axis functionality
        return self.inertial_measurement_unit.is_sensor_connected() and self.magnetometer_sensor.is_sensor_connected()

    @staticmethod
    def _calculate_heading_degrees(magnetic_field_x, magnetic_field_y):
        """
        Calculate compass heading in degrees from 2D magnetic field components.

        This static method converts 2D magnetic field readings from the AK8963
        magnetometer into a compass heading using trigonometric calculations.
        The heading represents the direction the sensor is pointing relative to
        magnetic north, providing compass functionality for navigation applications.

        Magnetometer Heading Calculation:
        The magnetometer measures the Earth's magnetic field in 3D space. For
        heading calculation, we use the horizontal components (X and Y axes)
        to determine the direction relative to magnetic north.

        Mathematical Background:
        - Uses atan2() for proper quadrant handling (returns -π to +π)
        - atan2(Y, X) gives angle from positive X-axis to the point (X, Y)
        - Converts radians to degrees for human-readable output
        - Normalizes result to 0-360° range for compass convention

        Coordinate System (Magnetometer):
        - X-axis: Typically points East (positive X = East)
        - Y-axis: Typically points North (positive Y = North)
        - Z-axis: Points up/down (not used in 2D heading calculation)

        Heading Convention:
        - 0° = Magnetic North (pointing toward magnetic north pole)
        - 90° = Magnetic East (pointing toward magnetic east)
        - 180° = Magnetic South (pointing toward magnetic south pole)
        - 270° = Magnetic West (pointing toward magnetic west)

        Limitations and Considerations:
        - Assumes sensor is level (no significant pitch/roll)
        - For tilted sensors, tilt compensation is required
        - Magnetic declination (difference between magnetic and true north) is not applied
        - Local magnetic interference can affect accuracy
        - Heading accuracy depends on magnetometer calibration

        Args:
            magnetic_field_x (float): X-axis magnetic field component in microteslas (μT)
                - Positive X typically points East
                - Measured by AK8963 magnetometer
            magnetic_field_y (float): Y-axis magnetic field component in microteslas (μT)
                - Positive Y typically points North
                - Measured by AK8963 magnetometer

        Returns:
            float: Heading angle in degrees (0° to 360°)
                - 0° = Magnetic North
                - 90° = Magnetic East
                - 180° = Magnetic South
                - 270° = Magnetic West
                - Values are normalized to 0-360° range

        Note:
            - This is a static method that can be called without class instance
            - Requires level sensor orientation for accurate results
            - For tilted sensors, use tilt compensation algorithms
            - Magnetic declination correction should be applied for true north
            - Local magnetic interference can affect accuracy

        Usage Example:
            # Calculate heading from magnetometer data
            magnetic_x = 15.2  # μT (East component)
            magnetic_y = 8.7   # μT (North component)

            heading = MPU9250NineAxisInertialMeasurementUnitHandler._calculate_heading_degrees(
                magnetic_x, magnetic_y
            )
            print(f"Heading: {heading:.1f}°")

            # Interpret heading
            if 0 <= heading < 90:
                print("Pointing Northeast")
            elif 90 <= heading < 180:
                print("Pointing Southeast")
            elif 180 <= heading < 270:
                print("Pointing Southwest")
            else:
                print("Pointing Northwest")
        """
        # Calculate heading using atan2 for proper quadrant handling
        # atan2(Y, X) gives angle from positive X-axis to the point (X, Y)
        # This provides the angle from East (X-axis) to the magnetic field vector
        heading_radians = math.atan2(magnetic_field_y, magnetic_field_x)

        # Convert radians to degrees for human-readable output
        # math.degrees() converts radians to degrees
        heading_degrees = math.degrees(heading_radians)

        # Normalize to 0-360° range (atan2 returns -180° to +180°)
        # Add 360° to negative values to get positive compass heading
        if heading_degrees < 0:
            heading_degrees += 360.0

        return heading_degrees

    def get_comprehensive_sensor_data(self):
        """
        Get comprehensive 9-axis sensor data including all measurements and calculated values.

        This method provides a complete snapshot of all sensor data from the MPU9250
        9-axis IMU system, combining data from both the MPU6500 6-axis IMU and the
        AK8963 3-axis magnetometer. It includes raw sensor readings, calculated
        angles, filtered estimates, and compass heading for comprehensive motion analysis.

        9-Axis Sensor Fusion Data:
        The MPU9250 provides complete 9-axis motion sensing by combining:
        - 3-axis accelerometer: Linear acceleration and gravity reference
        - 3-axis gyroscope: Angular velocity and rotation sensing
        - 3-axis magnetometer: Magnetic field and compass functionality
        - Temperature sensor: Thermal compensation and monitoring

        Data Processing Pipeline:
        1. Read comprehensive data from MPU6500 6-axis IMU
           - Raw accelerometer, gyroscope, and temperature data
           - Calculated tilt angles from accelerometer
           - Complementary filtered angle estimates
        2. Read magnetic field data from AK8963 magnetometer
           - 3-axis magnetic field measurements
           - Compass heading calculation
        3. Combine all data into unified structure
        4. Add timestamp for data correlation

        Comprehensive Data Structure:
        - Raw sensor readings: Direct measurements from all 9 axes
        - Calculated angles: Trigonometric calculations from accelerometer
        - Filtered angles: Sensor fusion results from complementary filter
        - Compass heading: Magnetic north reference from magnetometer
        - Temperature: Thermal compensation data
        - Timestamp: Data correlation and time-series analysis

        Use Cases:
        - Complete motion analysis and navigation
        - Drone and robotics orientation control
        - Gaming and virtual reality applications
        - Data logging and analysis systems
        - Real-time monitoring dashboards
        - Sensor fusion algorithm development

        Args:
            None

        Returns:
            dict: Comprehensive 9-axis sensor data dictionary containing:
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
                - 'magnetometer_data' (dict): 3-axis magnetic field data
                    * 'x_axis' (float): X-axis magnetic field (μT)
                    * 'y_axis' (float): Y-axis magnetic field (μT)
                    * 'z_axis' (float): Z-axis magnetic field (μT)
                - 'tilt_angles' (dict): Accelerometer-based tilt angles
                    * 'pitch_angle' (float): Pitch angle in degrees
                    * 'roll_angle' (float): Roll angle in degrees
                    * 'yaw_angle' (float): Yaw angle in degrees
                - 'filtered_angles' (dict): Complementary filtered angles
                    * 'pitch_angle' (float): Filtered pitch angle in degrees
                    * 'roll_angle' (float): Filtered roll angle in degrees
                - 'heading_degrees' (float): Compass heading in degrees (0° to 360°)

        Raises:
            RuntimeError: If 9-axis sensor system is not initialized

        Note:
            - This method updates the complementary filter state
            - All data is captured at the same moment for consistency
            - Heading calculation assumes level sensor orientation
            - Use for comprehensive motion analysis and navigation
            - More computationally expensive than individual sensor reads

        Usage Example:
            imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler()
            imu_9axis.initialize_sensor()

            # Get comprehensive 9-axis data
            data = imu_9axis.get_comprehensive_sensor_data()

            print(f"Timestamp: {data['timestamp_ms']} ms")
            print(f"Temperature: {data['temperature_celsius']:.1f}°C")
            print(f"Pitch: {data['filtered_angles']['pitch_angle']:.1f}°")
            print(f"Roll: {data['filtered_angles']['roll_angle']:.1f}°")
            print(f"Heading: {data['heading_degrees']:.1f}°")

            # Check for level orientation
            if abs(data['filtered_angles']['pitch_angle']) < 5 and abs(data['filtered_angles']['roll_angle']) < 5:
                print("Device is level - heading is accurate")
            else:
                print("Device is tilted - heading may be inaccurate")
        """
        # Verify 9-axis sensor system is properly initialized
        if not self.is_sensor_initialized:
            raise RuntimeError("9-axis sensor not initialized. Call initialize_sensor() first.")

        # Get comprehensive data from MPU6500 6-axis IMU
        # This includes accelerometer, gyroscope, temperature, and calculated angles
        imu_sensor_data = self.inertial_measurement_unit.get_comprehensive_sensor_data()

        # Get magnetic field data from AK8963 magnetometer
        # This provides compass functionality and heading calculation
        magnetic_field_data = self.magnetometer_sensor.read_magnetic_field_data()

        # Calculate compass heading from magnetic field components
        # Uses X and Y components for 2D heading calculation
        heading_degrees = self._calculate_heading_degrees(magnetic_field_data['x_axis'], magnetic_field_data['y_axis'])

        # Combine all 9-axis data into comprehensive dictionary
        # This provides complete motion sensing information
        return {
            'timestamp_ms': utime.ticks_ms(),
            'temperature_celsius': imu_sensor_data['temperature_celsius'],
            'accelerometer_data': imu_sensor_data['accelerometer_data'],
            'gyroscope_data': imu_sensor_data['gyroscope_data'],
            'magnetometer_data': magnetic_field_data,
            'tilt_angles': imu_sensor_data['tilt_angles'],
            'filtered_angles': imu_sensor_data['filtered_angles'],
            'heading_degrees': heading_degrees
        }

    def get_sensor_status(self):
        """
        Get comprehensive status information for the MPU9250 9-axis sensor system.

        This method provides a complete diagnostic overview of the MPU9250 9-axis IMU
        system, including status information for both the MPU6500 6-axis IMU and the
        AK8963 3-axis magnetometer. It combines individual sensor status information
        with overall system status for comprehensive monitoring and debugging.

        Dual-Sensor Status Information:
        The MPU9250 system status includes information from both sensors:
        - MPU6500 6-axis IMU status (accelerometer + gyroscope + temperature)
        - AK8963 3-axis magnetometer status (compass functionality)
        - Overall system initialization and connection status
        - Error messages and diagnostic information

        Status Information Categories:
        1. System-Level Status: Overall 9-axis system initialization and connection
        2. MPU6500 IMU Status: 6-axis sensor initialization, connection, and configuration
        3. AK8963 Magnetometer Status: 3-axis magnetometer initialization and connection
        4. Error Information: Any error messages from failed operations
        5. Configuration Parameters: Current system configuration settings

        Diagnostic Use Cases:
        - System health monitoring and reporting
        - Troubleshooting dual-sensor connection issues
        - Configuration verification and validation
        - Error tracking and debugging for both sensors
        - System status dashboards and monitoring
        - Automated testing and validation

        Args:
            None

        Returns:
            dict: Comprehensive dual-sensor status dictionary containing:
                - 'is_sensor_initialized' (bool): Whether 9-axis system is initialized
                    * True: Both MPU6500 and AK8963 sensors initialized successfully
                    * False: One or both sensors failed to initialize
                - 'is_sensor_connected' (bool): Whether both sensors are connected
                    * True: Both sensors respond to I2C communication
                    * False: One or both sensors are not connected or not responding
                - 'last_error_message' (str or None): Last error message from failed operations
                    * None: No errors recorded
                    * String: Description of last error that occurred
                - 'imu_sensor_status' (dict): MPU6500 6-axis IMU status information
                    * Contains detailed status from MPU6500InertialMeasurementUnitHandler
                    * Includes initialization, connection, and configuration status
                - 'magnetometer_sensor_status' (dict): AK8963 magnetometer status information
                    * Contains detailed status from AK8963MagnetometerHandler
                    * Includes initialization, connection, and configuration status

        Raises:
            None

        Note:
            - This method performs live connection checks on both sensors
            - Status information is current as of the method call
            - Use for comprehensive system monitoring and diagnostics
            - Individual sensor status provides detailed diagnostic information
            - Error messages are cleared when operations succeed

        Usage Example:
            imu_9axis = MPU9250NineAxisInertialMeasurementUnitHandler()

            # Check status before initialization
            status = imu_9axis.get_sensor_status()
            print(f"System initialized: {status['is_sensor_initialized']}")
            print(f"System connected: {status['is_sensor_connected']}")

            # Initialize both sensors
            imu_9axis.initialize_sensor()

            # Check status after initialization
            status = imu_9axis.get_sensor_status()
            print(f"System initialized: {status['is_sensor_initialized']}")
            print(f"System connected: {status['is_sensor_connected']}")

            # Check individual sensor status
            imu_status = status['imu_sensor_status']
            mag_status = status['magnetometer_sensor_status']

            print(f"IMU initialized: {imu_status['is_sensor_initialized']}")
            print(f"Magnetometer initialized: {mag_status['is_sensor_initialized']}")

            if status['last_error_message']:
                print(f"Last error: {status['last_error_message']}")
        """
        # Return comprehensive dual-sensor status information
        # This provides a complete overview of the MPU9250 9-axis system state
        return {
            'is_sensor_initialized': self.is_sensor_initialized,
            'is_sensor_connected': self.is_sensor_connected(),
            'last_error_message': self.last_error_message,
            'imu_sensor_status': self.inertial_measurement_unit.get_sensor_status(),
            'magnetometer_sensor_status': self.magnetometer_sensor.get_sensor_status(),
        }
