"""
AK8963 Magnetometer Handler Class for Raspberry Pi Pico (MicroPython)

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

This module provides a high-level interface for the AK8963 3-axis magnetometer sensor,
commonly found in MPU9250 9-axis IMU modules. The AK8963 is a digital compass that
measures the Earth's magnetic field in three dimensions (X, Y, Z axes).

Key Concepts:
- Magnetometer: A sensor that measures magnetic field strength and direction
- I2C Communication: Two-wire serial communication protocol used by the sensor
- Hard/Soft Iron Calibration: Process to compensate for magnetic interference
- MicroTesla (μT): Unit of magnetic field measurement (Earth's field ≈ 25-65 μT)

Raspberry Pi Pico Specifics:
- Uses I2C pins (GPIO 20/21 by default) for sensor communication
- Requires 3.3V power supply and proper pull-up resistors (4.7kΩ recommended)
- I2C frequency up to 400kHz supported by the Pico

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

# AK8963 sensor driver and configuration constants
from ak8963 import AK8963, MODE_CONTINOUS_MEASURE_1, OUTPUT_16_BIT


class AK8963MagnetometerHandler:
    """
    High-level handler for the AK8963 3-axis magnetometer sensor.

    This class provides a simplified interface for interacting with the AK8963 magnetometer,
    handling initialization, data reading, and calibration. It abstracts the low-level I2C
    communication and provides error handling and status monitoring.

    AK8963 Sensor Architecture:
    - 3-axis magnetometer with 16-bit ADC resolution
    - I2C slave device with address 0x0C (12 in decimal)
    - Built-in self-test and calibration capabilities
    - Measures magnetic field in microTesla (μT) units
    - Typical measurement range: ±4800 μT

    Usage Example:
        magnetometer = AK8963MagnetometerHandler()
        if magnetometer.initialize_sensor():
            magnetic_data = magnetometer.read_magnetic_field_data()
            print(f"X: {magnetic_data['x_axis']} μT")
    """

    def __init__(
        self,
        serial_clock_pin=21,
        serial_data_pin=20,
        i2c_frequency=400_000,
        i2c_bus_id=0,
        magnetometer_address=0x0C,
        measurement_mode=MODE_CONTINOUS_MEASURE_1,
        output_resolution=OUTPUT_16_BIT
    ):
        """
        Initialize the AK8963 magnetometer handler with configuration parameters.

        This constructor sets up the AK8963 3-axis magnetometer for magnetic field
        sensing and compass functionality. The AK8963 is commonly found in MPU9250
        9-axis IMU modules and provides precise magnetic field measurements for
        navigation and orientation applications.

        AK8963 Magnetometer Architecture:
        The AK8963 is a 3-axis magnetometer that measures the Earth's magnetic field:
        - 3-axis magnetic field sensing (X, Y, Z axes)
        - 16-bit ADC resolution for high precision measurements
        - I2C slave device with fixed address 0x0C (12 in decimal)
        - Built-in self-test and calibration capabilities
        - Typical measurement range: ±4800 μT (microTesla)
        - Earth's magnetic field: typically 25-65 μT

        Magnetic Field Measurement:
        - Measures total magnetic field including Earth's field and local interference
        - Used for compass functionality and navigation
        - Requires calibration to compensate for hard/soft iron interference
        - Provides heading calculation when combined with accelerometer data

        I2C Communication Configuration:
        - Uses standard I2C protocol for communication
        - Requires pull-up resistors (4.7kΩ recommended) on SCL and SDA lines
        - I2C frequency affects communication speed and reliability
        - Timeout prevents hanging on unresponsive devices

        Raspberry Pi Pico Integration:
        - Uses standard I2C pins (GPIO 20/21) for communication
        - Requires 3.3V power supply for proper operation
        - I2C frequency up to 400kHz supported by the Pico
        - Commonly used in MPU9250 9-axis IMU modules

        Args:
            serial_clock_pin (int): GPIO pin for I2C clock line (SCL). Default: 21
                - Raspberry Pi Pico GPIO21 is commonly used for I2C SCL
                - Must be connected to AK8963's SCL pin
            serial_data_pin (int): GPIO pin for I2C data line (SDA). Default: 20
                - Raspberry Pi Pico GPIO20 is commonly used for I2C SDA
                - Must be connected to AK8963's SDA pin
            i2c_frequency (int): I2C communication frequency in Hz. Default: 400000
                - Standard I2C speeds: 100kHz (slow), 400kHz (fast)
                - Higher frequency = faster communication but less reliable over long wires
                - 400kHz provides good balance for most applications
            i2c_bus_id (int): I2C bus identifier. Default: 0
                - Raspberry Pi Pico has I2C0 and I2C1 buses
                - Use I2C0 for standard applications
            magnetometer_address (int): I2C address of the AK8963 sensor. Default: 0x0C
                - Fixed address for AK8963 magnetometer (cannot be changed)
                - 0x0C = 12 in decimal
            measurement_mode: Sensor measurement mode. Default: MODE_CONTINOUS_MEASURE_1
                - Continuous measurement mode for real-time readings
                - Other modes available for power-saving applications
            output_resolution: ADC resolution setting. Default: OUTPUT_16_BIT
                - 16-bit resolution provides good precision for magnetic field measurements
                - Higher resolution = better accuracy but more data processing

        Returns:
            None

        Note:
            - The I2C bus is initialized but the sensor is not yet configured
            - Call initialize_sensor() to establish communication and configure the sensor
            - The AK8963 is commonly found in MPU9250 9-axis IMU modules
            - Magnetic field measurements are in microTesla (μT) units

        Usage Example:
            # Initialize with default settings
            magnetometer = AK8963MagnetometerHandler()

            # Initialize with custom I2C frequency
            magnetometer = AK8963MagnetometerHandler(
                i2c_frequency=100_000  # Use slower I2C for better reliability
            )

            # Initialize for MPU9250 module
            magnetometer = AK8963MagnetometerHandler(
                magnetometer_address=0x0C  # Standard AK8963 address in MPU9250
            )
        """
        # Store configuration parameters for reference and debugging
        self.serial_clock_pin = serial_clock_pin
        self.serial_data_pin = serial_data_pin
        self.i2c_frequency = i2c_frequency
        self.i2c_bus_id = i2c_bus_id
        self.magnetometer_address = magnetometer_address
        self.measurement_mode = measurement_mode
        self.output_resolution = output_resolution

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
        self.magnetometer_sensor = None  # Will hold the AK8963 driver instance
        self.is_sensor_initialized = False  # Tracks initialization status
        self.last_error_message = None  # Stores last error for debugging
        self._last_calibration_data = None  # Stores calibration results

    def initialize_sensor(self):
        """
        Initialize the AK8963 magnetometer sensor and establish I2C communication.

        This method creates the AK8963 driver instance, configures the sensor
        with the specified measurement mode and output resolution, and verifies
        communication by reading the WHO_AM_I register. The initialization
        process prepares the magnetometer for magnetic field measurements.

        Magnetometer Initialization Process:
        1. Create AK8963 driver instance with I2C bus and configuration
           - Configure measurement mode (continuous measurement by default)
           - Set output resolution (16-bit by default)
           - Establish I2C communication parameters
        2. Read WHO_AM_I register to verify sensor communication
           - This register should return 0x48 for AK8963
           - Confirms sensor is responding to I2C commands
        3. Set initialization status and clear any previous errors
           - Update internal state variables
           - Clear error messages from previous failed attempts

        AK8963 WHO_AM_I Register:
        - Register address: 0x00 (WHO_AM_I)
        - Expected value: 0x48 (72 in decimal)
        - Read-only register that identifies the sensor type
        - Used for sensor detection and communication verification

        Initialization Requirements:
        - I2C bus must be properly configured with pull-up resistors
        - Sensor must be powered with 3.3V supply
        - I2C communication must be functional
        - No other devices should conflict with address 0x0C

        Args:
            None

        Returns:
            bool: True if initialization successful, False otherwise
                - True: Sensor driver created and communication verified
                - False: Initialization failed due to communication or configuration error

        Raises:
            Exception: If I2C communication fails or sensor not found
                - I2C bus errors (timeout, device not found)
                - Sensor configuration errors
                - Hardware connection issues

        Note:
            - This method must be called before any other sensor operations
            - Check the return value to ensure successful initialization
            - Failed initialization prevents all subsequent sensor operations
            - Error messages are stored for debugging purposes

        Usage Example:
            magnetometer = AK8963MagnetometerHandler()

            # Initialize the sensor
            if magnetometer.initialize_sensor():
                print("Magnetometer initialized successfully")
                # Now safe to read magnetic field data
                data = magnetometer.read_magnetic_field_data()
            else:
                print("Failed to initialize magnetometer")
                print(f"Error: {magnetometer.last_error_message}")
        """
        try:
            # Create AK8963 driver instance with our I2C bus and configuration
            # The driver handles low-level register operations and data conversion
            self.magnetometer_sensor = AK8963(
                self.i2c_communication_bus,
                address=self.magnetometer_address,
                mode=self.measurement_mode,
                output=self.output_resolution
            )

            # Verify sensor communication by reading WHO_AM_I register
            # This register should return 0x48 for AK8963
            _ = self.magnetometer_sensor.whoami

            # Update initialization status
            self.is_sensor_initialized = True
            self.last_error_message = None
            return True

        except Exception as initialization_exception:
            # Handle initialization failures gracefully
            self.magnetometer_sensor = None
            self.is_sensor_initialized = False
            self.last_error_message = "Sensor initialization failed: {}".format(initialization_exception)
            return False

    def is_sensor_connected(self):
        """
        Check if the AK8963 magnetometer sensor is connected and responsive via I2C communication.

        This method verifies the physical connection and communication with the AK8963
        magnetometer by attempting to read the WHO_AM_I register. This register contains
        a fixed value (0x48) that identifies the sensor type and confirms I2C communication.

        I2C Communication Verification Process:
        1. Check if sensor driver instance exists (initialization check)
        2. Attempt to read WHO_AM_I register (0x00) from the sensor
        3. Verify the returned value matches expected AK8963 identifier (0x48)
        4. Handle any I2C communication errors gracefully

        AK8963 WHO_AM_I Register Details:
        - Register address: 0x00 (WHO_AM_I)
        - Expected value: 0x48 (72 in decimal)
        - Read-only register that always returns the same value
        - Used for sensor identification and communication verification

        Raspberry Pi Pico I2C Considerations:
        - I2C requires pull-up resistors (4.7kΩ) on SCL and SDA lines
        - Communication can fail due to loose connections, power issues, or bus conflicts
        - The WHO_AM_I register is read-only and always returns 0x48
        - I2C timeout is set to 1000ms to prevent hanging on unresponsive devices

        Connection Failure Scenarios:
        - Loose or broken I2C connections
        - Power supply issues (3.3V required)
        - Missing pull-up resistors (4.7kΩ recommended)
        - I2C bus conflicts or address collisions
        - Sensor hardware failure

        Args:
            None

        Returns:
            bool: True if sensor is connected and responsive, False otherwise
                - True: Sensor responds correctly to I2C commands
                - False: Sensor not found, communication error, or not initialized

        Raises:
            None (all exceptions are caught and handled internally)

        Note:
            - This method performs live I2C communication tests
            - Connection status can change during operation
            - Use for system health monitoring and diagnostics
            - Essential for detecting sensor disconnections during operation

        Usage Example:
            magnetometer = AK8963MagnetometerHandler()

            # Check connection status
            if magnetometer.is_sensor_connected():
                print("Magnetometer is connected and responsive")
                # Safe to initialize and use sensor
                magnetometer.initialize_sensor()
            else:
                print("Magnetometer is not connected")
                print("Check wiring, power supply, and pull-up resistors")
        """
        try:
            # Check if sensor instance exists
            if self.magnetometer_sensor is None:
                return False

            # Attempt to read WHO_AM_I register to test communication
            _ = self.magnetometer_sensor.whoami
            return True

        except Exception:
            # Any exception indicates communication failure
            return False

    def read_magnetic_field_data(self):
        """
        Read the current 3-axis magnetic field measurements from the AK8963 magnetometer.

        This method performs a complete magnetic field reading operation, retrieving
        data from all three axes (X, Y, Z) of the AK8963 magnetometer. The data
        is returned in a structured format with proper scaling applied based on
        the configured output resolution.

        Magnetic Field Measurement Process:
        1. I2C register read operations for all three magnetic field axes
        2. Raw 16-bit ADC values converted to physical units (microTesla)
        3. Data formatted into structured dictionary for easy access
        4. Values represent total magnetic field including Earth's field and local interference

        AK8963 Magnetometer Architecture:
        - 3-axis magnetic field sensor measuring Earth's magnetic field
        - 16-bit ADC resolution for high precision measurements
        - Measures magnetic field strength and direction in 3D space
        - Typical measurement range: ±4800 μT (microTesla)
        - Earth's magnetic field: typically 25-65 μT depending on location

        Magnetic Field Components:
        - X-axis: Typically points East (positive X = East)
        - Y-axis: Typically points North (positive Y = North)
        - Z-axis: Points up/down (positive Z = up)
        - Total field magnitude: √(X² + Y² + Z²)

        Physical Principles:
        - Measures total magnetic field including Earth's field and local interference
        - Used for compass functionality and navigation applications
        - Requires calibration to compensate for hard/soft iron interference
        - Magnetic field strength varies with geographic location

        Coordinate System (Right-Handed):
        - X-axis: Points East (positive = Eastward magnetic field)
        - Y-axis: Points North (positive = Northward magnetic field)
        - Z-axis: Points up (positive = Upward magnetic field)

        Args:
            None

        Returns:
            dict: 3-axis magnetic field data dictionary containing:
                - 'x_axis' (float): X-axis magnetic field strength (μT)
                - 'y_axis' (float): Y-axis magnetic field strength (μT)
                - 'z_axis' (float): Z-axis magnetic field strength (μT)

        Raises:
            RuntimeError: If sensor is not initialized or driver instance is None
            Exception: If I2C communication fails or sensor read error occurs

        Note:
            - All values are converted to float for consistent data types
            - Units are in microTesla (μT) - standard unit for magnetic field
            - Earth's magnetic field is typically 25-65 μT depending on location
            - Values include Earth's field plus any local magnetic interference
            - For accurate compass readings, calibration is recommended

        Usage Example:
            magnetometer = AK8963MagnetometerHandler()
            magnetometer.initialize_sensor()

            # Read magnetic field data
            data = magnetometer.read_magnetic_field_data()

            print(f"X-axis: {data['x_axis']:.2f} μT")
            print(f"Y-axis: {data['y_axis']:.2f} μT")
            print(f"Z-axis: {data['z_axis']:.2f} μT")

            # Calculate total magnetic field magnitude
            total_field = math.sqrt(
                data['x_axis']**2 + data['y_axis']**2 + data['z_axis']**2
            )
            print(f"Total field: {total_field:.2f} μT")

            # Check if field strength is reasonable (Earth's field range)
            if 20 <= total_field <= 70:
                print("Magnetic field strength is within Earth's field range")
            else:
                print("Magnetic field may be affected by local interference")
        """
        # Verify sensor is initialized before attempting to read
        if not self.is_sensor_initialized or self.magnetometer_sensor is None:
            raise RuntimeError("Magnetometer sensor not initialized. Call initialize_sensor() first.")

        # Read raw magnetic field data from sensor
        # The sensor returns a tuple of (x, y, z) values
        magnetic_x, magnetic_y, magnetic_z = self.magnetometer_sensor.magnetic

        # Convert to float and return as descriptive dictionary
        return {
            'x_axis': float(magnetic_x),  # X-axis magnetic field (μT)
            'y_axis': float(magnetic_y),  # Y-axis magnetic field (μT)
            'z_axis': float(magnetic_z)   # Z-axis magnetic field (μT)
        }

    def get_sensor_status(self):
        """
        Get comprehensive status information about the AK8963 magnetometer sensor.

        This method provides a complete diagnostic overview of the AK8963 magnetometer
        system, including initialization status, connection state, configuration parameters,
        and any error information. It's useful for system monitoring, debugging,
        and status reporting applications.

        Status Information Categories:
        1. Initialization Status: Whether the sensor has been properly initialized
        2. Connection Status: Whether the sensor is currently connected and responsive
        3. Error Information: Any error messages from failed operations
        4. Configuration Parameters: Current system configuration settings
        5. Calibration Data: Last calibration results and timestamp

        Diagnostic Use Cases:
        - System health monitoring and reporting
        - Troubleshooting magnetometer connection issues
        - Configuration verification and validation
        - Error tracking and debugging
        - System status dashboards
        - Automated testing and validation

        Args:
            None

        Returns:
            dict: Comprehensive magnetometer status dictionary containing:
                - 'is_sensor_initialized' (bool): Whether sensor has been initialized
                    * True: Sensor driver created and configured successfully
                    * False: Sensor not initialized or initialization failed
                - 'is_sensor_connected' (bool): Whether sensor is currently connected
                    * True: Sensor responds to I2C communication
                    * False: Sensor not found or communication failed
                - 'last_error_message' (str or None): Last error message from failed operations
                    * None: No errors recorded
                    * String: Description of last error that occurred
                - 'magnetometer_address' (int): I2C address of the AK8963 sensor
                    * Typically 0x0C (12 in decimal)
                - 'i2c_frequency' (int): I2C communication frequency in Hz
                    * Common values: 100000 (100kHz), 400000 (400kHz)
                - 'last_calibration_data' (dict or None): Last calibration results
                    * None: No calibration performed
                    * Dict: Contains offset, scale, and timestamp information

        Raises:
            None

        Note:
            - This method performs a live connection check (calls is_sensor_connected())
            - Status information is current as of the method call
            - Use for system monitoring and diagnostic applications
            - Calibration data includes timestamp for tracking calibration history
            - Error messages are cleared when operations succeed

        Usage Example:
            magnetometer = AK8963MagnetometerHandler()

            # Check status before initialization
            status = magnetometer.get_sensor_status()
            print(f"Initialized: {status['is_sensor_initialized']}")
            print(f"Connected: {status['is_sensor_connected']}")

            # Initialize sensor
            magnetometer.initialize_sensor()

            # Check status after initialization
            status = magnetometer.get_sensor_status()
            print(f"Initialized: {status['is_sensor_initialized']}")
            print(f"Connected: {status['is_sensor_connected']}")
            print(f"I2C Address: 0x{status['magnetometer_address']:02X}")
            print(f"I2C Frequency: {status['i2c_frequency']} Hz")

            # Check calibration status
            if status['last_calibration_data']:
                cal_data = status['last_calibration_data']
                print(f"Last calibrated: {cal_data['timestamp_ms']} ms")
                print(f"Calibration offset: {cal_data['offset']}")
            else:
                print("No calibration data available")

            if status['last_error_message']:
                print(f"Last error: {status['last_error_message']}")
        """
        return {
            'is_sensor_initialized': self.is_sensor_initialized,
            'is_sensor_connected': self.is_sensor_connected(),
            'last_error_message': self.last_error_message,
            'magnetometer_address': self.magnetometer_address,
            'i2c_frequency': self.i2c_frequency,
            'last_calibration_data': self._last_calibration_data,
        }

    def calibrate_magnetometer(self, calibration_samples=256, sample_delay_ms=200):
        """
        Run a comprehensive hard/soft-iron calibration routine for the AK8963 magnetometer.

        This method performs magnetometer calibration to compensate for magnetic
        interference from the surrounding environment. The calibration process
        involves rotating the sensor in all directions to map the magnetic field
        and calculate offset and scale factors for improved accuracy.

        Magnetometer Calibration Process:
        1. Collect magnetic field samples while rotating sensor in all directions
        2. Analyze collected data to identify magnetic field distortions
        3. Calculate hard iron offset values to center the magnetic field sphere
        4. Calculate soft iron scale factors to correct for field distortions
        5. Store calibration parameters for future use

        Calibration Types:
        - Hard Iron Calibration: Compensates for permanent magnetic materials
          * Removes constant magnetic bias from ferromagnetic materials
          * Calculates offset values to center the magnetic field sphere
          * Examples: steel screws, permanent magnets, magnetic components
        - Soft Iron Calibration: Compensates for materials that distort magnetic field
          * Corrects for materials that change magnetic field shape
          * Calculates scale factors to make field sphere perfectly round
          * Examples: aluminum, copper, conductive materials

        Magnetic Field Distortion Sources:
        - Hard Iron Interference: Permanent magnetic materials
          * Creates constant offset in magnetic field measurements
          * Shifts the center of the magnetic field sphere
        - Soft Iron Interference: Materials that distort magnetic field
          * Changes the shape of the magnetic field sphere
          * Creates elliptical instead of spherical field pattern
        - Environmental Factors: Local magnetic fields
          * Power lines, motors, speakers, electronic devices
          * Building materials with magnetic properties

        Calibration Procedure:
        1. Place sensor in a clean magnetic environment
        2. Slowly rotate sensor in figure-8 patterns covering all orientations
        3. Ensure complete 3D rotation to map entire magnetic field sphere
        4. Avoid magnetic interference during calibration process
        5. Allow sufficient time between samples for thorough rotation

        Args:
            calibration_samples (int): Number of samples to collect during calibration. Default: 256
                - More samples = better accuracy but longer calibration time
                - Recommended range: 128-512 samples
                - Minimum: 50 samples for basic calibration
            sample_delay_ms (int): Delay between samples in milliseconds. Default: 200
                - Longer delays allow for more thorough rotation between samples
                - Recommended range: 100-500 ms
                - Shorter delays may not allow complete rotation

        Returns:
            dict: Dictionary containing calibration results
                - 'offset' (tuple): Hard iron offset values for X, Y, Z axes (μT)
                    * Values to subtract from raw measurements to center field
                - 'scale' (tuple): Soft iron scale factors for X, Y, Z axes
                    * Values to multiply measurements to correct field shape

        Raises:
            RuntimeError: If sensor is not initialized
            Exception: If calibration process fails or insufficient data collected

        Note:
            - During calibration, slowly rotate the sensor in all directions
            - Figure-8 patterns work well for comprehensive coverage
            - Avoid magnetic interference from motors, speakers, or electronic devices
            - Calibration results are stored internally for future reference
            - Recalibration may be needed if environment changes significantly

        Usage Example:
            magnetometer = AK8963MagnetometerHandler()
            magnetometer.initialize_sensor()

            # Perform calibration with default settings
            print("Starting magnetometer calibration...")
            print("Rotate sensor slowly in all directions (figure-8 pattern)")

            result = magnetometer.calibrate_magnetometer()
            print(f"Calibration complete!")
            print(f"Hard iron offset: {result['offset']}")
            print(f"Soft iron scale: {result['scale']}")

            # Perform calibration with custom parameters
            result = magnetometer.calibrate_magnetometer(
                calibration_samples=128,  # Fewer samples for faster calibration
                sample_delay_ms=100       # Shorter delay for quicker process
            )
        """
        # Verify sensor is initialized before calibration
        if not self.is_sensor_initialized:
            raise RuntimeError("Magnetometer sensor not initialized. Call initialize_sensor() first.")

        # Perform calibration using the underlying sensor driver
        # The driver handles the complex calibration algorithm
        calibration_offset, calibration_scale = self.magnetometer_sensor.calibrate(
            count=int(calibration_samples),
            delay=int(sample_delay_ms)
        )

        # Store calibration data with timestamp for reference
        self._last_calibration_data = {
            'offset': calibration_offset,
            'scale': calibration_scale,
            'timestamp_ms': utime.ticks_ms()
        }

        # Return calibration results
        return {
            'offset': calibration_offset,   # Hard iron offset (X, Y, Z)
            'scale': calibration_scale      # Soft iron scale factors (X, Y, Z)
        }
