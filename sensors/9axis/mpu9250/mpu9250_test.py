"""
MPU9250 9-Axis Inertial Measurement Unit Handler Test and Demonstration Script

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

This script demonstrates the functionality of the MPU9250NineAxisInertialMeasurementUnitHandler class
by running various tests and demonstrations. It's designed to help users understand
how to use the 9-axis IMU sensor and verify that it's working correctly.

=== WHAT IS A 9-AXIS IMU? ===
A 9-axis IMU combines three types of sensors to provide complete motion and orientation data:
1. Accelerometer (3-axis): Measures linear acceleration including gravity
2. Gyroscope (3-axis): Measures angular velocity (rotation rate)
3. Magnetometer (3-axis): Measures magnetic field strength and direction

=== MPU9250 SENSOR ARCHITECTURE ===
The MPU9250 is actually two separate sensors in one package:
- MPU6500: Contains the 6-axis IMU (accelerometer + gyroscope) at I2C address 0x68
- AK8963: Contains the 3-axis magnetometer at I2C address 0x0C
- Both sensors share the same I2C bus but have different addresses

=== APPLICATIONS OF 9-AXIS IMU ===
- Drones: Complete flight control with attitude and heading
- Robotics: Full 3D orientation and navigation
- Virtual Reality: Head tracking and motion sensing
- Augmented Reality: Device orientation and movement tracking
- Navigation Systems: Compass functionality with tilt compensation
- Gaming: Motion controllers with full 3D movement detection

=== RASPBERRY PI PICO CONTEXT ===
The Raspberry Pi Pico is a microcontroller board with:
- RP2040 dual-core ARM Cortex-M0+ processor
- 264KB RAM, 2MB flash memory
- Multiple GPIO pins for sensor connections
- Built-in I2C, SPI, UART communication interfaces
- 3.3V logic levels (important for sensor compatibility)

=== MICROPYTHON CONTEXT ===
MicroPython is a Python implementation for microcontrollers that provides:
- Python syntax for embedded programming
- Built-in libraries for hardware interfaces (I2C, SPI, GPIO)
- Real-time capabilities suitable for sensor applications
- Simplified development compared to C/C++

Key Test Areas:
- Basic sensor initialization and data reading from both MPU6500 and AK8963
- Comprehensive sensor data including accelerometer, gyroscope, magnetometer, and temperature
- Tilt angle calculations and complementary filtering
- Heading calculation using magnetometer data
- Continuous monitoring and data logging
- Sensor status and health monitoring

Prerequisites:
- MPU9250 9-axis IMU sensor connected to Raspberry Pi Pico
- Proper I2C wiring (SCL to GPIO21, SDA to GPIO20)
- 4.7kΩ pull-up resistors on I2C lines
- 3.3V power supply

Hardware Setup:
- VCC: Connect to 3.3V power supply
- GND: Connect to ground
- SCL: Connect to GPIO21 (with 4.7kΩ pull-up to 3.3V)
- SDA: Connect to GPIO20 (with 4.7kΩ pull-up to 3.3V)

=== I2C COMMUNICATION EXPLANATION ===
I2C (Inter-Integrated Circuit) is a communication protocol that allows multiple
devices to communicate over just two wires (SCL and SDA). The MPU9250 uses I2C
to send sensor data to the Raspberry Pi Pico. Pull-up resistors are required
to ensure proper signal levels.

Note: The MPU9250 contains two sensors (MPU6500 and AK8963) that share the same I2C bus
but have different addresses (0x68 and 0x0C respectively).

Author: Ojas Jha
License: MIT License (see above)
Date: October 19, 2025
Version: 2.0
"""

# =============================================================================
# IMPORTS - Categorized by functionality
# =============================================================================

# === MICROPYTHON TIME UTILITIES ===
# utime provides time-related functions for MicroPython, similar to Python's time module
# but optimized for microcontrollers with limited resources
import utime  # MicroPython time utilities for delays and timestamps

# === MICROPYTHON HARDWARE CONTROL ===
# machine module provides direct access to hardware features like GPIO pins and LEDs
from machine import Pin  # GPIO pin control for LED blinking

# === CUSTOM SENSOR HANDLER ===
# Import our custom 9-axis IMU handler class that encapsulates all MPU9250 operations
# This handler provides a high-level interface to interact with both the MPU6500 and AK8963 sensors
from mpu9250_handler import MPU9250NineAxisInertialMeasurementUnitHandler  # MPU9250 9-axis IMU handler class


def demonstrate_basic_nine_axis_functionality():
    """
    Demonstrate basic 9-axis IMU sensor reading functionality.

    This function tests the fundamental operations of the 9-axis IMU:
    - Sensor initialization for both MPU6500 and AK8963 (dual sensor setup)
    - Temperature reading (internal sensor temperature for compensation)
    - Accelerometer, gyroscope, and magnetometer data reading (all 9 axes)
    - Tilt angle calculations from accelerometer data
    - Complementary filtered angle estimates (smooth orientation)
    - Heading calculation from magnetometer data (compass functionality)
    - Data validation and interpretation (sensor health check)

    === 9-AXIS SENSOR FUSION ===
    The 9-axis IMU combines data from three sensor types:
    1. Accelerometer: Provides gravity reference for tilt calculation
    2. Gyroscope: Provides rotation rate for smooth angle tracking
    3. Magnetometer: Provides Earth's magnetic field for heading calculation

    === HEADING CALCULATION ===
    Heading (compass direction) is calculated using magnetometer data:
    - Uses atan2() function to calculate angle from magnetic field components
    - Compensates for tilt using accelerometer data
    - Provides 0-360° compass reading relative to magnetic north

    === SENSOR VALIDATION ===
    Each sensor type has expected ranges when stationary:
    - Accelerometer: ~9.81 m/s² total magnitude (gravity)
    - Gyroscope: ~0 °/s total magnitude (no rotation)
    - Magnetometer: ~25-65 μT total magnitude (Earth's field)

    Returns:
        bool: True if test passed, False if test failed

    Note:
        This test verifies that both sensors can be initialized and comprehensive
        sensor data can be obtained. The accelerometer should show gravity (~9.81 m/s²)
        when stationary, gyroscope should show near-zero values when not rotating,
        and magnetometer should show Earth's magnetic field (~25-65 μT).
    """
    print("\n=== MPU9250 9-Axis IMU: Basic Functionality Test ===")

    # === CREATE 9-AXIS IMU HANDLER INSTANCE ===
    # The handler manages both MPU6500 and AK8963 sensors as a unified system
    # This object-oriented approach simplifies the complex dual-sensor setup
    nine_axis_imu_handler = MPU9250NineAxisInertialMeasurementUnitHandler()

    # === ATTEMPT DUAL SENSOR INITIALIZATION ===
    # Initialization involves:
    # 1. Setting up I2C communication for both sensors
    # 2. Configuring MPU6500 registers (accelerometer + gyroscope)
    # 3. Configuring AK8963 registers (magnetometer)
    # 4. Performing self-tests on both sensors
    if not nine_axis_imu_handler.initialize_sensor():
        print("9-axis IMU sensor initialization failed.")
        print("Check wiring: SCL->GPIO21, SDA->GPIO20, VCC->3.3V, GND->GND")
        print("Ensure 4.7kΩ pull-up resistors are connected to SCL and SDA")
        print("Verify both MPU6500 (0x68) and AK8963 (0x0C) are responding")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === GET COMPREHENSIVE SENSOR DATA ===
    # This single call retrieves data from all 9 axes plus temperature
    # Data is synchronized to ensure all measurements are taken at the same time
    comprehensive_data = nine_axis_imu_handler.get_comprehensive_sensor_data()

    # === DISPLAY TEMPERATURE ===
    # Temperature reading is important for sensor compensation
    print("Temperature: {:.2f} °C".format(comprehensive_data['temperature_celsius']))

    # === DISPLAY ACCELEROMETER DATA ===
    # 3-axis linear acceleration measurements in m/s²
    # When stationary, one axis should show gravity (~9.81 m/s²)
    accelerometer_data = comprehensive_data['accelerometer_data']
    print("Accelerometer (m/s²): x={:.3f} y={:.3f} z={:.3f}".format(
        accelerometer_data['x_axis'], accelerometer_data['y_axis'], accelerometer_data['z_axis']))

    # === DISPLAY GYROSCOPE DATA ===
    # 3-axis angular velocity measurements in degrees per second
    # When stationary, all values should be close to 0 °/s
    gyroscope_data = comprehensive_data['gyroscope_data']
    print("Gyroscope (°/s): x={:.2f} y={:.2f} z={:.2f}".format(
        gyroscope_data['x_axis'], gyroscope_data['y_axis'], gyroscope_data['z_axis']))

    # === DISPLAY MAGNETOMETER DATA ===
    # 3-axis magnetic field measurements in microTesla
    # Should show Earth's magnetic field (~25-65 μT total magnitude)
    magnetometer_data = comprehensive_data['magnetometer_data']
    print("Magnetometer (μT): x={:.2f} y={:.2f} z={:.2f}".format(
        magnetometer_data['x_axis'], magnetometer_data['y_axis'], magnetometer_data['z_axis']))

    # === DISPLAY TILT ANGLES ===
    # Raw angles calculated directly from accelerometer data
    # These represent the sensor's orientation relative to gravity
    tilt_angles = comprehensive_data['tilt_angles']
    print("Tilt angles (°): pitch={:.2f} roll={:.2f} yaw={:.2f}".format(
        tilt_angles['pitch_angle'], tilt_angles['roll_angle'], tilt_angles['yaw_angle']))

    # === DISPLAY FILTERED ANGLES ===
    # Smoothed angles using complementary filtering
    # These provide more stable orientation estimates for control applications
    filtered_angles = comprehensive_data['filtered_angles']
    print("Filtered angles (°): pitch={:.2f} roll={:.2f}".format(
        filtered_angles['pitch_angle'], filtered_angles['roll_angle']))

    # === DISPLAY HEADING ===
    # Compass direction calculated from magnetometer data
    # Provides 0-360° reading relative to magnetic north
    heading = comprehensive_data['heading_degrees']
    print("Heading: {:.1f}°".format(heading))

    # === VALIDATE ACCELEROMETER READINGS ===
    # Calculate total acceleration magnitude using Pythagorean theorem
    # This should be close to 9.81 m/s² when stationary (gravity only)
    total_acceleration = (accelerometer_data['x_axis']**2 + accelerometer_data['y_axis']**2 + accelerometer_data['z_axis']**2)**0.5
    print("Total acceleration magnitude: {:.3f} m/s²".format(total_acceleration))

    if 8.0 < total_acceleration < 12.0:
        print("[OK]Accelerometer readings are within expected range")
    else:
        print("⚠ Accelerometer readings may be outside normal range")

    # === VALIDATE MAGNETOMETER READINGS ===
    # Calculate total magnetic field magnitude
    # This should be within Earth's magnetic field range
    total_magnetic_field = (magnetometer_data['x_axis']**2 + magnetometer_data['y_axis']**2 + magnetometer_data['z_axis']**2)**0.5
    print("Total magnetic field magnitude: {:.2f} μT".format(total_magnetic_field))

    if 10 < total_magnetic_field < 100:
        print("[OK]Magnetometer readings are within expected range")
    else:
        print("⚠ Magnetometer readings may be outside normal range")

    # === VALIDATE GYROSCOPE READINGS ===
    # Calculate total angular velocity magnitude
    # This should be close to 0 °/s when stationary
    total_angular_velocity = (gyroscope_data['x_axis']**2 + gyroscope_data['y_axis']**2 + gyroscope_data['z_axis']**2)**0.5
    print("Total angular velocity magnitude: {:.2f} °/s".format(total_angular_velocity))

    if total_angular_velocity < 10.0:
        print("[OK]Gyroscope readings are within expected range")
    else:
        print("⚠ Gyroscope readings may indicate sensor movement or drift")

    return True


def demonstrate_continuous_nine_axis_monitoring():
    """
    Demonstrate continuous 9-axis IMU monitoring for 20 seconds with 1-second intervals.

    This function shows how to continuously monitor all 9-axis IMU data over time,
    which is essential for real-time applications like drones, robotics, or
    motion tracking systems.

    === CONTINUOUS MONITORING APPLICATIONS ===
    Continuous 9-axis monitoring is crucial for:
    1. Flight Control Systems: Real-time attitude and heading control for drones
    2. Robotics: Continuous orientation feedback for balance and navigation
    3. Motion Tracking: Recording complete 3D movement patterns
    4. Navigation Systems: Real-time compass and orientation updates
    5. Virtual Reality: Continuous head tracking for immersive experiences
    6. Data Logging: Collecting comprehensive motion data for analysis

    === MONITORING PARAMETERS ===
    - Duration: 20 seconds provides enough time to observe sensor behavior
    - Sampling Rate: 1 second intervals balance responsiveness with processing load
    - Data Display: Shows key metrics (pitch, roll, heading, temperature) for quick assessment

    === 9-AXIS DATA INTEGRATION ===
    The continuous monitoring demonstrates how all sensor data works together:
    - Pitch/Roll: From accelerometer + gyroscope fusion
    - Heading: From magnetometer with tilt compensation
    - Temperature: For sensor compensation and health monitoring

    Returns:
        bool: True if test passed, False if test failed
    """
    # === SET UP MONITORING PARAMETERS ===
    monitoring_duration_ms = 40000  # Monitor for 40 seconds (40,000 milliseconds)
    sampling_interval_ms = 1000     # Read data every 1 second (1,000 milliseconds)

    print(f"\n=== MPU9250 9-Axis IMU: Continuous Monitoring ({monitoring_duration_ms//1000} seconds) ===")

    # === CREATE AND INITIALIZE 9-AXIS IMU HANDLER ===
    nine_axis_imu_handler = MPU9250NineAxisInertialMeasurementUnitHandler()
    if not nine_axis_imu_handler.initialize_sensor():
        print("9-axis IMU sensor initialization failed.")
        return False

    # === SET UP ONBOARD LED FOR MONITORING ===
    # Configure the onboard LED for blinking during each measurement cycle
    led: Pin = Pin("LED", Pin.OUT)

    # === INITIALIZE TIMING VARIABLES ===
    # utime.ticks_ms() provides high-resolution timing for microcontrollers
    monitoring_start_time = utime.ticks_ms()  # Record when monitoring started
    measurement_count = 0  # Counter for tracking number of measurements

    # === CONTINUOUS MONITORING LOOP ===
    # This loop runs until the monitoring duration is reached
    while utime.ticks_diff(utime.ticks_ms(), monitoring_start_time) < monitoring_duration_ms:
        # === GET COMPREHENSIVE 9-AXIS SENSOR DATA ===
        # Retrieve all sensor data in one synchronized call
        # This includes accelerometer, gyroscope, magnetometer, and temperature
        comprehensive_data = nine_axis_imu_handler.get_comprehensive_sensor_data()
        measurement_count += 1  # Increment measurement counter

        # === BLINK ONBOARD LED AFTER READING DATA ===
        # Blink the LED once to indicate successful data reading in this cycle
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # === DISPLAY COMPREHENSIVE 9-AXIS DATA IN SPECIFIED FORMAT ===
        # Show all sensor data in a structured format for detailed analysis
        print("Measurement #{}".format(measurement_count))
        print("------------------------------------------------------")

        # Temperature
        print("Temperature: {:.2f} °C".format(comprehensive_data['temperature_celsius']))

        # Accelerometer data with total magnitude
        accel_data = comprehensive_data['accelerometer_data']
        total_accel = (accel_data['x_axis']**2 + accel_data['y_axis']**2 + accel_data['z_axis']**2)**0.5
        print("Accelerometer (m/s²): x={:.3f}, y={:.3f}, z={:.3f}".format(
            accel_data['x_axis'], accel_data['y_axis'], accel_data['z_axis']))
        print("Total acceleration magnitude: {:.3f} m/s²".format(total_accel))
        print()

        # Gyroscope data with total magnitude
        gyro_data = comprehensive_data['gyroscope_data']
        total_gyro = (gyro_data['x_axis']**2 + gyro_data['y_axis']**2 + gyro_data['z_axis']**2)**0.5
        print("Gyroscope (°/s): x={:.2f}, y={:.2f}, z={:.2f}".format(
            gyro_data['x_axis'], gyro_data['y_axis'], gyro_data['z_axis']))
        print("Total angular velocity magnitude: {:.2f} °/s".format(total_gyro))
        print()

        # Magnetometer data with total magnitude
        mag_data = comprehensive_data['magnetometer_data']
        total_mag = (mag_data['x_axis']**2 + mag_data['y_axis']**2 + mag_data['z_axis']**2)**0.5
        print("Magnetometer (μT): x={:.2f}, y={:.2f}, z={:.2f}".format(
            mag_data['x_axis'], mag_data['y_axis'], mag_data['z_axis']))
        print("Total magnetic field magnitude: {:.2f} μT".format(total_mag))
        print()

        # Raw tilt angles
        tilt_angles = comprehensive_data['tilt_angles']
        print("Raw tilt angles: pitch={:.2f}, roll={:.2f}, yaw={:.2f}".format(
            tilt_angles['pitch_angle'], tilt_angles['roll_angle'], tilt_angles['yaw_angle']))

        # Complementary filtered angles
        filtered_angles = comprehensive_data['filtered_angles']
        print("Complementary filtered: Pitch:{:.1f}° Roll:{:.1f}°".format(
            filtered_angles['pitch_angle'], filtered_angles['roll_angle']))

        # Heading (compass direction)
        heading = comprehensive_data['heading_degrees']
        print("Heading: {:.1f}°".format(heading))
        print()

        # === WAIT BEFORE NEXT MEASUREMENT ===
        # utime.sleep_ms() pauses execution without blocking the system
        utime.sleep_ms(sampling_interval_ms)

    return True


def demonstrate_sensor_status_information():
    """
    Demonstrate sensor status information retrieval.

    This function shows how to get detailed status information about both sensors
    in the MPU9250 package. Status information is crucial for debugging and
    monitoring sensor health in production systems.

    === SENSOR STATUS INFORMATION ===
    The status information typically includes:
    - Initialization Status: Whether both sensors are properly initialized
    - Communication Status: I2C communication health for both sensors
    - Calibration Status: Whether sensors have been calibrated
    - Filter Status: Current filter configuration and state
    - Error Information: Any detected errors or warnings
    - Performance Metrics: Sensor response times and data quality

    === STATUS MONITORING APPLICATIONS ===
    Status monitoring is important for:
    1. System Health Monitoring: Detecting sensor failures early
    2. Debugging: Identifying communication or configuration issues
    3. Quality Control: Ensuring sensors meet performance requirements
    4. Maintenance: Planning sensor calibration and replacement
    5. Safety Systems: Triggering alerts when sensors malfunction

    Returns:
        bool: True if test passed, False if test failed
    """
    print("\n=== MPU9250 9-Axis IMU: Sensor Status Information ===")

    # === CREATE AND INITIALIZE 9-AXIS IMU HANDLER ===
    nine_axis_imu_handler = MPU9250NineAxisInertialMeasurementUnitHandler()

    # === ATTEMPT SENSOR INITIALIZATION ===
    if not nine_axis_imu_handler.initialize_sensor():
        print("9-axis IMU sensor initialization failed.")
        # Even if initialization fails, we can still get status information
        # This helps diagnose what went wrong during initialization
        print("Status after failed initialization:")
        print(nine_axis_imu_handler.get_sensor_status())
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === DISPLAY COMPREHENSIVE STATUS INFORMATION ===
    # Get and display detailed status information for both sensors
    # This includes initialization status, communication health, and configuration
    print("Sensor Status Information:")
    print(nine_axis_imu_handler.get_sensor_status())

    return True


def main():
    """
    Run all 9-axis IMU demonstrations and report results.

    This is the main entry point that orchestrates all the demonstration functions.
    It provides a structured way to test all 9-axis IMU functionality and gives
    users a comprehensive overview of the sensor's capabilities.

    === DEMONSTRATION FLOW ===
    1. Basic Functionality: Tests dual sensor initialization and comprehensive data reading
    2. Continuous Monitoring: Shows real-time 9-axis data collection capabilities
    3. Sensor Status: Demonstrates status monitoring and health checking

    === 9-AXIS IMU TESTING SCOPE ===
    The demonstrations cover:
    - Dual sensor initialization (MPU6500 + AK8963)
    - All 9 axes of sensor data (3 accelerometer + 3 gyroscope + 3 magnetometer)
    - Sensor fusion and filtering
    - Heading calculation with tilt compensation
    - Continuous monitoring for real-time applications
    - Status monitoring for system health

    === ERROR HANDLING ===
    The function includes proper error handling for:
    - KeyboardInterrupt: Allows users to stop the program gracefully
    - General Exceptions: Catches and reports any unexpected errors
    - Individual Test Failures: Continues running other tests even if one fails

    Returns:
        bool: True if all demonstrations completed successfully, False otherwise
    """
    try:
        # === INITIALIZE DEMONSTRATION RESULTS LIST ===
        # This list stores tuples of (demonstration_name, success_status)
        # Using a list allows us to track results and provide a summary
        demonstration_results = []

        # === RUN ALL 9-AXIS IMU DEMONSTRATIONS ===
        # Each demonstration function returns True if successful, False if failed
        # We append the results to our tracking list for later analysis
        demonstration_results.append(("Basic Functionality", demonstrate_basic_nine_axis_functionality()))
        demonstration_results.append(("Continuous Monitoring", demonstrate_continuous_nine_axis_monitoring()))
        demonstration_results.append(("Sensor Status", demonstrate_sensor_status_information()))

        # === DISPLAY DEMONSTRATION SUMMARY ===
        # Provide a clear summary of all test results
        print("\n=== Demonstration Summary ===")
        successful_demonstrations = 0

        # === ITERATE THROUGH RESULTS ===
        # Process each demonstration result and display the status
        # The {:.<25} format creates left-aligned text with dots for padding
        for demonstration_name, demonstration_result in demonstration_results:
            status = "PASSED" if demonstration_result else "FAILED"
            print("{:.<25} {}".format(demonstration_name, status))

            # Count successful demonstrations for final summary
            if demonstration_result:
                successful_demonstrations += 1

        # === FINAL SUMMARY ===
        # Show overall success rate
        total_demonstrations = len(demonstration_results)
        print("Successfully completed {}/{} demonstrations".format(successful_demonstrations, total_demonstrations))

        # Return True if all demonstrations passed
        return successful_demonstrations == total_demonstrations

    except KeyboardInterrupt:
        # === HANDLE USER INTERRUPTION ===
        # KeyboardInterrupt occurs when user presses Ctrl+C
        # This allows graceful program termination
        print(f"\n🛑 Monitoring stopped by user")
        return True

    except Exception as e:
        # === HANDLE UNEXPECTED ERRORS ===
        # Catch any other exceptions that might occur during execution
        # This prevents the program from crashing and provides error information
        print(f"❌ Real-time monitoring failed: {e}")
        return False


# === PROGRAM ENTRY POINT ===
# This block ensures the main() function runs when the script is executed directly
# It's a Python convention that allows the script to be both imported and run standalone
if __name__ == "__main__":
    main()
