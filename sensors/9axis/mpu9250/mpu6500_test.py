"""
MPU6500 Inertial Measurement Unit Handler Test and Demonstration Script

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

This script demonstrates the functionality of the MPU6500InertialMeasurementUnitHandler class
by running various tests and demonstrations. It's designed to help users understand
how to use the 6-axis IMU sensor and verify that it's working correctly.

=== WHAT IS AN INERTIAL MEASUREMENT UNIT (IMU)? ===
An IMU is a sensor that measures motion and orientation. The MPU6500 is a 6-axis IMU containing:
1. Accelerometer: Measures linear acceleration (including gravity) in 3 axes (X, Y, Z)
2. Gyroscope: Measures angular velocity (rotation rate) in 3 axes (X, Y, Z)

=== APPLICATIONS OF IMU SENSORS ===
- Robotics: Balance control, navigation, and orientation sensing
- Drones: Flight stabilization and attitude control
- Gaming: Motion controllers and gesture recognition
- Mobile Devices: Screen rotation and step counting
- Industrial: Vibration monitoring and equipment health

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
- Basic sensor initialization and data reading
- Accelerometer and gyroscope data interpretation
- Tilt angle calculations from accelerometer data
- Complementary filtering for smooth angle estimates
- Filter configuration and control
- Continuous monitoring and data logging

Prerequisites:
- MPU6500 IMU sensor connected to Raspberry Pi Pico
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
devices to communicate over just two wires (SCL and SDA). The MPU6500 uses I2C
to send sensor data to the Raspberry Pi Pico. Pull-up resistors are required
to ensure proper signal levels.

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
# Import our custom IMU handler class that encapsulates all MPU6500 operations
# This handler provides a high-level interface to interact with the 6-axis IMU sensor
from mpu6500_handler import MPU6500InertialMeasurementUnitHandler  # MPU6500 IMU handler class


def demonstrate_basic_imu_functionality():
    """
    Demonstrate basic IMU sensor reading functionality.

    This function tests the fundamental operations of the 6-axis IMU:
    - Sensor initialization (establishes I2C communication and configures registers)
    - Temperature reading (internal sensor temperature for compensation)
    - Accelerometer data reading (3-axis linear acceleration measurements)
    - Gyroscope data reading (3-axis angular velocity measurements)
    - Data validation and interpretation (verifies sensor response quality)

    === ACCELEROMETER EXPLANATION ===
    The accelerometer measures linear acceleration in m/s². When stationary:
    - One axis should show ~9.81 m/s² (gravity pointing downward)
    - Other axes should show ~0 m/s² (no horizontal acceleration)
    - Total magnitude should be close to 9.81 m/s² (gravity only)

    === GYROSCOPE EXPLANATION ===
    The gyroscope measures angular velocity in degrees per second (°/s). When stationary:
    - All axes should show values close to 0 °/s
    - Any non-zero values indicate rotation or sensor drift
    - High values might indicate sensor movement or calibration issues

    === TEMPERATURE COMPENSATION ===
    IMU sensors are sensitive to temperature changes. The internal temperature
    reading helps compensate for temperature-related drift in measurements.

    Returns:
        bool: True if test passed, False if test failed

    Note:
        This test verifies that the sensor can be initialized and basic
        sensor readings can be obtained. The accelerometer should show
        gravity (~9.81 m/s²) when stationary, and gyroscope should show
        near-zero values when not rotating.
    """
    print("\n=== MPU6500 IMU: Basic Functionality Test ===")

    # === CREATE IMU HANDLER INSTANCE ===
    # The handler encapsulates all low-level I2C communication and sensor configuration
    # This object-oriented approach makes the code more maintainable and reusable
    imu_handler = MPU6500InertialMeasurementUnitHandler()

    # === ATTEMPT SENSOR INITIALIZATION ===
    # Initialization involves:
    # 1. Setting up I2C communication on the specified pins
    # 2. Configuring sensor registers for proper operation
    # 3. Performing self-tests to verify sensor functionality
    if not imu_handler.initialize_sensor():
        print("IMU sensor initialization failed.")
        print("Check wiring: SCL->GPIO21, SDA->GPIO20, VCC->3.3V, GND->GND")
        print("Ensure 4.7kΩ pull-up resistors are connected to SCL and SDA")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === READ AND DISPLAY TEMPERATURE ===
    # Temperature reading is important for sensor compensation
    # The MPU6500 has an internal temperature sensor for this purpose
    temperature = imu_handler.get_temperature_reading()
    print("Temperature: {:.2f} °C".format(temperature))

    # === READ ACCELEROMETER DATA ===
    # Get 3-axis acceleration measurements in m/s²
    # When stationary, one axis should show gravity (~9.81 m/s²)
    accelerometer_data = imu_handler.get_accelerometer_data()
    print("Accelerometer (m/s²): x={:.3f}, y={:.3f}, z={:.3f}".format(
        accelerometer_data['x_axis'], accelerometer_data['y_axis'], accelerometer_data['z_axis']))

    # === READ GYROSCOPE DATA ===
    # Get 3-axis angular velocity measurements in degrees per second
    # When stationary, all values should be close to 0 °/s
    gyroscope_data = imu_handler.get_gyroscope_data()
    print("Gyroscope (°/s): x={:.2f}, y={:.2f}, z={:.2f}".format(
        gyroscope_data['x_axis'], gyroscope_data['y_axis'], gyroscope_data['z_axis']))

    # === VALIDATE ACCELEROMETER READINGS ===
    # Calculate total acceleration magnitude using Pythagorean theorem
    # This should be close to 9.81 m/s² when stationary (gravity only)
    total_acceleration = (accelerometer_data['x_axis']**2 + accelerometer_data['y_axis']**2 + accelerometer_data['z_axis']**2)**0.5
    print("Total acceleration magnitude: {:.3f} m/s²".format(total_acceleration))

    # Check if acceleration is within expected range for gravity + small movements
    if 8.0 < total_acceleration < 12.0:  # Reasonable range for gravity + small movements
        print("[OK]Accelerometer readings are within expected range (gravity ≈ 9.81 m/s²)")
    else:
        print("⚠ Accelerometer readings may be outside normal range")

    # === VALIDATE GYROSCOPE READINGS ===
    # Calculate total angular velocity magnitude
    # This should be close to 0 °/s when stationary
    total_angular_velocity = (gyroscope_data['x_axis']**2 + gyroscope_data['y_axis']**2 + gyroscope_data['z_axis']**2)**0.5
    print("Total angular velocity magnitude: {:.2f} °/s".format(total_angular_velocity))

    # Check if gyroscope readings are within expected range
    if total_angular_velocity < 10.0:  # Should be near zero when stationary
        print("[OK]Gyroscope readings are within expected range (near zero when stationary)")
    else:
        print("⚠ Gyroscope readings may indicate sensor movement or drift")

    return True


def demonstrate_angle_calculation():
    """
    Demonstrate tilt angle calculation from accelerometer data.

    This function shows how to calculate orientation angles from accelerometer
    measurements and how to use complementary filtering for smoother results.

    === TILT ANGLE CALCULATION EXPLANATION ===
    Tilt angles describe the orientation of the sensor relative to gravity:
    - Pitch: Rotation around X-axis (forward/backward tilt)
    - Roll: Rotation around Y-axis (left/right tilt)
    - Yaw: Rotation around Z-axis (compass direction - not accurate from accelerometer alone)

    === COMPLEMENTARY FILTERING ===
    A complementary filter combines accelerometer and gyroscope data:
    - Accelerometer: Accurate for long-term orientation but noisy for quick movements
    - Gyroscope: Accurate for quick movements but drifts over time
    - Filter: Uses both sensors to get smooth, accurate orientation estimates

    The filter formula: angle = α × (angle + gyro × dt) + (1-α) × accel_angle
    Where α is the filter coefficient (0-1) that balances between sensors.

    Returns:
        bool: True if test passed, False if test failed
    """
    print("\n=== MPU6500 IMU: Angle Calculation Test ===")

    # === CREATE AND INITIALIZE IMU HANDLER ===
    imu_handler = MPU6500InertialMeasurementUnitHandler()
    if not imu_handler.initialize_sensor():
        print("IMU sensor initialization failed.")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === CALCULATE RAW TILT ANGLES ===
    # These angles are calculated directly from accelerometer data
    # They represent the current orientation based on gravity direction
    raw_tilt_angles = imu_handler.get_tilt_angles_from_accelerometer()
    print("Raw tilt angles: pitch={:.2f}, roll={:.2f}, yaw={:.2f}".format(
        raw_tilt_angles['pitch_angle'], raw_tilt_angles['roll_angle'], raw_tilt_angles['yaw_angle']))

    # === CALCULATE FILTERED ANGLES ===
    # These angles use complementary filtering to combine accelerometer and gyroscope data
    # They provide smoother, more accurate orientation estimates
    filtered_angles = imu_handler.update_complementary_filtered_angles()
    print("Complementary filtered: pitch={:.2f}, roll={:.2f}".format(
        filtered_angles['pitch_angle'], filtered_angles['roll_angle']))

    return True


def demonstrate_comprehensive_data_reading():
    """
    Demonstrate comprehensive sensor data reading.

    This function shows how to get all sensor data in a single call, which is
    more efficient than making multiple individual sensor readings. It's useful
    for applications that need synchronized data from all sensors.

    === COMPREHENSIVE DATA STRUCTURE ===
    The comprehensive data includes:
    - Timestamp: When the data was collected (in milliseconds)
    - Temperature: Internal sensor temperature for compensation
    - Accelerometer: 3-axis linear acceleration data
    - Gyroscope: 3-axis angular velocity data
    - Tilt angles: Raw angles calculated from accelerometer
    - Filtered angles: Smoothed angles using complementary filtering

    === DATA SYNCHRONIZATION ===
    Getting all data in one call ensures that all measurements are taken
    at the same time, which is important for accurate sensor fusion and
    orientation calculations.

    Returns:
        bool: True if test passed, False if test failed
    """
    print("\n=== MPU6500 IMU: Comprehensive Data Reading Test ===")

    # === CREATE AND INITIALIZE IMU HANDLER ===
    imu_handler = MPU6500InertialMeasurementUnitHandler()
    if not imu_handler.initialize_sensor():
        print("IMU sensor initialization failed.")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === GET COMPREHENSIVE SENSOR DATA ===
    # This single call retrieves all sensor data at once, ensuring synchronization
    comprehensive_data = imu_handler.get_comprehensive_sensor_data()

    # === DISPLAY TIMESTAMP ===
    # Timestamp helps track when measurements were taken
    print("Timestamp (ms):", comprehensive_data['timestamp_ms'])

    # === DISPLAY TEMPERATURE ===
    print("Temperature: {:.2f} °C".format(comprehensive_data['temperature_celsius']))

    # === DISPLAY ACCELEROMETER DATA ===
    # Extract accelerometer data from the comprehensive data structure
    accelerometer_data = comprehensive_data['accelerometer_data']
    print("Accelerometer: x={:.3f}, y={:.3f}, z={:.3f}".format(
        accelerometer_data['x_axis'], accelerometer_data['y_axis'], accelerometer_data['z_axis']))

    # === DISPLAY GYROSCOPE DATA ===
    # Extract gyroscope data from the comprehensive data structure
    gyroscope_data = comprehensive_data['gyroscope_data']
    print("Gyroscope: x={:.2f}, y={:.2f}, z={:.2f}".format(
        gyroscope_data['x_axis'], gyroscope_data['y_axis'], gyroscope_data['z_axis']))

    # === DISPLAY TILT ANGLES ===
    # Raw angles calculated directly from accelerometer data
    tilt_angles = comprehensive_data['tilt_angles']
    print("Tilt angles: pitch={:.2f}, roll={:.2f}, yaw={:.2f}".format(
        tilt_angles['pitch_angle'], tilt_angles['roll_angle'], tilt_angles['yaw_angle']))

    # === DISPLAY FILTERED ANGLES ===
    # Smoothed angles using complementary filtering
    filtered_angles = comprehensive_data['filtered_angles']
    print("Filtered angles: pitch={:.2f}, roll={:.2f}".format(
        filtered_angles['pitch_angle'], filtered_angles['roll_angle']))

    return True


def demonstrate_filter_configuration():
    """
    Demonstrate complementary filter configuration and control.

    This function shows how to configure the complementary filter parameters
    and reset the filter state. The filter coefficient determines how much
    weight is given to accelerometer vs gyroscope data.

    === COMPLEMENTARY FILTER COEFFICIENT ===
    The filter coefficient (α) controls the balance between sensors:
    - α = 0.95: More weight to gyroscope (responsive to quick movements, may drift)
    - α = 0.05: More weight to accelerometer (stable long-term, slower response)
    - α = 0.9: Good balance for most applications

    === FILTER RESET ===
    Resetting the filter clears the internal state and starts fresh.
    This is useful when the sensor orientation changes significantly or
    when you want to recalibrate the filter.

    Returns:
        bool: True if test passed, False if test failed
    """
    print("\n=== MPU6500 IMU: Filter Configuration Test ===")

    # === CREATE IMU HANDLER WITH CUSTOM FILTER COEFFICIENT ===
    # Initialize with a specific filter coefficient (0.95 = more gyroscope weight)
    imu_handler = MPU6500InertialMeasurementUnitHandler(complementary_filter_coefficient=0.95)

    if not imu_handler.initialize_sensor():
        print("IMU sensor initialization failed.")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === DISPLAY INITIAL SENSOR STATUS ===
    # This shows the current filter configuration and sensor state
    print("Initial sensor status:", imu_handler.get_sensor_status())

    # === MODIFY FILTER COEFFICIENT ===
    # Change the filter coefficient to 0.9 (slightly more accelerometer weight)
    imu_handler.set_complementary_filter_coefficient(0.9)

    # === RESET FILTER STATE ===
    # Clear the internal filter state and start fresh
    imu_handler.reset_complementary_filter()

    # === DISPLAY UPDATED SENSOR STATUS ===
    # Show the new filter configuration and reset state
    print("Updated sensor status:", imu_handler.get_sensor_status())

    return True


def demonstrate_continuous_monitoring():
    """
    Demonstrate continuous IMU monitoring for 20 seconds with 1-second intervals.

    This function shows how to continuously monitor IMU data over time, which is
    useful for real-time applications like robotics, drones, or motion tracking.

    === CONTINUOUS MONITORING APPLICATIONS ===
    Continuous monitoring is essential for:
    1. Real-time Control Systems: Providing continuous feedback for stabilization
    2. Motion Tracking: Recording movement patterns over time
    3. Data Logging: Collecting sensor data for analysis
    4. Performance Monitoring: Tracking system behavior during operation
    5. Safety Systems: Detecting unusual movements or orientations

    === MONITORING PARAMETERS ===
    - Duration: 20 seconds provides enough time to observe sensor behavior
    - Sampling Rate: 1 second intervals balance responsiveness with processing load
    - Data Display: Shows key metrics (temperature, pitch, roll) for quick assessment

    Returns:
        bool: True if test passed, False if test failed
    """
    # === SET UP MONITORING PARAMETERS ===
    monitoring_duration_ms = 40000  # Monitor for 40 seconds (40,000 milliseconds)
    sampling_interval_ms = 1000     # Read data every 1 second (1,000 milliseconds)

    print(f"\n=== MPU6500 IMU: Continuous Monitoring ({monitoring_duration_ms//1000} seconds) ===")

    # === CREATE AND INITIALIZE IMU HANDLER ===
    imu_handler = MPU6500InertialMeasurementUnitHandler()
    if not imu_handler.initialize_sensor():
        print("IMU sensor initialization failed.")
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
        # === GET COMPREHENSIVE SENSOR DATA ===
        # Retrieve all sensor data in one synchronized call
        comprehensive_data = imu_handler.get_comprehensive_sensor_data()
        measurement_count += 1  # Increment measurement counter

        # === BLINK ONBOARD LED AFTER READING DATA ===
        # Blink the LED once to indicate successful data reading in this cycle
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # === DISPLAY COMPREHENSIVE DATA IN SPECIFIED FORMAT ===
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

        # Raw tilt angles
        tilt_angles = comprehensive_data['tilt_angles']
        print("Raw tilt angles: pitch={:.2f}, roll={:.2f}, yaw={:.2f}".format(
            tilt_angles['pitch_angle'], tilt_angles['roll_angle'], tilt_angles['yaw_angle']))

        # Complementary filtered angles
        filtered_angles = comprehensive_data['filtered_angles']
        print("Complementary filtered: Pitch:{:.1f}° Roll:{:.1f}°".format(
            filtered_angles['pitch_angle'], filtered_angles['roll_angle']))
        print()

        # === WAIT BEFORE NEXT MEASUREMENT ===
        # utime.sleep_ms() pauses execution without blocking the system
        utime.sleep_ms(sampling_interval_ms)

    return True


def main():
    """
    Run all IMU demonstrations and report results.

    This is the main entry point that orchestrates all the demonstration functions.
    It provides a structured way to test all IMU functionality and gives
    users a comprehensive overview of the sensor's capabilities.

    === DEMONSTRATION FLOW ===
    1. Basic Functionality: Tests sensor initialization and basic readings
    2. Angle Calculation: Demonstrates tilt angle calculations and filtering
    3. Comprehensive Data: Shows synchronized data collection from all sensors
    4. Filter Configuration: Demonstrates filter parameter adjustment
    5. Continuous Monitoring: Shows real-time data collection capabilities

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

        # === RUN ALL DEMONSTRATIONS ===
        # Each demonstration function returns True if successful, False if failed
        # We append the results to our tracking list for later analysis
        demonstration_results.append(("Basic Functionality", demonstrate_basic_imu_functionality()))
        demonstration_results.append(("Angle Calculation", demonstrate_angle_calculation()))
        demonstration_results.append(("Comprehensive Data", demonstrate_comprehensive_data_reading()))
        demonstration_results.append(("Filter Configuration", demonstrate_filter_configuration()))
        demonstration_results.append(("Continuous Monitoring", demonstrate_continuous_monitoring()))

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
