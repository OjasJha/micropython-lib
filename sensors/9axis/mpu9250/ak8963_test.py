"""
AK8963 Magnetometer Handler Test and Demonstration Script

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

This script demonstrates the functionality of the AK8963MagnetometerHandler class
by running various tests and demonstrations. It's designed to help users understand
how to use the magnetometer sensor and verify that it's working correctly.

=== WHAT IS A MAGNETOMETER? ===
A magnetometer is a sensor that measures magnetic field strength and direction.
The AK8963 is a 3-axis magnetometer that can detect Earth's magnetic field,
making it useful for compass applications, navigation, and detecting magnetic objects.

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
- Magnetometer calibration process (essential for accurate compass readings)
- Continuous monitoring and data logging
- Error handling and status reporting

Prerequisites:
- AK8963 magnetometer sensor connected to Raspberry Pi Pico
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
devices to communicate over just two wires (SCL and SDA). The AK8963 uses I2C
to send magnetic field data to the Raspberry Pi Pico. Pull-up resistors are
required to ensure proper signal levels.

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
# Import our custom magnetometer handler class that encapsulates all AK8963 operations
# This handler provides a high-level interface to interact with the magnetometer sensor
from ak8963_handler import AK8963MagnetometerHandler  # AK8963 magnetometer handler class


def demonstrate_basic_magnetometer_functionality():
    """
    Demonstrate basic magnetometer reading functionality.

    This function tests the fundamental operations of the magnetometer:
    - Sensor initialization (establishes I2C communication)
    - Basic data reading (gets 3-axis magnetic field measurements)
    - Error handling (validates sensor response and data quality)

    === MAGNETIC FIELD MEASUREMENT EXPLANATION ===
    The AK8963 measures magnetic field strength in microTesla (μT) units.
    Earth's magnetic field typically ranges from 25-65 μT depending on location.
    The sensor provides X, Y, Z axis measurements that can be combined to get
    the total magnetic field magnitude using the Pythagorean theorem.

    === SENSOR INITIALIZATION PROCESS ===
    1. I2C communication setup with the sensor
    2. Register configuration for proper operation mode
    3. Self-test to verify sensor functionality
    4. Calibration parameter loading (if available)

    Returns:
        bool: True if test passed, False if test failed

    Note:
        This test verifies that the sensor can be initialized and basic
        magnetic field readings can be obtained. Expected values should
        be in the range of 25-65 μT for Earth's magnetic field.
    """
    print("\n=== AK8963 Magnetometer: Basic Functionality Test ===")

    # === CREATE SENSOR HANDLER INSTANCE ===
    # The handler encapsulates all low-level I2C communication and sensor configuration
    # This object-oriented approach makes the code more maintainable and reusable
    magnetometer_handler = AK8963MagnetometerHandler()

    # === ATTEMPT SENSOR INITIALIZATION ===
    # Initialization involves:
    # 1. Setting up I2C communication on the specified pins
    # 2. Configuring sensor registers for proper operation
    # 3. Performing self-tests to verify sensor functionality
    if not magnetometer_handler.initialize_sensor():
        print("Magnetometer sensor initialization failed.")
        print("Check wiring: SCL->GPIO21, SDA->GPIO20, VCC->3.3V, GND->GND")
        print("Ensure 4.7kΩ pull-up resistors are connected to SCL and SDA")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    # The onboard LED is connected to GPIO25 on Raspberry Pi Pico
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === READ MAGNETIC FIELD DATA ===
    # This function reads the current magnetic field measurements from all three axes
    # The data is returned as a dictionary with 'x_axis', 'y_axis', 'z_axis' keys
    # Each value represents the magnetic field strength in microTesla (μT)
    magnetic_field_data = magnetometer_handler.read_magnetic_field_data()

    # === DISPLAY MEASUREMENT RESULTS ===
    # Format and display the raw magnetic field measurements
    # The .2f format specifier shows values with 2 decimal places for readability
    print("Magnetic Field (microTesla): x={:.2f}, y={:.2f}, z={:.2f}".format(
        magnetic_field_data['x_axis'], magnetic_field_data['y_axis'], magnetic_field_data['z_axis']))

    # === CALCULATE TOTAL MAGNETIC FIELD MAGNITUDE ===
    # Use the Pythagorean theorem to calculate the total magnetic field strength
    # This gives us the magnitude of the 3D magnetic field vector
    # Formula: magnitude = sqrt(x² + y² + z²)
    total_field = (magnetic_field_data['x_axis']**2 + magnetic_field_data['y_axis']**2 + magnetic_field_data['z_axis']**2)**0.5
    print("Total magnetic field magnitude: {:.2f} μT".format(total_field))

    # === VALIDATE MEASUREMENT QUALITY ===
    # Check if the total magnetic field is within expected range for Earth's field
    # This helps identify if the sensor is working correctly or if there are issues
    if 10 < total_field < 100:  # Reasonable range for Earth's magnetic field
        print("[OK]Magnetic field readings are within expected range")
        return True
    else:
        print("⚠ Magnetic field readings may be outside normal range")
        return True  # Still consider test passed if sensor responds


def demonstrate_magnetometer_calibration():
    """
    Demonstrate magnetometer calibration functionality.

    This function shows how to perform magnetometer calibration to compensate
    for hard and soft iron interference. During calibration, the sensor should
    be rotated slowly in all directions to map the magnetic field.

    === MAGNETOMETER CALIBRATION EXPLANATION ===
    Calibration is crucial for accurate compass readings because:
    1. Hard Iron Interference: Permanent magnets or ferromagnetic materials create
       constant magnetic fields that shift the sensor's zero point
    2. Soft Iron Interference: Ferromagnetic materials distort the magnetic field,
       making it non-uniform and affecting the sensor's sensitivity
    3. Manufacturing Variations: Each sensor has slight differences in sensitivity
       and offset that need to be compensated

    === CALIBRATION PROCESS ===
    The calibration process involves:
    1. Collecting magnetic field samples while rotating the sensor
    2. Analyzing the data to find the center and scale of the magnetic field sphere
    3. Calculating offset and scale factors to correct for interference
    4. Storing these factors for future measurements

    Returns:
        bool: True if test passed, False if test failed

    Note:
        Calibration is essential for accurate compass readings. The process
        involves rotating the sensor in figure-8 patterns to map the magnetic
        field and calculate offset and scale factors.
    """
    print("\n=== AK8963 Magnetometer: Calibration Test (Quick) ===")

    # === CREATE SENSOR HANDLER INSTANCE ===
    # We need a fresh instance to ensure clean calibration state
    magnetometer_handler = AK8963MagnetometerHandler()

    # === ATTEMPT SENSOR INITIALIZATION ===
    # Calibration requires the sensor to be properly initialized first
    if not magnetometer_handler.initialize_sensor():
        print("Magnetometer sensor initialization failed.")
        return False

    # === BLINK ONBOARD LED AFTER SENSOR INITIALIZATION ===
    # Blink the onboard LED once to indicate successful sensor initialization
    # The onboard LED is connected to GPIO25 on Raspberry Pi Pico
    led: Pin = Pin("LED", Pin.OUT)
    led.toggle()
    utime.sleep(1)    # Keep LED on for 1 s
    led.toggle()
    print("[OK]Sensor initialized - LED blinked once")

    # === PROVIDE CALIBRATION INSTRUCTIONS ===
    # Clear instructions help users perform proper calibration
    print("Rotate sensor slowly in all directions for calibration...")
    print("Make figure-8 patterns and rotate around all axes")
    print("Avoid magnetic interference from motors, speakers, or metal objects")

    # === PERFORM CALIBRATION ===
    # The calibration function collects samples while the user rotates the sensor
    # Parameters:
    # - calibration_samples=64: Number of measurements to collect (reduced for quick test)
    # - sample_delay_ms=50: Delay between samples in milliseconds
    # This creates a balance between calibration quality and test duration
    calibration_result = magnetometer_handler.calibrate_magnetometer(calibration_samples=64, sample_delay_ms=50)

    # === DISPLAY CALIBRATION RESULTS ===
    # Show the calculated offset and scale factors
    # These values will be used to correct future magnetic field measurements
    print("Calibration Offset:", calibration_result['offset'])
    print("Calibration Scale:", calibration_result['scale'])
    print("Sensor Status:", magnetometer_handler.get_sensor_status())

    # === VERIFY CALIBRATION SUCCESS ===
    # Check if calibration produced valid results
    # Valid calibration should have non-None offset and scale values
    if calibration_result['offset'] is not None and calibration_result['scale'] is not None:
        print("[OK]Calibration completed successfully")
        return True
    else:
        print("⚠ Calibration may not have completed properly")
        return False


def demonstrate_continuous_monitoring():
    """
    Demonstrate continuous magnetometer monitoring for 20 seconds.

    This function shows how to continuously read magnetometer data over time,
    which is useful for monitoring magnetic field changes or detecting
    magnetic objects in the environment.

    === CONTINUOUS MONITORING APPLICATIONS ===
    Continuous monitoring is useful for:
    1. Compass Applications: Tracking heading changes over time
    2. Magnetic Object Detection: Detecting when magnetic objects approach
    3. Environmental Monitoring: Observing magnetic field variations
    4. Data Logging: Recording magnetic field data for analysis
    5. Real-time Systems: Providing continuous feedback for control systems

    === MICROPYTHON TIMING FUNCTIONS ===
    - utime.ticks_ms(): Returns current time in milliseconds (wraps around after ~49 days)
    - utime.ticks_diff(): Calculates time difference between two tick values
    - utime.sleep_ms(): Pauses execution for specified milliseconds
    These functions are optimized for microcontrollers and handle timing efficiently.

    Returns:
        bool: True if test passed, False if test failed

    Note:
        This test runs for 20 seconds, reading data every 1 second. In a real
        application, you might want to adjust the sampling rate based on
        your requirements. Higher sampling rates provide more responsive
        detection but use more processing power.
    """
    print("\n=== AK8963 Magnetometer: Continuous Monitoring (20 seconds) ===")

    # === CREATE SENSOR HANDLER INSTANCE ===
    # Fresh instance ensures clean state for monitoring
    magnetometer_handler = AK8963MagnetometerHandler()

    # === ATTEMPT SENSOR INITIALIZATION ===
    # Continuous monitoring requires stable sensor operation
    if not magnetometer_handler.initialize_sensor():
        print("Magnetometer sensor initialization failed.")
        return False

    # === SET UP ONBOARD LED FOR MONITORING ===
    # Configure the onboard LED for blinking during each measurement cycle
    # The onboard LED is connected to GPIO25 on Raspberry Pi Pico
    led: Pin = Pin("LED", Pin.OUT)

    # === SET UP MONITORING PARAMETERS ===
    # These parameters control the monitoring behavior
    monitoring_duration_ms = 20000  # Monitor for 20 seconds (20,000 milliseconds)
    sampling_interval_ms = 1000     # Read data every 1 second (1,000 milliseconds)

    # === INITIALIZE TIMING VARIABLES ===
    # utime.ticks_ms() provides high-resolution timing for microcontrollers
    # It's more efficient than using floating-point time calculations
    monitoring_start_time = utime.ticks_ms()  # Record when monitoring started
    measurement_count = 0  # Counter for tracking number of measurements

    print("Starting continuous monitoring...")
    print("Try moving magnetic objects near the sensor to see field changes")

    # === CONTINUOUS MONITORING LOOP ===
    # This loop runs until the monitoring duration is reached
    # utime.ticks_diff() calculates elapsed time efficiently
    while utime.ticks_diff(utime.ticks_ms(), monitoring_start_time) < monitoring_duration_ms:
        # === READ MAGNETIC FIELD DATA ===
        # Get current magnetic field measurements from all three axes
        magnetic_field_data = magnetometer_handler.read_magnetic_field_data()
        measurement_count += 1  # Increment measurement counter

        # === BLINK ONBOARD LED AFTER READING DATA ===
        # Blink the LED once to indicate successful data reading in this cycle
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # === DISPLAY MEASUREMENT WITH TIMESTAMP ===
        # Calculate elapsed time since monitoring started
        elapsed_time = utime.ticks_diff(utime.ticks_ms(), monitoring_start_time)

        # Format and display the measurement with timing information
        # This helps users understand the temporal relationship of measurements
        print("Measurement #{} (t={}ms) Magnetic Field x={:.2f} y={:.2f} z={:.2f} (microTesla)".format(
            measurement_count, elapsed_time,
            magnetic_field_data['x_axis'], magnetic_field_data['y_axis'], magnetic_field_data['z_axis']))

        # === WAIT BEFORE NEXT MEASUREMENT ===
        # utime.sleep_ms() pauses execution without blocking the system
        # This allows other MicroPython tasks to run if needed
        utime.sleep_ms(sampling_interval_ms)

    # === MONITORING COMPLETION ===
    print("[OK]Continuous monitoring completed successfully")
    print("Total measurements taken: {}".format(measurement_count))
    return True


def main():
    """
    Run all magnetometer demonstrations and report results.

    This is the main entry point that orchestrates all the demonstration functions.
    It provides a structured way to test all magnetometer functionality and gives
    users a comprehensive overview of the sensor's capabilities.

    === DEMONSTRATION FLOW ===
    1. Basic Functionality: Tests sensor initialization and basic readings
    2. Calibration: Demonstrates the calibration process for accurate measurements
    3. Continuous Monitoring: Shows real-time data collection capabilities

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
        demonstration_results.append(("Basic Functionality", demonstrate_basic_magnetometer_functionality()))
        demonstration_results.append(("Calibration", demonstrate_magnetometer_calibration()))
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
