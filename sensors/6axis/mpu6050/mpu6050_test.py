"""
MPU6050 Handler Test and Demonstration Program for Raspberry Pi Pico 2 W

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

PROGRAM OVERVIEW:
================
This demonstration program showcases the MPU6050Handler class, which provides a
high-level, object-oriented interface for the MPU6050 Inertial Measurement Unit (IMU).
The handler encapsulates all the functionality from the original test program into
a reusable class that can be easily integrated into larger projects.

KEY FEATURES DEMONSTRATED:
==========================
1. OBJECT-ORIENTED SENSOR INTERFACE:
   - Clean, intuitive API for sensor operations
   - Automatic initialization and configuration
   - Built-in error handling and status monitoring

2. COMPREHENSIVE DATA ACCESS:
   - Single method call for all sensor data
   - Individual data access methods for specific needs
   - Timestamped readings for data logging

3. ADVANCED SENSOR FUSION:
   - Automatic complementary filtering
   - Configurable filter coefficients
   - Stable angle estimation without drift

4. EASY INTEGRATION:
   - Simple setup with configurable parameters
   - Status monitoring and error reporting
   - Reset and configuration methods

HARDWARE SETUP REQUIREMENTS:
============================
- Raspberry Pi Pico 2 W microcontroller
- MPU6050 IMU sensor module
- I2C connections: SDA to GPIO 20, SCL to GPIO 21
- 3.3V power supply and common ground
- Pull-up resistors on I2C lines (often built into sensor modules)

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

# Custom MPU6050 Handler Class
from mpu6050_handler import MPU6050Handler

# MicroPython Time Utilities
import utime  # For delays and timing

# MicroPython Pin Control
from machine import Pin

# ============================================================================
# DEMONSTRATION FUNCTIONS
# ============================================================================

def demonstrate_basic_usage():
    """
    Demonstrate basic MPU6050Handler usage with simple data reading.

    This function shows the most common use case: creating a handler,
    initializing the sensor, and reading data.
    """
    print("\n" + "=" * 60)
    print("🔧 BASIC USAGE DEMONSTRATION")
    print("=" * 60)

    try:
        # Create handler with default settings
        print("Creating MPU6050Handler with default settings...")
        mpu = MPU6050Handler()

        # Initialize the sensor
        print("Initializing sensor...")
        if not mpu.initialize():
            print("❌ Sensor initialization failed!")
            return False

        # Blink LED after sensor initialization
        led: Pin = Pin("LED", Pin.OUT)
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # Read and display basic data
        print("Reading sensor data...")
        temperature = mpu.get_temperature()
        acceleration = mpu.get_acceleration()
        angular_velocity = mpu.get_angular_velocity()

        print(f"🌡️  Temperature: {temperature:.2f} °C")
        print(f"📊 Acceleration: X={acceleration['x']:.3f}g, Y={acceleration['y']:.3f}g, Z={acceleration['z']:.3f}g")
        print(f"🔄 Angular Velocity: X={angular_velocity['x']:.2f}°/s, Y={angular_velocity['y']:.2f}°/s, Z={angular_velocity['z']:.2f}°/s")

        return True

    except Exception as e:
        print(f"❌ Basic usage demonstration failed: {e}")
        return False

def demonstrate_angle_calculations():
    """
    Demonstrate tilt angle calculations and sensor fusion.

    This function shows how to get both raw tilt angles and
    filtered angles using the handler's built-in sensor fusion.
    """
    print("\n" + "=" * 60)
    print("📐 ANGLE CALCULATIONS DEMONSTRATION")
    print("=" * 60)

    try:
        # Create and initialize handler
        mpu = MPU6050Handler()
        if not mpu.initialize():
            print("❌ Sensor initialization failed!")
            return False

        # Blink LED after sensor initialization
        led: Pin = Pin("LED", Pin.OUT)
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # Get tilt angles from accelerometer
        print("Calculating tilt angles from accelerometer...")
        tilt_angles = mpu.get_tilt_angles()
        print(f"📐 Raw Tilt Angles:")
        print(f"   Pitch: {tilt_angles['pitch']:.2f}°")
        print(f"   Roll:  {tilt_angles['roll']:.2f}°")
        print(f"   Yaw:   {tilt_angles['yaw']:.2f}°")

        # Get filtered angles using sensor fusion
        print("\nApplying sensor fusion with complementary filter...")
        filtered_angles = mpu.get_filtered_angles()
        print(f"🎯 Filtered Angles (Sensor Fusion):")
        print(f"   Pitch: {filtered_angles['pitch']:.2f}°")
        print(f"   Roll:  {filtered_angles['roll']:.2f}°")

        return True

    except Exception as e:
        print(f"❌ Angle calculations demonstration failed: {e}")
        return False

def demonstrate_comprehensive_data():
    """
    Demonstrate comprehensive data access in a single call.

    This function shows how to get all sensor data, calculated angles,
    and filtered angles in one method call.
    """
    print("\n" + "=" * 60)
    print("📊 COMPREHENSIVE DATA DEMONSTRATION")
    print("=" * 60)

    try:
        # Create and initialize handler
        mpu = MPU6050Handler()
        if not mpu.initialize():
            print("❌ Sensor initialization failed!")
            return False

        # Blink LED after sensor initialization
        led: Pin = Pin("LED", Pin.OUT)
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # Get all data in one call
        print("Reading comprehensive sensor data...")
        data = mpu.get_comprehensive_data()

        # Display all data in organized format
        print(f"⏰ Timestamp: {data['timestamp_ms']} ms")
        print(f"🌡️  Temperature: {data['temperature']:.2f} °C")

        print(f"\n📊 Linear Acceleration:")
        print(f"   X: {data['accelerometer']['x']:.3f}g")
        print(f"   Y: {data['accelerometer']['y']:.3f}g")
        print(f"   Z: {data['accelerometer']['z']:.3f}g")

        print(f"\n🔄 Angular Velocity:")
        print(f"   X: {data['gyroscope']['x']:.2f}°/s")
        print(f"   Y: {data['gyroscope']['y']:.2f}°/s")
        print(f"   Z: {data['gyroscope']['z']:.2f}°/s")

        print(f"\n📐 Raw Tilt Angles:")
        print(f"   Pitch: {data['tilt_angles']['pitch']:.2f}°")
        print(f"   Roll:  {data['tilt_angles']['roll']:.2f}°")
        print(f"   Yaw:   {data['tilt_angles']['yaw']:.2f}°")

        print(f"\n🎯 Filtered Angles (Sensor Fusion):")
        print(f"   Pitch: {data['filtered_angles']['pitch']:.2f}°")
        print(f"   Roll:  {data['filtered_angles']['roll']:.2f}°")

        return True

    except Exception as e:
        print(f"❌ Comprehensive data demonstration failed: {e}")
        return False

def demonstrate_configuration():
    """
    Demonstrate handler configuration and status monitoring.

    This function shows how to configure the handler parameters
    and monitor its status.
    """
    print("\n" + "=" * 60)
    print("⚙️  CONFIGURATION DEMONSTRATION")
    print("=" * 60)

    try:
        # Create handler with custom configuration
        print("Creating handler with custom configuration...")
        mpu = MPU6050Handler(
            scl_pin=21,           # Custom SCL pin
            sda_pin=20,           # Custom SDA pin
            i2c_freq=400000,      # Custom I2C frequency
            device_address=0x68,  # Custom device address
            filter_coefficient=0.95,  # Custom filter coefficient
            i2c_id=0              # Custom I2C controller
        )

        # Initialize the sensor
        if not mpu.initialize():
            print("❌ Sensor initialization failed!")
            return False

        # Blink LED after sensor initialization
        led: Pin = Pin("LED", Pin.OUT)
        led.toggle()
        utime.sleep(1)    # Keep LED on for 1 s
        led.toggle()

        # Display current status
        print("\nCurrent handler status:")
        status = mpu.get_status()
        print(f"   Initialized: {status['is_initialized']}")
        print(f"   Connected: {status['is_connected']}")
        print(f"   Last Error: {status['last_error']}")
        print(f"   Filter Coefficient: {status['filter_coefficient']}")
        print(f"   Device Address: 0x{status['device_address']:02X}")
        print(f"   I2C Frequency: {status['i2c_frequency']} Hz")

        # Demonstrate filter coefficient change
        print("\nChanging filter coefficient...")
        mpu.set_filter_coefficient(0.90)
        print("Filter coefficient changed to 0.90 (more stable)")

        # Demonstrate filter reset
        print("\nResetting sensor fusion filter...")
        mpu.reset_filter()
        print("Filter reset to zero state")

        return True

    except Exception as e:
        print(f"❌ Configuration demonstration failed: {e}")
        return False

def demonstrate_real_time_monitoring():
    """
    Demonstrate real-time sensor monitoring with the handler.

    This function shows how to use the handler for continuous
    sensor monitoring with clean, readable code.
    """
    print("\n" + "=" * 60)
    print("🎯 REAL-TIME MONITORING DEMONSTRATION")
    print("=" * 60)
    print("📡 Monitoring sensor data for 10 seconds...")
    print("⏹️  Press Ctrl+C to stop early")
    print("=" * 60)

    try:
        # Create and initialize handler
        mpu = MPU6050Handler()
        if not mpu.initialize():
            print("❌ Sensor initialization failed!")
            return False

        # Initialize LED for blinking
        led: Pin = Pin("LED", Pin.OUT)

        # Monitor for 10 seconds
        start_time = utime.ticks_ms()
        reading_count = 0

        while utime.ticks_diff(utime.ticks_ms(), start_time) < 10000:  # 10 seconds
            # Get comprehensive data
            data = mpu.get_comprehensive_data()
            reading_count += 1

            # Blink LED for every cycle
            led.toggle()
            utime.sleep(1)    # Keep LED on for 1 s
            led.toggle()

            # Display data
            print(f"\n📈 Reading #{reading_count} - {data['timestamp_ms']} ms")
            print(f"🌡️  Temp: {data['temperature']:.1f}°C")
            print(f"📐 Tilt: Pitch={data['tilt_angles']['pitch']:.1f}°, Roll={data['tilt_angles']['roll']:.1f}°")
            print(f"🎯 Filtered: Pitch={data['filtered_angles']['pitch']:.1f}°, Roll={data['filtered_angles']['roll']:.1f}°")
            print(f"📊 Accel: X={data['accelerometer']['x']:.2f}g, Y={data['accelerometer']['y']:.2f}g, Z={data['accelerometer']['z']:.2f}g")

            # Wait 1 second between readings
            utime.sleep(1)

        print(f"\n✅ Monitoring completed! Total readings: {reading_count}")
        return True

    except KeyboardInterrupt:
        print(f"\n🛑 Monitoring stopped by user after {reading_count} readings")
        return True
    except Exception as e:
        print(f"❌ Real-time monitoring failed: {e}")
        return False

# ============================================================================
# MAIN PROGRAM FUNCTION
# ============================================================================

def main():
    """
    Main program function demonstrating MPU6050Handler capabilities.

    This function runs through all the demonstration functions to showcase
    the different ways the MPU6050Handler can be used.
    """
    print("🚀 MPU6050Handler Demonstration Program")
    print("=" * 60)
    print("This program demonstrates the MPU6050Handler class capabilities")
    print("for easy integration of MPU6050 sensor functionality.")
    print("=" * 60)

    # Track demonstration results
    results = []

    try:
        # Run all demonstrations
        print("\n🎬 Starting demonstration sequence...")

        # 1. Basic Usage
        results.append(("Basic Usage", demonstrate_basic_usage()))

        # 2. Angle Calculations
        results.append(("Angle Calculations", demonstrate_angle_calculations()))

        # 3. Comprehensive Data
        results.append(("Comprehensive Data", demonstrate_comprehensive_data()))

        # 4. Configuration
        results.append(("Configuration", demonstrate_configuration()))

        # 5. Real-time Monitoring
        results.append(("Real-time Monitoring", demonstrate_real_time_monitoring()))

        # Display results summary
        print("\n" + "=" * 60)
        print("📋 DEMONSTRATION RESULTS SUMMARY")
        print("=" * 60)

        successful = 0
        for name, success in results:
            status = "✅ PASSED" if success else "❌ FAILED"
            print(f"{name:.<30} {status}")
            if success:
                successful += 1

        print(f"\nOverall: {successful}/{len(results)} demonstrations passed")

        if successful == len(results):
            print("🎉 All demonstrations completed successfully!")
            print("The MPU6050Handler is ready for integration into your projects.")
        else:
            print("⚠️  Some demonstrations failed. Check sensor connections and try again.")

    except KeyboardInterrupt:
        print("\n\n🛑 Program terminated by user")
        print("✅ Demonstration stopped gracefully")

    except Exception as e:
        print(f"\n❌ Unexpected error occurred: {e}")
        print("🔧 Check sensor connections and try again")

# ============================================================================
# PROGRAM ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    """
    Program entry point - executes when script is run directly.

    This demonstration program showcases the MPU6050Handler class and
    provides examples of how to integrate it into larger projects.
    """
    main()
