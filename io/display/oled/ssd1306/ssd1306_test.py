"""
Comprehensive SSD1306 OLED Display Test Suite for Raspberry Pi Pico 2 W

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

This comprehensive test script validates all aspects of SSD1306 OLED display functionality
using MicroPython on the Raspberry Pi Pico 2 W microcontroller. It serves as both a
validation tool and an educational example for OLED display programming.

=== EDUCATIONAL PURPOSE ===
This script demonstrates:
- Proper MicroPython module organization and documentation
- Hardware initialization and error handling patterns
- Graphics programming concepts (pixels, lines, rectangles)
- Animation techniques for embedded displays
- Memory management in resource-constrained environments
- I2C communication protocols and troubleshooting

=== TEST COVERAGE ===
1. Hardware Detection & Initialization
   - I2C bus scanning and device detection
   - Display initialization sequence validation
   - Error handling for connection issues

2. Basic Display Functions
   - Text rendering at multiple positions
   - Character set validation (letters, numbers, symbols)
   - Frame buffer management

3. Graphics Primitives
   - Individual pixel manipulation
   - Line drawing (horizontal, vertical, diagonal)
   - Rectangle drawing (outline and filled)

4. Display Control Features
   - Contrast adjustment (brightness control)
   - Display inversion (black/white swap)
   - Power management (display on/off)

5. Advanced Features
   - Real-time animation (bouncing ball physics)
   - Text scrolling effects
   - Performance benchmarking
   - Memory usage monitoring

6. Compatibility Testing
   - Backward compatibility with old method names
   - API consistency validation

=== HARDWARE REQUIREMENTS ===
- Raspberry Pi Pico 2 W (ARM Cortex-M33 dual-core @ 150MHz)
- SSD1306 OLED Display Module (128x64 pixels recommended)
- I2C Connections:
  * SCL (Serial Clock) → GPIO 7
  * SDA (Serial Data) → GPIO 6
  * VCC → 3.3V (or 5V if module supports it)
  * GND → Ground

=== RASPBERRY PI PICO 2 W SPECIFIC FEATURES ===
- 520KB SRAM (sufficient for multiple frame buffers)
- Flexible GPIO pin assignment for I2C/SPI
- Hardware I2C controllers with DMA support
- Built-in 3.3V logic (compatible with most OLED modules)
- WiFi capability for remote monitoring (not used in this test)

=== MICROPYTHON CONCEPTS DEMONSTRATED ===
- const() usage for memory-efficient constants
- bytearray() for efficient buffer management
- Garbage collection monitoring with gc module
- Exception handling for hardware failures
- Time-based operations with time module
- Object-oriented programming patterns

Author: Ojas Jha
License: MIT License (see above)
Date: October 18, 2025
Version: 2.0 - Enhanced with extensive documentation
"""

# ============================================================================
# IMPORTS AND DEPENDENCIES
# ============================================================================

# Hardware abstraction layer imports
from machine import Pin, SoftI2C  # GPIO and I2C hardware control
import ssd1306_handler as ssd1306  # SSD1306 OLED display driver
import time                       # Time delays and performance measurement
import gc                         # Garbage collection for memory monitoring


# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

"""
Configuration constants for hardware and test parameters.
These constants are defined at module level for easy customization and
to demonstrate best practices in embedded programming.

Why Use Constants:
- Centralized configuration management
- Prevents magic numbers in code
- Easy hardware adaptation for different setups
- Compile-time optimization in MicroPython
"""

# === HARDWARE CONFIGURATION ===
# I2C Communication Settings
I2C_SCL_PIN = 7              # Serial Clock Line GPIO pin
                            # Can be any GPIO pin on Pico 2 W
I2C_SDA_PIN = 6              # Serial Data Line GPIO pin
                            # Can be any GPIO pin on Pico 2 W
I2C_FREQUENCY = 400000       # I2C bus frequency in Hz (400kHz)
                            # Standard speeds: 100kHz (standard), 400kHz (fast)
                            # Some modules support 1MHz (fast-mode plus)

# Display Hardware Specifications
DISPLAY_WIDTH_PIXELS = 128   # Display width in pixels (standard for most SSD1306)
DISPLAY_HEIGHT_PIXELS = 64   # Display height in pixels (64 for larger, 32 for smaller)
DEFAULT_I2C_ADDRESS = 0x3C   # Default SSD1306 I2C address (7-bit)
                            # Alternative: 0x3D (check module documentation)

# === TEST CONFIGURATION ===
# Timing Constants for Test Pacing
TEST_DELAY_SHORT = 1.0       # Short delay between tests (1 second)
TEST_DELAY_MEDIUM = 2.0      # Medium delay for user observation (2 seconds)
TEST_DELAY_LONG = 3.0        # Long delay for complex tests (3 seconds)

# Animation Parameters
ANIMATION_FRAMES = 20        # Number of frames for animation tests
                            # Higher = longer animations, more CPU usage

# Display Control Test Parameters
CONTRAST_LEVELS = [50, 100, 150, 200, 255]  # Contrast levels to test
                                           # Range: 0 (dimmest) to 255 (brightest)


# ============================================================================
# HARDWARE INITIALIZATION FUNCTIONS
# ============================================================================

def initialize_display():
    """
    Initialize the SSD1306 OLED display with I2C interface.

    This function demonstrates proper hardware initialization patterns for
    embedded systems, including error handling, device detection, and
    fallback strategies.

    Initialization Process:
    1. Create I2C bus object with specified pins and frequency
    2. Scan I2C bus to detect connected devices
    3. Verify SSD1306 display is present at expected address
    4. Create display object and initialize hardware
    5. Handle errors gracefully with informative messages

    I2C Bus Scanning:
    The I2C scan operation sends a general call to all possible addresses
    (0x08 to 0x77) and records which devices acknowledge. This is useful for:
    - Verifying hardware connections
    - Discovering unknown device addresses
    - Troubleshooting communication issues

    Error Handling Strategy:
    - Try expected address first (0x3C)
    - Fall back to any detected device if available
    - Provide clear troubleshooting guidance on failure
    - Return None values to indicate failure (Python convention)

    Raspberry Pi Pico 2 W I2C Notes:
    - SoftI2C can use any GPIO pins (more flexible)
    - Hardware I2C (machine.I2C) is faster but uses specific pins
    - Pull-up resistors are usually built into OLED modules
    - 3.3V logic levels are compatible with most SSD1306 modules

    Returns:
        tuple: (oled_display, i2c_bus) if successful, (None, None) if failed
               oled_display (SSD1306_I2C): Initialized display object
               i2c_bus (SoftI2C): I2C bus object for potential reuse

    Raises:
        Exception: Catches all exceptions and converts to None return
                  (embedded systems should avoid unhandled exceptions)

    Example Usage:
        display, bus = initialize_display()
        if display is None:
            print("Hardware initialization failed!")
            return
        # Continue with display operations...
    """
    try:
        # === PHASE 1: I2C BUS INITIALIZATION ===
        print("Initializing I2C bus...")
        print(f"  SCL Pin: GPIO {I2C_SCL_PIN}")
        print(f"  SDA Pin: GPIO {I2C_SDA_PIN}")
        print(f"  Frequency: {I2C_FREQUENCY} Hz ({I2C_FREQUENCY//1000} kHz)")

        # Create software I2C bus object
        # SoftI2C allows any GPIO pins to be used for I2C communication
        i2c_bus = SoftI2C(
            scl=Pin(I2C_SCL_PIN),    # Serial Clock Line pin
            sda=Pin(I2C_SDA_PIN),    # Serial Data Line pin
            freq=I2C_FREQUENCY       # Communication frequency
        )

        # === PHASE 2: DEVICE DETECTION ===
        print("Scanning I2C bus for devices...")

        # Scan all possible I2C addresses (0x08 to 0x77)
        # This helps identify what devices are actually connected
        devices = i2c_bus.scan()

        if devices:
            print(f"Found I2C devices at addresses: {[hex(addr) for addr in devices]}")
        else:
            print("No I2C devices detected on the bus")
            print("Check wiring: SCL, SDA, VCC, GND connections")
            return None, None

        # === PHASE 3: SSD1306 ADDRESS VERIFICATION ===
        i2c_address = None

        if DEFAULT_I2C_ADDRESS in devices:
            # Expected address found - this is the normal case
            i2c_address = DEFAULT_I2C_ADDRESS
            print(f"[OK]SSD1306 found at expected address {hex(i2c_address)}")

        else:
            # Expected address not found - try to adapt
            print(f"⚠ Warning: SSD1306 not found at expected address {hex(DEFAULT_I2C_ADDRESS)}")

            if devices:
                # Use first available device as fallback
                i2c_address = devices[0]
                print(f"  Trying first available device at {hex(i2c_address)}")
                print("  (This might work if your display uses a different address)")
            else:
                print("  No alternative devices available")
                return None, None

        # === PHASE 4: DISPLAY OBJECT CREATION ===
        print("Initializing OLED display object...")
        print(f"  Display Size: {DISPLAY_WIDTH_PIXELS}x{DISPLAY_HEIGHT_PIXELS} pixels")
        print(f"  I2C Address: {hex(i2c_address)}")

        # Create SSD1306 display object
        # This will automatically initialize the display hardware
        oled_display = ssd1306.SSD1306_I2C(
            DISPLAY_WIDTH_PIXELS,    # Width in pixels
            DISPLAY_HEIGHT_PIXELS,   # Height in pixels
            i2c_bus,                # I2C bus object
            i2c_address             # Device I2C address
        )

        print("[OK]Display initialized successfully!")
        print(f"  Frame buffer size: {len(oled_display.frame_buffer)} bytes")
        print(f"  Memory pages: {oled_display.memory_pages}")

        return oled_display, i2c_bus

    except OSError as e:
        # I2C communication errors (most common)
        print(f"[✗]  I2C communication error: {e}")
        print("Troubleshooting tips:")
        print("  1. Check wiring connections (SCL, SDA, VCC, GND)")
        print("  2. Verify power supply (3.3V or 5V depending on module)")
        print("  3. Ensure pull-up resistors are present (usually built into module)")
        print("  4. Try different GPIO pins if wiring is suspected")
        return None, None

    except Exception as e:
        # Any other unexpected errors
        print(f"[✗]  Unexpected error during initialization: {e}")
        print("This might indicate a software or hardware compatibility issue")
        return None, None


def test_basic_text_display(oled_display):
    """Test basic text display functionality."""
    print("Testing basic text display...")

    oled_display.fill(0)  # Clear display

    # Test different text positions and content
    test_messages = [
        ("SSD1306 Test", 0, 0),
        ("Line 2: Numbers", 0, 10),
        ("12345 67890", 0, 20),
        ("Symbols: !@#$%", 0, 30),
        ("Bottom line", 0, 50)
    ]

    for message, x, y in test_messages:
        oled_display.text(message, x, y)

    oled_display.show()
    time.sleep(TEST_DELAY_MEDIUM)


def test_graphics_primitives(oled_display):
    """Test graphics drawing functions."""
    print("Testing graphics primitives...")

    # Test individual pixels
    oled_display.fill(0)
    oled_display.text("Pixel Test", 0, 0)

    # Draw pixel pattern
    for i in range(0, 128, 4):
        for j in range(15, 35, 4):
            oled_display.pixel(i, j, 1)

    oled_display.show()
    time.sleep(TEST_DELAY_SHORT)

    # Test lines
    oled_display.fill(0)
    oled_display.text("Line Test", 0, 0)

    # Draw various lines
    oled_display.line(0, 15, 127, 15, 1)    # Horizontal line
    oled_display.line(64, 15, 64, 63, 1)    # Vertical line
    oled_display.line(0, 63, 127, 15, 1)    # Diagonal line
    oled_display.line(127, 63, 0, 15, 1)    # Diagonal line

    oled_display.show()
    time.sleep(TEST_DELAY_SHORT)

    # Test rectangles
    oled_display.fill(0)
    oled_display.text("Rectangle Test", 0, 0)

    # Draw rectangles
    oled_display.rect(10, 15, 30, 20, 1)        # Outline rectangle
    oled_display.fill_rect(50, 15, 30, 20, 1)   # Filled rectangle
    oled_display.rect(90, 15, 30, 40, 1)        # Tall rectangle

    oled_display.show()
    time.sleep(TEST_DELAY_MEDIUM)


def test_display_control_functions(oled_display):
    """Test display control functions like contrast, invert, power."""
    print("Testing display control functions...")

    # Setup test display
    oled_display.fill(0)
    oled_display.text("Control Tests", 0, 0)
    oled_display.fill_rect(0, 15, 128, 20, 1)
    oled_display.text("White Block", 20, 40)
    oled_display.show()
    time.sleep(TEST_DELAY_SHORT)

    # Test contrast levels
    print("Testing contrast levels...")
    for contrast_level in CONTRAST_LEVELS:
        oled_display.fill(0)
        oled_display.text(f"Contrast: {contrast_level}", 0, 0)
        oled_display.fill_rect(0, 15, 128, 30, 1)
        oled_display.set_contrast(contrast_level)
        oled_display.show()
        time.sleep(TEST_DELAY_SHORT)

    # Reset to normal contrast
    oled_display.set_contrast(255)

    # Test display inversion
    print("Testing display inversion...")
    oled_display.fill(0)
    oled_display.text("Normal Display", 0, 0)
    oled_display.rect(0, 15, 128, 40, 1)
    oled_display.show()
    time.sleep(TEST_DELAY_SHORT)

    oled_display.set_invert_display(True)
    time.sleep(TEST_DELAY_SHORT)

    oled_display.set_invert_display(False)
    time.sleep(TEST_DELAY_SHORT)

    # Test power control
    print("Testing power control...")
    oled_display.fill(0)
    oled_display.text("Power Test", 0, 0)
    oled_display.text("Display OFF in 2s", 0, 15)
    oled_display.show()
    time.sleep(TEST_DELAY_MEDIUM)

    oled_display.turn_display_off()
    time.sleep(TEST_DELAY_MEDIUM)

    oled_display.turn_display_on()
    oled_display.fill(0)
    oled_display.text("Display ON", 0, 0)
    oled_display.show()
    time.sleep(TEST_DELAY_SHORT)


def test_animation_effects(oled_display):
    """Test animation and movement effects."""
    print("Testing animation effects...")

    # Bouncing ball animation
    print("Running bouncing ball animation...")
    ball_x, ball_y = 5, 20
    ball_dx, ball_dy = 2, 1
    ball_size = 3

    for frame in range(ANIMATION_FRAMES * 2):
        oled_display.fill(0)
        oled_display.text("Bouncing Ball", 0, 0)

        # Draw ball
        oled_display.fill_rect(ball_x, ball_y, ball_size, ball_size, 1)

        # Update ball position
        ball_x += ball_dx
        ball_y += ball_dy

        # Bounce off walls
        if ball_x <= 0 or ball_x >= DISPLAY_WIDTH_PIXELS - ball_size:
            ball_dx = -ball_dx
        if ball_y <= 12 or ball_y >= DISPLAY_HEIGHT_PIXELS - ball_size:
            ball_dy = -ball_dy

        oled_display.show()
        time.sleep(0.1)

    # Scrolling text effect
    print("Running scrolling text animation...")
    scroll_text = "    This is a scrolling text demonstration for SSD1306 OLED display!    "

    for offset in range(len(scroll_text) * 8):
        oled_display.fill(0)
        oled_display.text("Scrolling Text:", 0, 0)

        # Calculate text position for scrolling effect
        text_x = DISPLAY_WIDTH_PIXELS - (offset % (len(scroll_text) * 8))
        oled_display.text(scroll_text, text_x, 25)

        oled_display.show()
        time.sleep(0.05)

        # Break after reasonable number of iterations
        if offset > 200:
            break


def test_memory_and_performance(oled_display):
    """Test memory usage and performance."""
    print("Testing memory and performance...")

    # Memory usage test
    gc.collect()
    mem_free_start = gc.mem_free()
    print(f"Free memory at start: {mem_free_start} bytes")

    # Performance test - rapid updates
    print("Running performance test...")
    start_time = time.ticks_ms()

    for i in range(50):
        oled_display.fill(0)
        oled_display.text(f"Frame: {i:03d}", 0, 0)
        oled_display.text(f"Time: {time.ticks_ms()}", 0, 15)

        # Draw some graphics
        oled_display.rect(i % 100, 30, 20, 20, 1)
        oled_display.show()

    end_time = time.ticks_ms()
    duration = time.ticks_diff(end_time, start_time)
    fps = 50 * 1000 / duration if duration > 0 else 0

    # Display performance results
    oled_display.fill(0)
    oled_display.text("Performance Test", 0, 0)
    oled_display.text(f"50 frames in", 0, 15)
    oled_display.text(f"{duration}ms", 0, 25)
    oled_display.text(f"~{fps:.1f} FPS", 0, 35)
    oled_display.show()

    gc.collect()
    mem_free_end = gc.mem_free()
    print(f"Free memory at end: {mem_free_end} bytes")
    print(f"Memory used: {mem_free_start - mem_free_end} bytes")

    time.sleep(TEST_DELAY_LONG)


def test_backward_compatibility(oled_display):
    """Test backward compatibility methods."""
    print("Testing backward compatibility methods...")

    oled_display.fill(0)
    oled_display.text("Compatibility", 0, 0)
    oled_display.text("Test", 0, 15)

    # Test old method names
    oled_display.show()  # Should work (backward compatibility)
    time.sleep(TEST_DELAY_SHORT)

    oled_display.contrast(128)  # Old method name
    time.sleep(TEST_DELAY_SHORT)

    oled_display.invert(1)  # Old method name
    time.sleep(TEST_DELAY_SHORT)

    oled_display.invert(0)  # Back to normal
    time.sleep(TEST_DELAY_SHORT)

    print("Backward compatibility test completed!")


def run_comprehensive_tests(oled_display):
    """Run all comprehensive tests."""
    print("\n" + "="*50)
    print("STARTING COMPREHENSIVE SSD1306 OLED TESTS")
    print("="*50)

    try:
        # Run all test suites
        test_basic_text_display(oled_display)
        test_graphics_primitives(oled_display)
        test_display_control_functions(oled_display)
        test_animation_effects(oled_display)
        test_memory_and_performance(oled_display)
        test_backward_compatibility(oled_display)

        # Final success message
        oled_display.fill(0)
        oled_display.text("All Tests", 0, 0)
        oled_display.text("PASSED!", 0, 15)
        oled_display.text("SSD1306 Working", 0, 30)
        oled_display.text("Perfectly!", 0, 45)
        oled_display.show()

        print("\n" + "="*50)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("SSD1306 OLED Display is working perfectly!")
        print("="*50)

    except Exception as e:
        print(f"\nTest failed with error: {e}")
        oled_display.fill(0)
        oled_display.text("TEST FAILED!", 0, 0)
        oled_display.text(f"Error: {str(e)[:20]}", 0, 15)
        oled_display.show()


def main():
    """
    Main function to run comprehensive SSD1306 OLED display tests.
    """
    print("SSD1306 OLED Display Comprehensive Test Suite")
    print("Raspberry Pi Pico 2 W - MicroPython")
    print("-" * 50)

    # Initialize display
    oled_display, i2c_bus = initialize_display()

    if oled_display is None:
        print("Failed to initialize display. Please check:")
        print("1. Wiring connections (SCL->GPIO7, SDA->GPIO6)")
        print("2. Display power supply")
        print("3. I2C address (default 0x3C)")
        return

    # Run comprehensive tests
    try:
        run_comprehensive_tests(oled_display)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        oled_display.fill(0)
        oled_display.text("Test Stopped", 0, 0)
        oled_display.show()

    except Exception as e:
        print(f"\nUnexpected error: {e}")
        print("Please check hardware connections and try again")

    finally:
        print("\nTest session ended.")


# Run the main function when script is executed
if __name__ == "__main__":
    main()
