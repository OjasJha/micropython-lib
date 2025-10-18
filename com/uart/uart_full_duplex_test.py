"""
Universal UART Full Duplex Communication Module for Raspberry Pi Pico
=====================================================================

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
========
This module implements a robust UART communication system using Raspberry Pi Pico 2 
and MicroPython. It provides full duplex communication capabilities, meaning the Pico 
can simultaneously send and receive data through any UART-compatible device connected 
to GP0 and GP1.

COMPATIBILITY:
=============
This code is designed to work with ANY UART communication device, not just XBee modules.
It has been thoroughly tested and works excellently with:
- XBee radio modules (primary test platform)
- Other wireless communication modules
- Serial devices and sensors
- Any device that communicates via UART at 9600 baud

WHAT IS XBEE? (PRIMARY TEST PLATFORM)
====================================
XBee is a family of radio frequency (RF) modules that enable wireless communication between
devices. Think of it as a "wireless cable" that allows your Pico to communicate with other
XBee-equipped devices over distances of up to several kilometers (depending on the model).

This code was primarily developed and tested with XBee modules, but the UART communication
protocol is universal and works with any compatible device.

WHAT IS FULL DUPLEX?
===================
Full duplex means the system can send and receive data at the same time, similar to how
you can talk and listen simultaneously during a phone conversation. This is more efficient
than half-duplex systems that can only send OR receive at any given moment.

MICROPYTHON CONTEXT:
===================
MicroPython is a lightweight implementation of Python 3 designed for microcontrollers like
the Raspberry Pi Pico. It provides a subset of Python's standard library optimized for
embedded systems with limited memory and processing power.

RASPBERRY PI PICO 2 CONTEXT:
===========================
The Raspberry Pi Pico 2 is a microcontroller board based on the RP2040 chip. Unlike a
full computer, it's designed for embedded applications and has:
- 2MB of flash memory for storing programs
- 264KB of RAM for runtime operations
- Multiple GPIO pins for connecting sensors, actuators, and communication modules
- Built-in UART (Universal Asynchronous Receiver-Transmitter) for serial communication

HARDWARE ARCHITECTURE:
=====================
The Pico communicates with any UART device through UART0 (serial communication):
- UART0 is a hardware peripheral that handles serial communication automatically
- GP0 and GP1 are GPIO pins configured for UART0 TX (transmit) and RX (receive)
- The UART peripheral handles the low-level details of serial communication (start bits,
  stop bits, parity, etc.) so we don't have to implement this in software
- This works with ANY device that supports UART communication at 9600 baud

INTERRUPT-DRIVEN COMMUNICATION:
==============================
Instead of constantly checking for incoming data (polling), this code uses interrupts:
- When data arrives at the UART, the hardware automatically triggers an interrupt
- The interrupt handler processes the data immediately, ensuring no data is lost
- This is more efficient than polling and provides real-time responsiveness

WIRING DIAGRAM (UNIVERSAL UART CONNECTION):
==========================================
- Pico GP0 (UART0 TX) / Pin No. 1 → Device RX pin (Data In)
- Pico GP1 (UART0 RX) / Pin No. 2 ← Device TX pin (Data Out)
- Pico GP2 / Pin No. 4 → XBee RST pin (Reset control - optional but recommended)
- Pico GND ↔ Device GND (CRITICAL: Common ground reference)

Note: For XBee modules, connect to the S2C Adapter's TX/RX/RST pins as shown above.
The RST connection is optional but highly recommended to prevent race conditions.
For other UART devices, connect directly to their TX/RX pins (reset may not be needed).

POWER SUPPLY RECOMMENDATIONS:
============================
IMPORTANT: Do NOT power high-current devices through the Raspberry Pi Pico's 3.3V pin!

For XBee S2C modules specifically:
The XBee S2C module experiences significant current spikes during:
- Initial bootup and initialization
- Network association and scanning
- Data transmission bursts
- Radio frequency (RF) operations

These current spikes can:
- Overwhelm the Pico's USB power supply
- Cause voltage drops and brownouts
- Lead to unstable operation or communication failures
- Potentially damage the Pico's power regulation circuits

RECOMMENDED POWER SETUP:
=======================
For XBee modules:
1. Use an external power supply (3.3V to 5V) for the XBee S2C adapter
2. The S2C adapter has built-in voltage regulation and can handle input voltages from 3.3V to 5V
3. The adapter will provide clean, regulated 3.3V to the XBee module
4. This external supply can handle current spikes without affecting the Pico

For other UART devices:
- Check the device's power requirements
- Use appropriate external power supply if needed
- Ensure voltage levels are compatible (typically 3.3V or 5V)

CRITICAL GROUND CONNECTION:
==========================
- Connect the GND of the external power supply to the Pico's GND
- This creates a common ground reference voltage between both devices
- Without this connection, communication will fail due to voltage level mismatches
- The ground connection ensures proper signal integrity and prevents floating voltages

WIRING SUMMARY:
==============
- Pico GP0 → Device RX (data transmission from Pico)
- Pico GP1 ← Device TX (data reception to Pico)
- Pico GP2 → XBee RST (reset control - optional but recommended for XBee)
- Pico GND ↔ Device GND (common ground reference)
- External power supply → Device VCC (if required)
- External supply GND → Pico GND (shared ground)

USAGE EXAMPLE:
=============
For XBee modules:
1. Connect the XBee module as described above
2. Power both the Pico and XBee with external supplies
3. Run this script on the Pico
4. The Pico will continuously send test messages
5. If you short GP0 to GP1 (loopback test), you'll see received messages
6. For real communication, connect another XBee module to another device

For other UART devices:
1. Connect your UART device to GP0 and GP1 as described above
2. Ensure proper power supply for your device
3. Run this script on the Pico
4. The Pico will continuously send test messages
5. Your device should receive and can respond to these messages
6. Any data sent by your device will be displayed as "XBee RX Line:" messages

Author: Ojas Jha
Version: 2.0
Date: October 18, 2025
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# Time and timing functions for MicroPython
# utime provides time-related functions similar to Python's time module
# but optimized for microcontrollers with limited resources
import utime

# Import the UART handler class
# This module provides a comprehensive UART communication class with initialization, LED setup, and interrupt-driven communication capabilities
from uart_handler import UartHandler

# Hardware abstraction layer for MicroPython
# machine module provides direct access to hardware peripherals on the Pico
from machine import UART, Pin
# - UART: Universal Asynchronous Receiver-Transmitter for serial communication
# - Pin: GPIO pin control for digital input/output operations

# Operating system interface for MicroPython
# uos provides operating system-like functionality for file operations and system calls
# We use it to free UART0 from the REPL (Read-Eval-Print Loop) console
import uos

# =============================================================================
# CONFIGURATION CONSTANTS - HARDWARE AND COMMUNICATION SETTINGS
# =============================================================================

# XBee Reset Pin Configuration
# ============================
# GPIO pin used to perform hardware reset on the XBee module
# This ensures the XBee boots up properly after the Pico is ready
XBEE_RESET_PIN = 2  # GP2 - connected to XBee RST (Reset) pin
                     # Active-low reset: Pull to GND to reset, release (high) to run

# XBee Initialization Timing
# ==========================
# Time to wait after hardware reset for XBee to complete its boot process
# The XBee needs time to:
# - Boot its internal microcontroller
# - Initialize its radio hardware
# - Associate with its network (if configured)
XBEE_INIT_DELAY_SECONDS = 5  # 5 seconds - sufficient for most XBee modules

# Data Buffer and Timing Settings
# ===============================
# These constants control how the system handles incoming data and transmission timing
TRANSMISSION_DELAY_SECONDS = 4  # Delay between sending messages (in seconds)
                                 # Increased from 2 to 4 seconds for USB buffer stability
                                 # Prevents overwhelming the UART device with data
                                 # Allows USB serial buffer time to clear and prevents hanging
                                 # Also allows time to see received messages


# =============================================================================
# UART RECEIVE CALLBACK FUNCTIONS
# =============================================================================

def on_command_received_via_uart(character_byte, uart_handler):
    """
    Handle single character commands received via UART.

    This function is called when a single character is received via UART.
    It processes commands supported by the pico-updated.py system and provides
    appropriate responses via USB and UART.

    Supported Commands (from pico-updated.py):
    - 'a': MPU6050 6-axis motion sensor data capture
    - 'b': System reset
    - 's': SD card data logging (housekeeping + 6-axis + GPS data)
    - 'g': GPS data capture
    - 'c': Camera image capture and save to SD card
    - Others: Unknown command handling

    Args:
        character_byte (int): The received byte value (0-255)
        uart_handler (UartHandler): UART handler instance for sending responses (required)
    """
    # Ensure byte value is in valid range (0-255)
    byte_value = character_byte & 0xFF

    if 32 <= byte_value <= 126:  # Printable ASCII range
        command = chr(byte_value)

        # Print command received via USB (for monitoring)
        print(f"UART Command Received: '{command}' (ASCII: {byte_value})")

        # Process the command based on pico-updated.py supported commands
        if command == "a":
            print("Processing MPU6050 6-axis motion sensor data capture...")
            usb_response = "Command 'a' received: Starting MPU6050 6-axis motion sensor data capture"
            uart_response = "ACK: MPU6050 capture initiated"

        elif command == "b":
            print("Processing system reset...")
            usb_response = "Command 'b' received: Executing system reset"
            uart_response = "ACK: System reset initiated"

        elif command == "c":
            print("Processing camera image capture and SD card save...")
            usb_response = "Command 'c' received: Starting camera image capture and saving to SD card"
            uart_response = "ACK: Camera capture initiated"

        elif command == "s":
            print("Processing SD card data logging...")
            usb_response = "Command 's' received: Starting SD card data logging (HK + 6-AXIS + GPS)"
            uart_response = "ACK: SD card logging initiated"

        elif command == "g":
            print("Processing GPS data capture...")
            usb_response = "Command 'g' received: Starting GPS data capture"
            uart_response = "ACK: GPS capture initiated"

        else:
            print(f"Unknown command '{command}' received")
            usb_response = f"Unknown command '{command}' - no action taken"
            uart_response = f"NAK: Unknown command '{command}'"

        # Send response via USB (for monitoring)
        print(f"USB Response: {usb_response}")

        # Send dummy response via UART back to the sender
        try:
            # Send the response via UART with proper formatting
            uart_response_formatted = f"{uart_response}\r\n"
            bytes_sent = uart_handler.write(uart_response_formatted.encode())
            print(f"UART Response sent: '{uart_response}' ({bytes_sent} bytes)")
        except Exception as e:
            print(f"Failed to send UART response: {e}")

    else:
        # Display as hexadecimal value with custom formatting
        print(f"UART Command Handler: Control character (0x{byte_value:02X})")
        print("USB Response: Control character received - no action taken")

        # Send response for control character via UART
        try:
            uart_response = "NAK: Control character"
            uart_response_formatted = f"{uart_response}\r\n"
            bytes_sent = uart_handler.write(uart_response_formatted.encode())
            print(f"UART Response sent: '{uart_response}' ({bytes_sent} bytes)")
        except Exception as e:
            print(f"Failed to send UART response: {e}")

def on_message_received_via_uart(message_string, uart_handler):
    """
    Handle complete line messages received via UART.

    This function is called when a complete line (terminated by CR/LF) is
    received via UART. It provides a custom way to handle messages instead
    of the default print behavior and sends an acknowledgment back via UART.

    Args:
        message_string (str): The received message string
        uart_handler (UartHandler): UART handler instance for sending responses (required)
    """
    # Custom message processing with enhanced formatting
    print(f"Custom UART Message Handler: '{message_string}' (Length: {len(message_string)} chars)")

    # Send acknowledgment back via UART
    try:
        # Send acknowledgment message via UART
        ack_message = f"ACK: Line processed - {len(message_string)} chars received\r\n"
        bytes_sent = uart_handler.write(ack_message.encode())
        print(f"UART Acknowledgment sent: 'Line processed - {len(message_string)} chars received' ({bytes_sent} bytes)")
    except Exception as e:
        print(f"Failed to send UART acknowledgment: {e}")

# =============================================================================
# DATA TRANSMISSION AND MAIN APPLICATION
# =============================================================================
def main():
    """
    Main entry point for UART communication application.

    This function serves as the application's entry point and provides
    proper error handling and graceful shutdown capabilities.

    APPLICATION LIFECYCLE:
    =====================
    1. Initialize the communication system (hardware setup)
    2. Start the main communication loop (integrated directly in main)
    3. Handle user interruption (Ctrl+C)
    4. Handle unexpected errors gracefully

    ERROR HANDLING:
    ==============
    - KeyboardInterrupt: Graceful shutdown when user presses Ctrl+C
    - General Exception: Catches and displays any unexpected errors
    - System continues running despite individual message failures

    SHUTDOWN BEHAVIOR:
    =================
    When the application is stopped (Ctrl+C):
    - Displays a friendly shutdown message
    - Allows the system to clean up resources
    - Returns control to the MicroPython REPL

    HARDWARE INITIALIZATION:
    =======================
    Hardware initialization happens in this order:
    1. UART and LED setup (done at module import time)
    2. UART interrupt configuration (done in main() for better error handling)
    3. Communication loop starts after all hardware is ready

    COMMUNICATION LOOP:
    ==================
    The main communication loop is integrated directly into this function:
    - Continuously sends test messages via UART
    - Displays transmission information for monitoring
    - Receives and displays incoming data via interrupt handler
    - Provides visual feedback and status information
    - Demonstrates true full duplex communication

    Initializes the system and runs the communication loop.
    """
    try:
        # Display initialization message
        print("Initializing UART Communication System...")

        # =========================================================
        # HARDWARE RESET SEQUENCE FOR XBEE MODULE
        # =========================================================
        # This solves the race condition where the Pico boots faster than the XBee.
        # By performing a hardware reset, we ensure the XBee starts its boot process
        # only after the Pico is ready and in control.
        #
        # Reset Sequence Explanation:
        # ---------------------------
        # The RST pin on XBee modules is active-low, which means:
        # - HIGH (1) = Normal operation (inactive reset state)
        # - LOW (0) = Reset active (XBee held in reset)
        #
        # The sequence performs: High -> Low -> High
        # 1. Start HIGH: Ensure we're in inactive state
        # 2. Pull LOW: Trigger the reset (XBee stops/resets)
        # 3. Release HIGH: XBee begins its boot sequence
        print(f"Performing hardware reset on XBee via GP{XBEE_RESET_PIN}...")
        reset_pin = Pin(XBEE_RESET_PIN, Pin.OUT)

        # Step 1: Start high (inactive state)
        reset_pin.value(1)
        utime.sleep(0.1)  # Brief delay to ensure pin is stable

        # Step 2: Pull low to trigger reset
        reset_pin.value(0)
        utime.sleep(0.1)  # Hold reset for 100ms (well above typical 1-10ms requirement)

        # Step 3: Release reset - XBee starts booting
        reset_pin.value(1)

        # Now wait for the XBee to finish its boot process after the reset
        # This delay accounts for:
        # - Internal microcontroller boot
        # - Radio hardware initialization
        # - Network association (if configured)
        print(f"Reset complete. Waiting for XBee to initialize ({XBEE_INIT_DELAY_SECONDS} second delay)...")
        utime.sleep(XBEE_INIT_DELAY_SECONDS)
        print("Initialization delay complete. XBee should now be ready for communication.")
        # =========================================================

        # Initialize UART handler
        # =======================
        # Create UART handler instance which automatically initializes all hardware
        # components including UART interface, status LED, and interrupt configuration
        uart_handler = UartHandler(
            uart_interface_id=0,      # UART0 - connected to GP0 (TX) and GP1 (RX)
            uart_tx_pin=0,           # GP0 - transmit pin (Pico -> Device)
            uart_rx_pin=1,           # GP1 - receive pin (Device -> Pico)
            uart_baudrate=9600,      # 9600 bps - standard baudrate for most UART devices
            max_line_buffer_size=256, # 256 bytes - prevents memory overflow
            on_command_received=on_command_received_via_uart,  # Command handler with UART instance
            on_message_received=on_message_received_via_uart   # Custom message handler
        )
        print("UART Handler initialized successfully.")

        # Display startup information and instructions
        print("=" * 70)
        print("UART Full Duplex Communication Started")
        print("Supported Commands (from pico-updated.py):")
        print("  'a' - MPU6050 6-axis motion sensor data capture")
        print("  'b' - System reset")
        print("  'c' - Camera image capture and save to SD card")
        print("  's' - SD card data logging (HK + 6-AXIS + GPS)")
        print("  'g' - GPS data capture")

        print("Send any of these commands via UART device")
        print("=" * 70)

        # Initialize communication variables
        message_counter = 0  # Track number of messages sent
        test_message = "UART Test Message"  # Test message content

        # Main communication loop - runs indefinitely
        while True:
            # Send message and get transmission details
            # Get current timestamp for message
            # utime.ticks_ms() returns milliseconds since system boot
            current_timestamp = utime.ticks_ms()

            # ========================================
            # SENDING MESSAGE TO UART DEVICE
            # ========================================
            # Format message with timestamp and counter
            # The \r\n ensures proper line termination for the receiving end
            formatted_message = f"[{current_timestamp}ms] {test_message} #{message_counter}\r\n"

            # Send message via UART
            # encode() converts the string to bytes using UTF-8 encoding
            # write() sends the bytes through the UART hardware peripheral
            bytes_sent = uart_handler.write(formatted_message.encode())
            timestamp = current_timestamp

            # ========================================
            # SENDING TRANSMISSION MESSAGE VIA USB PORT FOR MONITORING
            # ========================================
            # Display transmission information for monitoring
            # Shows what was sent, when, and how many bytes were transmitted
            # Format the message part (without line terminators for display)
            formatted_message = f"[{timestamp}ms] {test_message} #{message_counter}"

            # Display comprehensive transmission information
            print(f"{formatted_message} | Bytes Transmitted: {bytes_sent} | Message Counter: {message_counter}")

            # Increment counter for next message
            message_counter += 1

            # Wait before next transmission
            # This delay gives the USB buffer time to clear and prevents overwhelming the console.
            # The print() function can block if the USB buffer fills up faster than Thonny can read it,
            # which would prevent the script from processing Ctrl+C and lead to timeout errors.
            # This delay prevents overwhelming the UART device and allows time
            # to see received messages (if any) during the pause
            utime.sleep(TRANSMISSION_DELAY_SECONDS)

    except KeyboardInterrupt:
        # Handle user interruption (Ctrl+C)
        # This provides a clean way to stop the application
        print("\\nUART Communication stopped by user.")
        print("System shutdown complete.")

    except Exception as e:
        # Handle unexpected errors
        # This prevents the system from crashing silently
        print(f"Error in UART communication: {e}")
        print("Please check your hardware connections and try again.")

# APPLICATION STARTUP
# ==================
# This section runs when the script is executed directly
# It ensures the main function is called to start the application
if __name__ == "__main__":
    # Start the UART communication application
    main()
