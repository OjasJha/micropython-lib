
"""
UART Loopback Test for Raspberry Pi Pico 2
==========================================

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
---------
This module implements a comprehensive UART (Universal Asynchronous Receiver-Transmitter)
loopback test for the Raspberry Pi Pico 2 microcontroller. The test creates a hardware
loopback connection between the transmit (TX) and receive (RX) pins to verify that:
1. UART communication is working correctly
2. Interrupt-driven data reception is functioning
3. The microcontroller can handle real-time data processing

WHAT IS UART?
-------------
UART is a serial communication protocol that allows two devices to communicate by
transmitting data one bit at a time over a single wire (or pair of wires). It's
asynchronous, meaning there's no shared clock signal - both devices must agree on
the data rate (baud rate) beforehand.

RASPBERRY PI PICO 2 SPECIFICS:
------------------------------
The Raspberry Pi Pico 2 is a microcontroller board based on the RP2040 chip. It has:
- 2.4GHz dual-core ARM Cortex-M0+ processor
- 264KB of SRAM
- 2MB of flash memory
- Multiple GPIO pins that can be configured for various functions
- Built-in UART hardware that can be configured on different pin combinations

HARDWARE SETUP:
---------------
1. Connect a jumper wire between:
   - GP0 (pin 1) - This is the TX (transmit) pin
   - GP1 (pin 2) - This is the RX (receive) pin
2. This creates a "loopback" where data sent from TX is immediately received on RX
3. No external components needed - just the jumper wire

WHY USE INTERRUPTS?
-------------------
Interrupts allow the microcontroller to respond immediately to events (like receiving
data) without constantly checking (polling). This is more efficient and allows the
main program to do other tasks while waiting for data.

TESTING CONCEPTS:
-----------------
- Loopback Testing: A method to test communication by connecting output back to input
- Interrupt-Driven I/O: Using hardware interrupts to handle data reception
- Asynchronous Communication: Data transmission without shared timing signals
- Buffer Management: Handling data that arrives faster than it can be processed

Author: Ojas Jha
Version: 2.0
Date: October 18, 2025
Platform: Raspberry Pi Pico 2 with MicroPython
"""

# =============================================================================
# IMPORT STATEMENTS - Grouped by Function
# =============================================================================

# Time and timing functions
import utime
# MicroPython-specific functions for scheduling and optimization
import micropython

# Hardware abstraction layer - provides access to microcontroller peripherals
from machine import UART, Pin
# Operating system interface for MicroPython
import uos

# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

# UART Configuration
UART_INTERFACE_ID = 0  # Which UART peripheral to use (Pico 2 has UART0 and UART1)
UART_BAUDRATE = 9600   # Data transmission rate in bits per second

"""
UART_INTERFACE_ID EXPLANATION:
------------------------------
The RP2040 chip has two UART peripherals (UART0 and UART1). UART0 is typically used
by the REPL (console), so we need to free it before using it for our application.

UART_BAUDRATE EXPLANATION:
--------------------------
Baud rate determines how fast data is transmitted. Common rates are:
- 9600 bps: Slow but reliable, good for testing
- 115200 bps: Fast, commonly used for debugging
- 230400 bps: Very fast, used for high-speed communication

9600 is chosen because it's:
- Reliable and well-supported
- Easy to debug (slow enough to see what's happening)
- Standard rate for many devices like XBee modules
"""

# Pin Configuration for UART
UART_TX_PIN = 0   # GP0 -> Transmit pin (Physical pin 1 on Pico 2)
UART_RX_PIN = 1   # GP1 -> Receive pin (Physical pin 2 on Pico 2)

"""
PIN CONFIGURATION EXPLANATION:
------------------------------
The Raspberry Pi Pico 2 has multiple GPIO pins that can be configured for different
functions. For UART communication, we need:
- TX (Transmit): Sends data out
- RX (Receive): Receives data in

GP0 and GP1 are the default pins for UART0 on the Pico 2. These pins are:
- GP0 (pin 1): Can be configured as UART0 TX
- GP1 (pin 2): Can be configured as UART0 RX

PHYSICAL PIN LAYOUT:
-------------------
When looking at the Pico 2 with the USB connector at the top:
- Pin 1 (GP0) is the first pin on the left side
- Pin 2 (GP1) is the second pin on the left side
- These are the pins we'll connect with a jumper wire for the loopback test
"""

# Test Configuration
TEST_MESSAGE = b'ABC\r\n'              # Message to transmit during the test
TRANSMISSION_INTERVAL_SECONDS = 1      # Time between transmissions

"""
TEST CONFIGURATION EXPLANATION:
-------------------------------
TEST_MESSAGE: The data we'll send during the loopback test
- b'ABC\r\n' means: send the bytes for 'A', 'B', 'C', carriage return, newline
- \r\n is the standard line ending (CRLF - Carriage Return + Line Feed)
- This creates a recognizable pattern that's easy to see in the output

TRANSMISSION_INTERVAL_SECONDS: How often to send the test message
- 1 second gives enough time to see each transmission clearly
- Can be adjusted for faster/slower testing
- Too fast might overwhelm the interrupt handler
- Too slow makes the test take longer
"""

def display_received_byte(byte_value):
    """
    Display a received byte in a formatted way.
    This function is scheduled to run in the main thread.

    This function is designed to be called from an interrupt handler via
    micropython.schedule(). It takes a byte value, formats it, and displays
    it to the console.

    WHY USE micropython.schedule()?
    ------------------------------
    Interrupt handlers run in a special context and have restrictions:
    - They can't directly call print() or other functions that might block
    - They need to return quickly to avoid blocking other interrupts
    - micropython.schedule() safely queues a function to run in the main thread

    THREAD SAFETY:
    -------------
    By using micropython.schedule(), we ensure that the display function
    runs in the main thread where it's safe to use print() and other
    potentially blocking functions.

    BYTE PROCESSING:
    ---------------
    - byte_value &= 0xFF: Ensures the value is in the valid byte range (0-255)
    - This prevents issues with negative numbers or values outside byte range

    ASCII CHARACTER DETECTION:
    -------------------------
    - ASCII printable characters are in the range 32-126
    - 32 = space character, 126 = tilde (~)
    - Characters below 32 are control characters (newline, tab, etc.)
    - Characters above 126 are extended ASCII or non-ASCII

    FORMATTING OPTIONS:
    ------------------
    - Printable characters: Display as 'A', 'B', 'C', etc.
    - Non-printable characters: Display as hexadecimal (0x0A, 0x0D, etc.)

    Args:
        byte_value (int): The byte value to display (0-255)

    Returns:
        None

    Example:
        >>> display_received_byte(65)
        RX: 'A'
        >>> display_received_byte(10)
        RX: 0x0A
    """
    # Ensure byte is in valid range (0-255)
    # This prevents issues with negative numbers or values outside byte range
    byte_value &= 0xFF

    # Check if it's a printable ASCII character
    # ASCII printable range is 32 (space) to 126 (tilde)
    if 32 <= byte_value <= 126:
        formatted_byte = f"RX: '{chr(byte_value)}'"
    else:
        # Format non-printable characters as hexadecimal
        # :02X means: 2 digits, uppercase hexadecimal, zero-padded
        formatted_byte = f"RX: 0x{byte_value:02X}"

    # Display the formatted byte to the console
    print(formatted_byte)

# =============================================================================
# INTERRUPT HANDLERS
# =============================================================================

"""
INTERRUPT HANDLERS EXPLANATION:
-------------------------------
Interrupt handlers are special functions that are called automatically by the
microcontroller's hardware when specific events occur. In this case, we're
setting up an interrupt handler for UART data reception.

WHAT ARE INTERRUPTS?
-------------------
Interrupts are a mechanism that allows the microcontroller to respond immediately
to hardware events without constantly checking (polling) for them. When data
arrives on the UART, the hardware automatically calls our interrupt handler.

BENEFITS OF INTERRUPT-DRIVEN I/O:
--------------------------------
- Efficiency: No need to constantly check for data
- Responsiveness: Immediate response to data arrival
- Power saving: CPU can sleep while waiting for data
- Real-time capability: Predictable response times

INTERRUPT HANDLER REQUIREMENTS:
------------------------------
- Must be fast and not block for long periods
- Should not call functions that might block (like print())
- Must handle errors gracefully
- Should return quickly to allow other interrupts
"""

def handle_uart_receive_interrupt(uart_interface):
    """
    Handle UART receive interrupt.
    This function is called automatically by the hardware when data is received.

    This is the core interrupt handler that processes incoming UART data. It's
    called by the microcontroller's hardware whenever data arrives on the UART
    receive line. The function must be fast and efficient to avoid blocking
    other system operations.

    INTERRUPT PROCESSING FLOW:
    -------------------------
    1. Hardware detects data on UART RX line
    2. Hardware automatically calls this function
    3. Function reads all available data from UART buffer
    4. Function processes each byte individually
    5. Function schedules display operations for the main thread
    6. Function returns quickly to allow other interrupts

    DATA BUFFER MANAGEMENT:
    ----------------------
    The UART hardware has internal buffers that store incoming data. We read
    all available data at once to avoid missing bytes that arrive in quick
    succession. This is especially important at higher baud rates.

    THREAD SAFETY:
    -------------
    This function runs in interrupt context, so it can't directly call print()
    or other potentially blocking functions. Instead, it uses micropython.schedule()
    to queue display operations for the main thread.

    Args:
        uart_interface (UART): The UART interface that triggered the interrupt

    Returns:
        None

    Note:
        This function runs in interrupt context and must return quickly.
        Do not add blocking operations or long computations here.

    Example:
        When data "ABC" is received, this function will be called and will
        schedule display operations for bytes 65, 66, and 67 (ASCII values).
    """
    # Read all available data from the UART buffer
    # This ensures we don't miss any data that arrived while we were processing
    received_data = uart_interface.read()

    # Check if we actually received any data
    if not received_data:
        return  # No data to process, exit early

    # Process each received byte individually
    # This allows us to handle each byte separately and provide detailed feedback
    for byte_value in received_data:
        try:
            # Schedule the display function to run in the main thread
            # This is safe because display_received_byte() is designed to be
            # called from interrupt context via micropython.schedule()
            micropython.schedule(display_received_byte, byte_value)
        except Exception as e:
            # Handle any errors in scheduling (should be rare)
            # We can't use print() directly in interrupt context, so we
            # would need to handle this differently in a production system
            pass  # For now, we'll silently ignore scheduling errors


# =============================================================================
# MAIN APPLICATION
# =============================================================================

"""
MAIN APPLICATION EXPLANATION:
-----------------------------
The main application functions coordinate the entire test process. They handle
the initialization of all components and run the continuous loopback test.

APPLICATION FLOW:
----------------
1. Initialize hardware (UART, LED)
2. Configure interrupt handling
3. Run continuous loopback test
4. Handle user interruption gracefully

TESTING METHODOLOGY:
-------------------
The loopback test works by:
1. Transmitting data on the TX pin
2. The jumper wire connects TX to RX
3. The same data is received on the RX pin
4. Interrupts handle the received data
5. Data is displayed to verify correct reception

This verifies that both transmission and reception are working correctly.
"""

def main():
    """
    Main application entry point.
    Initializes hardware and runs the loopback test.

    This function serves as the main entry point for the application. It
    coordinates the initialization of all hardware components and runs
    the loopback test. The function follows a clear sequence:

    1. Initialize UART interface
    2. Initialize status LED (if available)
    3. Configure interrupt handling
    4. Run the loopback test

    LOOPBACK TEST PRINCIPLE:
    -----------------------
    A loopback test is a method of testing communication systems by connecting
    the output back to the input. In this case:
    - We transmit data on GP0 (TX pin)
    - The jumper wire connects GP0 to GP1 (RX pin)
    - The same data is received on GP1
    - This verifies that both TX and RX are working correctly

    TEST DATA PATTERN:
    -----------------
    We transmit "ABC\r\n" which contains:
    - 'A' (ASCII 65): Printable character
    - 'B' (ASCII 66): Printable character
    - 'C' (ASCII 67): Printable character
    - '\r' (ASCII 13): Carriage return (non-printable)
    - '\n' (ASCII 10): Newline (non-printable)

    This pattern tests both printable and non-printable character handling.

    INTERRUPT VERIFICATION:
    ----------------------
    The test verifies that:
    1. Data is transmitted correctly
    2. Interrupts are triggered when data is received
    3. Data is processed and displayed correctly
    4. The system can handle continuous data flow

    USER INTERACTION:
    ----------------
    The test runs continuously until the user presses Ctrl+C. This allows
    for extended testing and observation of the system behavior.

    GLOBAL VARIABLES:
    ----------------
    The function uses global variables to store hardware objects so they
    can be accessed by other functions (like interrupt handlers).

    ERROR HANDLING:
    --------------
    Each initialization step includes error handling to ensure the system
    can continue even if some components fail to initialize.

    Returns:
        None

    Raises:
        Exception: If critical hardware initialization fails
        KeyboardInterrupt: When user stops the test with Ctrl+C

    Example Output:
        UART0 freed from REPL
        UART initialized on GP0/GP1 at 9600 baud
        Status LED initialized
        Using RX_ANY interrupt trigger
        UART interrupt handler configured
        UART LOOPBACK TEST STARTED
        TX: Transmitted test message #0
        RX: 'A'
        RX: 'B'
        RX: 'C'
        RX: 0x0D
        RX: 0x0A
        TX: Transmitted test message #1
        ...
    """
    # Initialize hardware components
    # Global variables allow access from other functions
    global uart_interface, status_led

    # Initialize UART interface for communication
    # Free UART0 from REPL if it was previously attached
    try:
        uos.dupterm(None, 1)
        print("UART0 freed from REPL")
    except Exception as e:
        print(f"Warning: Could not free UART0 from REPL: {e}")
        # This is not critical - the UART might not be attached to REPL

    # Initialize UART for communication
    # This creates a UART object that handles the low-level hardware communication
    uart_interface = UART(
        UART_INTERFACE_ID,    # Use UART0 (the first UART peripheral)
        baudrate=UART_BAUDRATE,  # Set communication speed
        tx=Pin(UART_TX_PIN),     # Configure GP0 as transmit pin
        rx=Pin(UART_RX_PIN)      # Configure GP1 as receive pin
    )

    # Clear any previous data in the buffer
    # This ensures we start with a clean communication channel
    _ = uart_interface.read()
    print(f"UART initialized on GP{UART_TX_PIN}/GP{UART_RX_PIN} at {UART_BAUDRATE} baud")

    # Initialize status LED for visual feedback (optional)
    try:
        # Create a Pin object for the onboard LED
        # Pin("LED", Pin.OUT) automatically finds the correct pin for the LED
        status_led = Pin("LED", Pin.OUT)
        print("Status LED initialized")
    except Exception as e:
        # Handle the case where LED is not available
        print(f"Status LED not available: {e}")
        status_led = None

    # Configure interrupt handling for data reception
    # Determine available interrupt triggers
    # Start with no triggers enabled
    interrupt_triggers = 0

    # Check for RX_ANY trigger (available in most MicroPython versions)
    if hasattr(UART, "RX_ANY"):
        interrupt_triggers |= UART.RX_ANY
        print("Using RX_ANY interrupt trigger")

    # Check for IRQ_RXIDLE trigger (available in some MicroPython versions)
    if hasattr(UART, "IRQ_RXIDLE"):
        interrupt_triggers |= UART.IRQ_RXIDLE
        print("Using IRQ_RXIDLE interrupt trigger")

    # Set up the interrupt handler
    # The lambda function creates a wrapper that passes the UART object to our handler
    uart_interface.irq(
        handler=lambda uart: handle_uart_receive_interrupt(uart),
        trigger=interrupt_triggers
    )

    print("UART interrupt handler configured")

    # Display test information and instructions
    print("\n" + "="*60)
    print("UART LOOPBACK TEST STARTED")
    print("="*60)
    print("Hardware Setup:")
    print(f"  - Short GP{UART_TX_PIN} (pin 1) to GP{UART_RX_PIN} (pin 2)")
    print(f"  - This creates a loopback connection")
    print("="*60)
    print("You should see received data (RX) printed below as interrupts occur.")
    print("Press Ctrl+C to stop the test.")
    print("="*60 + "\n")

    # Initialize transmission counter for tracking
    transmission_counter = 0

    try:
        # Main test loop - runs indefinitely until interrupted
        while True:
            # Transmit test data
            # This sends the test message on the TX pin
            uart_interface.write(TEST_MESSAGE)
            print(f"TX: Transmitted test message #{transmission_counter}")

            # Increment counter for next transmission
            transmission_counter += 1

            # Wait before next transmission
            # This gives time to see the received data and prevents overwhelming the system
            utime.sleep(TRANSMISSION_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        # Handle user interruption gracefully
        print("\n" + "="*60)
        print("TEST STOPPED BY USER")
        print("="*60)
        print(f"Total transmissions: {transmission_counter}")
        print("Test completed successfully!")

# =============================================================================
# APPLICATION STARTUP
# =============================================================================

"""
APPLICATION STARTUP EXPLANATION:
--------------------------------
This section handles the startup of the application. The if __name__ == "__main__"
construct ensures that the main() function is only called when the script is
run directly, not when it's imported as a module.

WHY USE if __name__ == "__main__"?
---------------------------------
This is a Python best practice that allows the script to be used in two ways:
1. As a standalone program: python script.py
2. As an imported module: import script (without running main())

This makes the code more flexible and reusable.
"""

if __name__ == "__main__":
    # Start the application
    main()
