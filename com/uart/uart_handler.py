"""
UART Handler Class for Raspberry Pi Pico
=======================================

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
This module provides a comprehensive UART communication class for the
Raspberry Pi Pico using MicroPython. It encapsulates UART initialization,
interrupt-driven data reception, LED status indication, and real-time
communication handling in an object-oriented design.

WHAT IS AN IRQ HANDLER?
=======================
An IRQ (Interrupt Request) handler is a special function that gets called
automatically by the microcontroller's hardware when specific events occur.
In this case, we're setting up an IRQ handler for UART data reception.

BENEFITS OF INTERRUPT-DRIVEN COMMUNICATION:
===========================================
- Efficiency: No need to constantly check for data (polling)
- Responsiveness: Immediate response to data arrival
- Power saving: CPU can sleep while waiting for data
- Real-time capability: Predictable response times
- No data loss: Hardware automatically triggers processing

INTERRUPT CONTEXT LIMITATIONS:
==============================
Interrupt handlers run in a special context with restrictions:
- No printing (print() statements)
- No memory allocation (creating new objects)
- No complex operations that might take too long
- Must return quickly to allow other interrupts

To safely perform complex operations, we use micropython.schedule() to move
operations out of interrupt context into a scheduled context where these
operations are allowed.

Author: Ojas Jha
Version: 2.0
Date: October 18, 2025
"""

# =============================================================================
# IMPORT STATEMENTS - MICROPYTHON LIBRARIES
# =============================================================================

# MicroPython-specific utilities for interrupt handling and scheduling
# micropython.schedule() allows safe execution of functions from interrupt context
import micropython

# Hardware abstraction layer for MicroPython
# machine module provides direct access to hardware peripherals on the Pico
from machine import UART, Pin

# Operating system interface for MicroPython
# uos provides operating system-like functionality for file operations and system calls
# We use it to free UART0 from the REPL (Read-Eval-Print Loop) console
import uos

# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================
# Configuration constants have been moved to the UartHandler class constructor
# with default values. This provides better flexibility and encapsulation.

# =============================================================================
# UART HANDLER CLASS
# =============================================================================

class UartHandler:
    """
    UART Handler Class for Raspberry Pi Pico

    This class encapsulates all UART communication functionality including
    hardware initialization, interrupt handling, data processing, and status
    indication. It provides a clean object-oriented interface for UART
    communication with any compatible device.

    FEATURES:
    =========
    - Automatic UART hardware initialization
    - Built-in status LED for visual feedback (controlled only in interrupt context)
    - Interrupt-driven data reception
    - Real-time data processing
    - Utility functions for buffer management
    - Scheduled handlers/callbacks are unaffected by LED control

    USAGE:
    ======
    # Create UART handler instance with default settings
    uart = UartHandler()

    # Create UART handler with custom settings and callbacks
    uart = UartHandler(
        uart_interface_id=0,      # UART0
        uart_tx_pin=0,           # GP0
        uart_rx_pin=1,           # GP1
        uart_baudrate=115200,    # 115200 bps
        max_line_buffer_size=512, # 512 byte buffer
        on_command_received=my_command_handler,    # Custom command handler (character_byte, uart_handler)
        on_message_received=my_message_handler     # Custom message handler (message_string, uart_handler)
    )

    # The UART is now ready for communication
    # Send data: uart.write(data)
    # Receive data: handled automatically via interrupts and callbacks
    """

    def __init__(self, uart_interface_id=0, uart_tx_pin=0, uart_rx_pin=1, uart_baudrate=9600, max_line_buffer_size=256, on_command_received=None, on_message_received=None):
        """
        Initialize the UART Handler.

        This constructor sets up all necessary hardware components:
        1. Initializes UART interface with specified parameters
        2. Initializes status LED
        3. Sets up UART interrupts

        The UART handler is ready for communication after initialization.

        Args:
            uart_interface_id (int, optional): UART interface ID. Defaults to 0 (UART0).
                The Raspberry Pi Pico has two UART interfaces (UART0 and UART1).
                UART0 is connected to GP0 (TX) and GP1 (RX) by default.

            uart_tx_pin (int, optional): GPIO pin for UART transmission. Defaults to 0 (GP0).
                This pin is physically connected to the device's RX pin (Data In).
                The Pico transmits data to the device through this pin.

            uart_rx_pin (int, optional): GPIO pin for UART reception. Defaults to 1 (GP1).
                This pin is physically connected to the device's TX pin (Data Out).
                The Pico receives data from the device through this pin.

            uart_baudrate (int, optional): Communication speed in bits per second. Defaults to 9600.
                9600 bps is a common default for many UART devices including XBee modules.
                Higher baudrates = faster communication but require better signal quality.
                Can be changed to match device requirements (e.g., 115200, 57600, etc.).

            max_line_buffer_size (int, optional): Maximum characters to store in receive buffer.
                Defaults to 256. Prevents memory overflow from malformed data.

            on_command_received (callable, optional): Callback function for single character commands.
                If provided, this function will be called instead of the default print behavior.
                Function signature: on_command_received(character_byte, uart_handler)

            on_message_received (callable, optional): Callback function for complete line messages.
                If provided, this function will be called instead of the default print behavior.
                Function signature: on_message_received(message_string, uart_handler)
        """
        # Store configuration parameters
        self.uart_interface_id = uart_interface_id
        self.uart_tx_pin = uart_tx_pin
        self.uart_rx_pin = uart_rx_pin
        self.uart_baudrate = uart_baudrate
        self.max_line_buffer_size = max_line_buffer_size

        # Store callback functions
        self.on_command_received = on_command_received
        self.on_message_received = on_message_received

        # Initialize hardware components
        self.uart_device = self._initialize_uart_interface()
        self.status_led = self._initialize_status_led()

        # Initialize data buffer
        self.received_data_buffer = bytearray()

        # Setup UART interrupts
        self._setup_uart_interrupts()

        print("UartHandler::UART Handler initialized successfully")

    def _initialize_uart_interface(self):
        """
        Initialize UART interface for UART communication.

        This private method sets up the UART (Universal Asynchronous Receiver-Transmitter)
        peripheral on the Raspberry Pi Pico for communication with any UART device.

        UART EXPLANATION:
        ================
        UART is a hardware peripheral that handles serial communication automatically.
        It converts parallel data (bytes) into serial data (bits) for transmission
        and vice versa for reception. The UART peripheral handles:
        - Start and stop bits
        - Parity checking (optional)
        - Baud rate timing
        - Data framing

        WHY FREE UART0 FROM REPL?
        =========================
        By default, MicroPython uses UART0 for the REPL (Read-Eval-Print Loop) console,
        which allows you to interact with the Pico through a serial terminal.
        We need to free UART0 from this default assignment so we can use it for
        UART communication instead.

        Returns:
            UART: Configured UART object for UART communication
                   This object provides methods like read(), write(), and irq()

        Raises:
            Exception: If UART initialization fails (rare, usually indicates hardware issue)
        """
        # Free UART0 from REPL if it was ever attached
        # uos.dupterm() manages the REPL console connection
        # Setting it to None disconnects the REPL from UART0
        try:
            uos.dupterm(None, 1)  # 1 refers to UART0
        except:
            # If this fails, it's not critical - UART0 might not be used by REPL
            pass

        # Initialize UART for communication with any UART device
        # The UART constructor configures the hardware peripheral
        uart_device = UART(
            self.uart_interface_id,        # Use specified UART interface
            baudrate=self.uart_baudrate,   # Set communication speed
            tx=Pin(self.uart_tx_pin),      # Configure transmit pin
            rx=Pin(self.uart_rx_pin)       # Configure receive pin
        )

        # Clear any previous data in the UART buffer
        # This ensures we start with a clean communication channel
        # The underscore (_) indicates we're intentionally ignoring the return value
        _ = uart_device.read()

        return uart_device

    def _initialize_status_led(self):
        """
        Initialize the status LED for visual feedback during data reception.

        This private method sets up the built-in LED on the Raspberry Pi Pico
        to provide visual feedback when data is being received.

        LED BEHAVIOR:
        ============
        - LED turns ON when data is received (in interrupt handler)
        - LED turns OFF when data processing is complete (in interrupt handler finally block)
        - This creates a "blink" pattern that indicates active communication
        - IMPORTANT: Scheduled handlers/callbacks are unaffected by LED control

        WHY USE A STATUS LED?
        ====================
        - Visual confirmation that the system is working
        - Helps identify communication problems (no blinking = no data received)
        - Useful for debugging without needing a serial monitor
        - Provides immediate feedback during development

        Returns:
            Pin or None: LED pin object if available, None if LED is not accessible
                         Returns None gracefully to allow code to work even without LED
        """
        try:
            # Create a Pin object for the built-in LED
            # Pin.OUT means we can control the LED (turn it on/off)
            status_led = Pin("LED", Pin.OUT)
            return status_led
        except Exception:
            # If LED initialization fails, return None
            # This allows the code to continue working without LED feedback
            return None

    def _handle_received_line(self, _):
        """
        Display a complete line of received data.

        This private method is called from a scheduled context (not from interrupt context),
        making it safe to perform operations like printing and memory allocation.

        INTERRUPT CONTEXT EXPLANATION:
        =============================
        Interrupt handlers run in a special context where certain operations are restricted:
        - No printing (print() statements)
        - No memory allocation (creating new objects)
        - No complex operations that might take too long

        To safely display received data, we use micropython.schedule() to move the
        display operation out of the interrupt context into a scheduled context where
        these operations are allowed.

        LED STATUS NOTE:
        ================
        IMPORTANT: This scheduled handler function does NOT control the status LED.
        The LED is turned ON at the start of the interrupt handler and turned OFF
        at the end of the interrupt handler (in the finally block). By the time
        this scheduled function executes, the LED has already been turned off.
        This scheduled handler execution is completely unaffected by LED control.

        DATA DECODING PROCESS:
        =====================
        1. Try to decode the bytearray as UTF-8 text
        2. If that fails, extract only printable ASCII characters (32-126)
        3. Strip whitespace from the beginning and end
        4. Display the message if it's not empty

        Args:
            _: Unused parameter (required by micropython.schedule())
        """
        try:
            # Try to decode as UTF-8 with error handling
            # errors="ignore" means skip invalid characters instead of raising an exception
            received_message = self.received_data_buffer.decode(errors="ignore").strip()
        except Exception:
            # Fallback: extract only printable ASCII characters
            # This handles cases where the data isn't valid UTF-8
            # ASCII range 32-126 includes all printable characters (space through tilde)
            received_message = "".join(chr(byte) for byte in self.received_data_buffer if 32 <= byte <= 126)

        # Clear buffer for next line
        # This prepares the buffer to receive the next line of data
        self.received_data_buffer = bytearray()

        # Only process non-empty messages
        if received_message:
            # Use callback if available, otherwise use default print behavior
            if self.on_message_received:
                self.on_message_received(received_message, self)
            else:
                # Default behavior - print the message
                print("UartHandler::UART RX Line:", received_message)

    def _handle_received_character(self, character_byte):
        """
        Display a single received character.

        This private method handles single-character commands or responses from UART devices.
        It's called from a scheduled context, making it safe to print.

        LED STATUS NOTE:
        ================
        IMPORTANT: This scheduled handler function does NOT control the status LED.
        The LED is turned ON at the start of the interrupt handler and turned OFF
        at the end of the interrupt handler (in the finally block). By the time
        this scheduled function executes, the LED has already been turned off.
        This scheduled handler execution is completely unaffected by LED control.

        CHARACTER PROCESSING:
        ====================
        - Printable ASCII characters (32-126) are displayed as text
        - Non-printable characters are displayed as hexadecimal values
        - This helps identify control characters and binary data

        Args:
            character_byte (int): The received byte value (0-255)
        """
        # Ensure byte value is in valid range (0-255)
        # The & 0xFF operation masks the value to 8 bits
        byte_value = character_byte & 0xFF

        # Use callback if available, otherwise use default print behavior
        if self.on_command_received:
            # Always pass UART handler instance as second parameter
            # Callback functions should accept (character_byte, uart_handler) signature
            self.on_command_received(byte_value, self)
        else:
            # Default behavior - print the character
            if 32 <= byte_value <= 126:  # Printable ASCII range
                # Display as character
                print("UartHandler::UART RX Command:", chr(byte_value))
            else:
                # Display as hexadecimal value
                print("UartHandler::UART RX Command (0x%02X)" % byte_value)

    def _handle_uart_receive_interrupt(self, _):
        """
        UART receive interrupt handler - the heart of interrupt-driven communication.

        This private method is called automatically by the hardware whenever UART data
        arrives. It runs in interrupt context, which means it must execute quickly and
        cannot perform operations like printing or creating new objects.

        WHAT IS AN INTERRUPT HANDLER?
        =============================
        An interrupt handler is a special function that the microcontroller hardware
        automatically calls when a specific event occurs (in this case, UART data arrival).
        Think of it as a "doorbell" - when someone rings (data arrives), this function
        automatically answers (processes the data).

        INTERRUPT CONTEXT RESTRICTIONS:
        ================================
        Because this function runs in interrupt context, it has limitations:
        - NO print() statements allowed (would block other interrupts)
        - NO memory allocation for new objects (could cause memory fragmentation)
        - NO time-consuming operations (blocks other interrupts)
        - MUST return quickly (typically within microseconds)

        These restrictions exist because interrupts can occur at any time, even while
        other code is running, and the system needs to handle them efficiently without
        disrupting normal program flow.

        SOLUTION: micropython.schedule()
        =================================
        To safely perform operations that aren't allowed in interrupt context, we use
        micropython.schedule() which queues a function to run later in "normal" context
        where printing and memory allocation are allowed. Think of it as:
        - Interrupt context: "I've got data, let me note what to do with it"
        - Scheduled context: "Now I can safely process and display that data"

        DATA PROCESSING STRATEGY:
        ========================
        This handler implements a state-machine approach to data processing:

        1. READ PHASE:
           - Read all available bytes from UART hardware buffer
           - The hardware buffer is limited (typically 32 bytes), so we must drain it quickly

        2. ACCUMULATION PHASE:
           - Store non-line-ending characters in our software buffer (received_data_buffer)
           - Buffer grows as data arrives, character by character

        3. DETECTION PHASE:
           - When a line ending character (LF=10 or CR=13) is detected
           - Determine if the accumulated data is:
             a) Single character command (1 printable ASCII character)
             b) Multi-character message (any other case)

        4. SCHEDULING PHASE:
           - Schedule appropriate handler function to process the data outside interrupt context
           - Clear buffer to prepare for next transmission

        PROTOCOL DESIGN RATIONALE:
        =========================
        Why distinguish between commands and messages?
        - Single character commands: Fast responses (e.g., 'R' for reset, 'S' for status)
        - Multi-character messages: Complete data packets (e.g., "TEMP:25.3", "ERROR:404")

        This dual-mode handling provides flexibility for different communication patterns:
        - Commands: Quick interactive control
        - Messages: Structured data transfer

        LED VISUAL FEEDBACK:
        ===================
        The status LED is controlled ONLY within this interrupt handler:
        - Turns ON at the start (try block) when interrupt begins
        - Turns OFF at the end (finally block) when interrupt completes
        - This creates a brief "blink" for each data reception
        - The LED state does NOT affect scheduled handler functions
        - By the time scheduled functions run, the LED is already off

        HARDWARE BUFFER MANAGEMENT:
        ==========================
        The Raspberry Pi Pico's UART hardware has a small internal FIFO buffer
        (typically 32 bytes). When data arrives, the hardware stores it in this buffer
        and triggers our interrupt. We must read and empty this buffer quickly to:
        - Prevent buffer overflow (data loss)
        - Allow room for new incoming data
        - Maintain real-time responsiveness

        Args:
            _ (unused): Required parameter for interrupt handlers. The hardware passes
               the UART object, but we already have access to it via self.uart_device,
               so we ignore this parameter.

        Returns:
            None: Interrupt handlers don't return values
        """
        try:
            # ==================================================================
            # PHASE 1: LED ON - Visual indication that data reception started
            # ==================================================================
            # Turn on status LED to provide visual feedback
            # The LED will stay on for the duration of this interrupt handler
            # This is safe in interrupt context because it's just a hardware register write
            if self.status_led:
                # Setting LED value to 1 turns it ON
                # This is a fast hardware operation that directly writes to GPIO register
                self.status_led.value(1)

            # ==================================================================
            # PHASE 2: DRAIN HARDWARE BUFFER - Read all available data
            # ==================================================================
            # Read all available bytes from the UART hardware buffer
            # The read() method returns None if no data is available, or a bytes object
            # if data is present. This must happen quickly to prevent hardware buffer overflow.
            #
            # WHY READ ALL AT ONCE?
            # - The hardware FIFO buffer is small (32 bytes on RP2040)
            # - If we don't drain it quickly, new data will be lost (buffer overflow)
            # - Reading once is more efficient than multiple small reads
            received_data = self.uart_device.read()

            # Early exit if no data is available
            # This can happen if the interrupt was triggered but data hasn't fully arrived yet
            # or if another interrupt already processed the data
            if not received_data:
                return

            # ==================================================================
            # PHASE 3: BYTE-BY-BYTE PROCESSING - Parse the data stream
            # ==================================================================
            # Process each received byte individually
            # This loop implements our protocol parser that distinguishes between
            # regular data characters and line ending characters
            #
            # ITERATION EXPLANATION:
            # In Python/MicroPython, when you iterate over a bytes or bytearray object,
            # each iteration gives you an integer value (0-255) representing one byte
            for byte_value in received_data:

                # ============================================================
                # DECISION POINT 1: Is this byte a line ending character?
                # ============================================================
                # Line ending characters signal the end of a command or message
                # ASCII values:
                #   10 (0x0A) = Line Feed (LF) - Unix/Linux style line ending
                #   13 (0x0D) = Carriage Return (CR) - Classic Mac style line ending
                #
                # Most terminals and serial devices send one or both of these
                # characters to indicate the end of a line of text
                if byte_value in (10, 13):  # Line Feed or Carriage Return

                    # =======================================================
                    # LINE ENDING DETECTED: Process accumulated buffer
                    # =======================================================
                    # Only process if we have accumulated data
                    # Empty buffer means we received just a line ending character
                    # with no preceding data (this can happen with CR+LF pairs)
                    if self.received_data_buffer:

                        # ===================================================
                        # DECISION POINT 2: Command or Message?
                        # ===================================================
                        # Determine if the accumulated data is a single-character
                        # command or a multi-character message
                        #
                        # COMMAND CRITERIA:
                        # - Exactly 1 byte in buffer
                        # - That byte is printable ASCII (32-126)
                        #   - 32 = space (lowest printable)
                        #   - 126 = tilde ~ (highest printable)
                        #
                        # MESSAGE CRITERIA:
                        # - Multiple bytes in buffer, OR
                        # - Single byte but not printable ASCII (control character)
                        if len(self.received_data_buffer) == 1 and 32 <= self.received_data_buffer[0] <= 126:
                            # ===============================================
                            # SINGLE CHARACTER COMMAND DETECTED
                            # ===============================================
                            # Extract the single byte from the buffer
                            # This is our command byte (e.g., 'R', 'S', 'A', etc.)
                            command_byte = self.received_data_buffer[0]

                            # Schedule the command handler to process this command
                            # in normal context where printing is allowed
                            #
                            # WHY TRY-EXCEPT?
                            # The schedule queue has limited size (typically 32 entries)
                            # If the queue is full (too many pending scheduled calls),
                            # the schedule() call will raise an exception
                            try:
                                # Schedule _handle_received_character to run outside interrupt context
                                # Pass the command_byte as the parameter
                                #
                                # IMPORTANT: micropython.schedule() signature:
                                #   schedule(func, arg) - schedules func to be called with arg
                                #   The arg must be an integer or None
                                micropython.schedule(self._handle_received_character, command_byte)
                            except Exception:
                                # If scheduling fails (queue full), silently ignore
                                # This prevents the interrupt handler from crashing
                                # The command will be lost, but the system remains stable
                                #
                                # Alternative approach: Could set an error flag here
                                pass  # Ignore if schedule queue is full

                            # ===============================================
                            # BUFFER CLEANUP
                            # ===============================================
                            # Clear the buffer by creating a new empty bytearray
                            #
                            # WHY CREATE NEW INSTEAD OF CLEARING?
                            # In MicroPython interrupt context:
                            # - Creating a small empty bytearray() is safe and fast
                            # - Alternative methods (clear(), del [:], etc.) might
                            #   trigger memory operations that could cause issues
                            # - Reassignment is the most reliable method in interrupt context
                            self.received_data_buffer = bytearray()
                        else:
                            # ===============================================
                            # MULTI-CHARACTER MESSAGE DETECTED
                            # ===============================================
                            # The buffer contains either:
                            # - Multiple characters (a message/string)
                            # - A single non-printable character (control character)

                            # Schedule the line handler to process this message
                            # in normal context where printing and decoding are allowed
                            try:
                                # Schedule _handle_received_line to run outside interrupt context
                                # Pass 0 as the parameter (unused, but required by schedule())
                                #
                                # NOTE: The line handler will access self.received_data_buffer
                                # directly, so we don't pass the buffer content as a parameter
                                micropython.schedule(self._handle_received_line, 0)
                            except Exception:
                                # ===============================================
                                # EMERGENCY BUFFER CLEAR
                                # ===============================================
                                # If scheduling fails, we MUST clear the buffer
                                # Otherwise, the buffer would grow indefinitely as new
                                # data arrives, eventually causing memory exhaustion
                                #
                                # This is a safety mechanism: better to lose one message
                                # than to crash the entire system
                                self.received_data_buffer = bytearray()

                # ============================================================
                # DECISION POINT 3: Regular character - accumulate it
                # ============================================================
                # This byte is not a line ending, so it's part of a command/message
                # Add it to our buffer for later processing
                else:
                    # =======================================================
                    # BUFFER OVERFLOW PROTECTION
                    # =======================================================
                    # Check if buffer has room before appending
                    # max_line_buffer_size prevents runaway memory growth from
                    # malformed data or missing line endings
                    #
                    # WHAT HAPPENS IF BUFFER IS FULL?
                    # - New characters are silently dropped
                    # - This prevents memory overflow but truncates long messages
                    # - When line ending arrives, the truncated message is processed
                    #
                    # WHY THIS APPROACH?
                    # - Memory safety is critical in embedded systems
                    # - Limited RAM on microcontrollers (264KB on Pico)
                    # - Better to truncate than to crash
                    if len(self.received_data_buffer) < self.max_line_buffer_size:
                        # Append the byte to our software buffer
                        # bytearray.append() is safe in interrupt context when the
                        # bytearray already exists (no new memory allocation needed)
                        self.received_data_buffer.append(byte_value)
                    # If buffer is full, the byte is silently dropped
                    # This could be logged in a more advanced implementation

        # ==================================================================
        # PHASE 4: CLEANUP - Guaranteed to run regardless of errors
        # ==================================================================
        finally:
            # Turn off status LED to indicate interrupt processing is complete
            # The 'finally' block ensures this happens even if an exception occurs
            # in the try block, preventing the LED from getting stuck ON
            #
            # WHY USE FINALLY?
            # - Guarantees LED turns off even if errors occur
            # - Provides consistent visual feedback
            # - Prevents LED from staying on permanently (confusing)
            #
            # IMPORTANT TIMING NOTE:
            # By the time the scheduled handler functions (_handle_received_character
            # or _handle_received_line) execute, this finally block has already run
            # and the LED is off. The LED state only indicates interrupt activity,
            # not scheduled function activity.
            if self.status_led:
                # Setting LED value to 0 turns it OFF
                # This completes the "blink" cycle
                self.status_led.value(0)

    def _setup_uart_interrupts(self):
        """
        Configure UART interrupts for automatic data reception.

        This private method sets up the hardware interrupt system that automatically
        calls our interrupt handler whenever data arrives at the UART.

        INTERRUPT TRIGGERS EXPLAINED:
        ============================
        Interrupt triggers determine when the interrupt handler is called:
        - UART.RX_ANY: Triggered when any data is received
        - UART.IRQ_RXIDLE: Triggered when the receive line becomes idle
          (useful for detecting end of transmission)

        FIRMWARE COMPATIBILITY:
        ======================
        Different versions of MicroPython firmware may have different interrupt
        trigger constants. This method checks for availability and uses
        whatever triggers are supported.

        WHY USE INTERRUPTS?
        ==================
        Without interrupts, we would need to constantly check for incoming data
        (polling), which wastes CPU cycles and can miss data. Interrupts provide:
        - Immediate response to incoming data
        - Efficient CPU usage (CPU can do other tasks between data reception)
        - No data loss due to timing issues
        - Real-time communication capability

        Raises:
            RuntimeError: If UART IRQ triggers are not available on this firmware
                         This indicates very old firmware that should be updated
        """
        # Clear any remaining data in the buffer before enabling interrupts
        # This ensures we start with a clean state
        try:
            _ = self.uart_device.read()
        except Exception:
            # If reading fails, continue anyway - interrupts will still work
            pass

        # Determine available interrupt triggers based on firmware version
        # We use bitwise OR (|) to combine multiple triggers if available
        interrupt_triggers = 0

        # Check for RX_ANY trigger (available in most firmware versions)
        if hasattr(UART, "RX_ANY"):
            interrupt_triggers |= UART.RX_ANY
            print("UartHandler::UART.RX_ANY trigger available")

        # Check for IRQ_RXIDLE trigger (available in newer firmware versions)
        if hasattr(UART, "IRQ_RXIDLE"):
            interrupt_triggers |= UART.IRQ_RXIDLE
            print("UartHandler::UART.IRQ_RXIDLE trigger available")

        # Configure interrupts if any triggers are available
        if interrupt_triggers:
            # Create a lambda function that calls our interrupt handler
            interrupt_handler = lambda _: self._handle_uart_receive_interrupt(_)

            # Attach our interrupt handler to the UART
            # The handler will be called automatically when triggers occur
            self.uart_device.irq(handler=interrupt_handler, trigger=interrupt_triggers)
            print(f"UartHandler::UART interrupts configured with triggers: 0x{interrupt_triggers:02X}")
        else:
            # Very old firmware fallback (rare). If you hit this, update firmware.
            raise RuntimeError("UartHandler::UART IRQ triggers not available on this firmware")

    # =============================================================================
    # PUBLIC UTILITY METHODS
    # =============================================================================

    def clear_receive_buffer(self):
        """
        Clear the receive buffer.

        This method can be called to manually clear the receive buffer,
        useful for resetting the communication state.
        """
        self.received_data_buffer = bytearray()

    def get_receive_buffer(self):
        """
        Get a copy of the current receive buffer.

        Returns:
            bytearray: A copy of the current receive buffer
        """
        return bytearray(self.received_data_buffer)

    def get_buffer_size(self):
        """
        Get the current size of the receive buffer.

        Returns:
            int: Current number of bytes in the receive buffer
        """
        return len(self.received_data_buffer)

    def write(self, data):
        """
        Write data to the UART device.

        This is a convenience method that provides direct access to the
        UART device's write method.

        Args:
            data (bytes): Data to write to the UART device

        Returns:
            int: Number of bytes written
        """
        return self.uart_device.write(data)

    def read(self):
        """
        Read data from the UART device.

        This is a convenience method that provides direct access to the
        UART device's read method.

        Returns:
            bytes or None: Data read from the UART device, or None if no data available
        """
        return self.uart_device.read()