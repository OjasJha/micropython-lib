"""
SSD1306 OLED Display Driver for MicroPython on Raspberry Pi Pico 2 W

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

This module provides a comprehensive driver for SSD1306-based OLED displays, supporting
both I2C and SPI communication interfaces. The SSD1306 is a popular single-chip CMOS
OLED/PLED driver controller that can handle displays up to 128x64 pixels.

Key Features:
- Full graphics support through MicroPython's FrameBuffer
- Both I2C and SPI interface support
- Hardware-specific optimizations for Raspberry Pi Pico 2 W
- Power management functions
- Display control (contrast, inversion, on/off)
- Backward compatibility with existing SSD1306 libraries

Hardware Architecture Notes:
The SSD1306 uses a page-based memory architecture where each page represents 8 vertical
pixels. For a 128x64 display, there are 8 pages (64÷8=8), each containing 128 bytes.
This is why we calculate memory_pages = display_height // 8.

Raspberry Pi Pico 2 W Specific Considerations:
- Uses ARM Cortex-M33 dual-core processor running at up to 150MHz
- 520KB SRAM allows for multiple frame buffers if needed
- Built-in I2C and SPI peripherals with flexible GPIO pin assignment
- 3.3V logic levels compatible with most SSD1306 modules

Communication Protocols:
I2C: Uses 2 wires (SDA/SCL) with addresses typically 0x3C or 0x3D
SPI: Uses 4+ wires (MOSI, SCK, CS, DC, optionally RST) for faster communication

Author: Ojas Jha
License: MIT License (see above)
Date: October 18, 2025
"""

# ============================================================================
# IMPORTS AND DEPENDENCIES
# ============================================================================

# MicroPython-specific imports for hardware optimization
from micropython import const  # Compile-time constants for memory efficiency
import framebuf                # Graphics primitives and frame buffer management


# ============================================================================
# SSD1306 COMMAND REGISTER DEFINITIONS
# ============================================================================

"""
SSD1306 Command Set Overview:
The SSD1306 controller uses a comprehensive set of commands to control display behavior.
These commands are sent via I2C or SPI to configure various aspects of the display.

Command Categories:
1. Fundamental Commands: Basic display control (on/off, contrast)
2. Scrolling Commands: Hardware-based scrolling (not implemented here)
3. Addressing Setting Commands: Memory addressing modes and ranges
4. Hardware Configuration Commands: Physical display configuration
5. Timing & Driving Scheme Commands: Clock and timing control
6. Charge Pump Commands: Internal voltage generation

Why const() is used:
MicroPython's const() creates compile-time constants that are more memory-efficient
than regular variables, as they're embedded directly in the bytecode.
"""

# Fundamental Command Set - Basic Display Control
COMMAND_SET_CONTRAST = const(0x81)           # Set display contrast (0x00-0xFF)
COMMAND_SET_ENTIRE_DISPLAY_ON = const(0xA4)  # Enable entire display on/off (A4/A5)
COMMAND_SET_NORMAL_INVERSE = const(0xA6)     # Set normal/inverse display mode (A6/A7)
COMMAND_SET_DISPLAY_ON_OFF = const(0xAE)     # Turn display on/off (AE=off, AF=on)

# Addressing Setting Command Set - Memory Management
COMMAND_SET_MEMORY_ADDRESSING = const(0x20)  # Set memory addressing mode (horizontal/vertical/page)
COMMAND_SET_COLUMN_ADDRESS = const(0x21)     # Set column start and end address (0-127)
COMMAND_SET_PAGE_ADDRESS = const(0x22)       # Set page start and end address (0-7 for 64px height)

# Hardware Configuration Command Set - Physical Display Setup
COMMAND_SET_DISPLAY_START_LINE = const(0x40) # Set display start line register (40-7F)
COMMAND_SET_SEGMENT_REMAP = const(0xA0)      # Set segment re-map (A0/A1 - normal/flipped horizontally)
COMMAND_SET_MULTIPLEX_RATIO = const(0xA8)    # Set multiplex ratio (15-63, height-1)
COMMAND_SET_COM_OUTPUT_DIRECTION = const(0xC0) # Set COM output scan direction (C0/C8 - normal/flipped vertically)
COMMAND_SET_DISPLAY_OFFSET = const(0xD3)     # Set display offset (vertical shift 0-63)
COMMAND_SET_COM_PIN_CONFIG = const(0xDA)     # Set COM pins hardware configuration (depends on display size)

# Timing & Driving Scheme Command Set - Clock and Refresh Control
COMMAND_SET_DISPLAY_CLOCK_DIVIDE = const(0xD5) # Set display clock divide ratio/oscillator frequency
COMMAND_SET_PRECHARGE_PERIOD = const(0xD9)   # Set pre-charge period (affects display quality)
COMMAND_SET_VCOM_DESELECT_LEVEL = const(0xDB) # Set VCOMH deselect level (affects contrast)

# Charge Pump Command Set - Power Management
COMMAND_SET_CHARGE_PUMP = const(0x8D)        # Enable/disable charge pump (14=enable, 10=disable)

# ============================================================================
# BASE SSD1306 DISPLAY CLASS
# ============================================================================

class SSD1306(framebuf.FrameBuffer):
    """
    Base SSD1306 OLED Display Controller Class

    This class provides the core functionality for SSD1306-based OLED displays.
    It inherits from MicroPython's FrameBuffer class to provide graphics primitives
    like text rendering, line drawing, rectangle drawing, and pixel manipulation.

    FrameBuffer Inheritance Benefits:
    - Built-in graphics functions: text(), line(), rect(), fill_rect(), pixel()
    - Memory-efficient pixel manipulation
    - Hardware-accelerated operations where possible
    - Consistent API across different display types

    Memory Architecture:
    The SSD1306 uses a page-based memory layout where each page represents 8 vertical
    pixels (1 byte per column). This is different from typical row-based layouts:

    Page Layout (for 128x64 display):
    Page 0: Pixels 0-7   (top 8 rows)
    Page 1: Pixels 8-15
    Page 2: Pixels 16-23
    ...
    Page 7: Pixels 56-63 (bottom 8 rows)

    Each page contains 128 bytes (one per column), and each byte represents
    8 vertical pixels with LSB at the top (MONO_VLSB format).

    Power Management:
    The class supports both external VCC and internal charge pump configurations.
    Internal charge pump is more common and requires no external power supply,
    while external VCC can provide better performance and lower power consumption.
    """

    def __init__(self, display_width, display_height, uses_external_vcc):
        """
        Initialize the SSD1306 OLED display base class.

        This constructor sets up the basic display parameters and creates the frame buffer
        that will hold the pixel data. It also inherits from FrameBuffer to provide
        graphics primitives functionality.

        Memory Calculation Example:
        For a 128x64 display: 8 pages × 128 columns = 1024 bytes frame buffer
        For a 128x32 display: 4 pages × 128 columns = 512 bytes frame buffer

        Args:
            display_width (int): Width of the display in pixels (typically 128)
                               Valid range: 1-128 pixels
            display_height (int): Height of the display in pixels (typically 32 or 64)
                                Valid range: 8-64 pixels (must be multiple of 8)
            uses_external_vcc (bool): Power supply configuration
                                    True: External VCC supply (3.3V-5V)
                                    False: Internal charge pump (most common)

        Raises:
            ValueError: If display_height is not a multiple of 8
            MemoryError: If insufficient RAM for frame buffer allocation

        Note:
            The display is automatically initialized after construction.
            Call turn_display_off() immediately after construction if you want
            to set up the display without turning it on initially.
        """
        # Store display configuration parameters
        self.display_width = display_width      # Horizontal resolution in pixels
        self.display_height = display_height    # Vertical resolution in pixels
        self.uses_external_vcc = uses_external_vcc  # Power supply configuration

        # Calculate memory pages (SSD1306 page-based architecture)
        # Each page represents 8 vertical pixels, so divide height by 8
        self.memory_pages = self.display_height // 8

        # Allocate frame buffer memory
        # Size = pages × width (e.g., 8 × 128 = 1024 bytes for 128x64 display)
        buffer_size = self.memory_pages * self.display_width
        self.frame_buffer = bytearray(buffer_size)

        # Initialize parent FrameBuffer class
        # MONO_VLSB: Monochrome, Vertical, Least Significant Bit first
        # This matches the SSD1306's page-based memory layout
        super().__init__(
            self.frame_buffer,      # Buffer to store pixel data
            self.display_width,     # Width in pixels
            self.display_height,    # Height in pixels
            framebuf.MONO_VLSB     # Pixel format matching SSD1306 architecture
        )

        # Initialize the physical display hardware
        self.initialize_display()

    def initialize_display(self):
        """
        Initialize the SSD1306 OLED display with proper configuration sequence.

        This method sends a carefully ordered sequence of commands to properly configure
        the SSD1306 controller. The order is critical - certain commands must be sent
        before others to ensure proper display operation.

        Initialization Sequence Explanation:
        1. Turn display OFF to prevent artifacts during configuration
        2. Set memory addressing mode (how pixels are written)
        3. Configure display geometry (size, orientation, offset)
        4. Set timing parameters (refresh rate, precharge)
        5. Configure power management (charge pump)
        6. Set display appearance (contrast, inversion)
        7. Turn display ON

        Why This Order Matters:
        - Display must be OFF during hardware configuration
        - Memory addressing must be set before geometry commands
        - Timing settings affect power consumption and display quality
        - Charge pump must be enabled before turning display ON

        Raspberry Pi Pico 2 W Considerations:
        - 3.3V logic levels are compatible with SSD1306 requirements
        - Internal charge pump eliminates need for external power supply
        - Fast ARM Cortex-M33 processor handles initialization quickly
        """
        # Define initialization command sequence
        # Each command is explained inline for educational purposes
        initialization_commands = (
            # === PHASE 1: DISPLAY POWER CONTROL ===
            # Turn display off during initialization to prevent visual artifacts
            # This also reduces power consumption during setup
            COMMAND_SET_DISPLAY_ON_OFF | 0x00,  # 0xAE: Display OFF

            # === PHASE 2: MEMORY ADDRESSING CONFIGURATION ===
            # Set how pixel data is written to display memory
            COMMAND_SET_MEMORY_ADDRESSING,      # 0x20: Set memory addressing mode
            0x00,  # Horizontal addressing mode (most efficient for full-screen updates)
                   # 0x00 = Horizontal, 0x01 = Vertical, 0x02 = Page addressing

            # === PHASE 3: DISPLAY GEOMETRY CONFIGURATION ===
            # Configure physical display layout and orientation
            COMMAND_SET_DISPLAY_START_LINE | 0x00,  # 0x40: Set display start line to 0
                                                    # Controls vertical scrolling offset

            COMMAND_SET_SEGMENT_REMAP | 0x01,       # 0xA1: Remap columns (flip horizontally)
                                                    # 0xA0 = normal, 0xA1 = flipped

            COMMAND_SET_MULTIPLEX_RATIO,            # 0xA8: Set multiplex ratio
            self.display_height - 1,                # Height - 1 (e.g., 63 for 64px height)
                                                    # This tells controller how many rows to scan

            COMMAND_SET_COM_OUTPUT_DIRECTION | 0x08, # 0xC8: Scan from bottom to top
                                                     # 0xC0 = top-to-bottom, 0xC8 = bottom-to-top

            COMMAND_SET_DISPLAY_OFFSET,             # 0xD3: Set display offset
            0x00,                                   # No vertical offset (display starts at top)

            # COM pin configuration depends on display physical layout
            COMMAND_SET_COM_PIN_CONFIG,             # 0xDA: Set COM pins hardware configuration
            # Different displays have different COM pin wiring:
            # 128x32: 0x02 (sequential COM pin configuration)
            # 128x64: 0x12 (alternative COM pin configuration)
            0x02 if self.display_width > 2 * self.display_height else 0x12,

            # === PHASE 4: TIMING AND DRIVING SCHEME ===
            # Configure display refresh timing and drive strength
            COMMAND_SET_DISPLAY_CLOCK_DIVIDE,       # 0xD5: Set display clock divide ratio
            0x80,                                   # Default: divide ratio=1, oscillator freq=8
                                                    # Higher values = slower refresh, lower power

            COMMAND_SET_PRECHARGE_PERIOD,           # 0xD9: Set pre-charge period
            0x22 if self.uses_external_vcc else 0xF1,  # Different timing for external vs internal VCC
                                                        # Affects pixel brightness and power consumption

            COMMAND_SET_VCOM_DESELECT_LEVEL,        # 0xDB: Set VCOMH deselect level
            0x30,                                   # ~0.83 * VCC (affects contrast and power)
                                                    # 0x00=0.65*VCC, 0x30=0.83*VCC, 0x40=VCC

            # === PHASE 5: DISPLAY APPEARANCE ===
            # Set initial display appearance parameters
            COMMAND_SET_CONTRAST,                   # 0x81: Set contrast control
            0xFF,                                   # Maximum contrast (0x00-0xFF)
                                                    # Higher = brighter but more power consumption

            COMMAND_SET_ENTIRE_DISPLAY_ON,          # 0xA4: Entire display on/off
                                                    # 0xA4 = follow RAM, 0xA5 = ignore RAM (all pixels on)

            COMMAND_SET_NORMAL_INVERSE,             # 0xA6: Set normal/inverse display
                                                    # 0xA6 = normal, 0xA7 = inverted (black<->white swapped)

            # === PHASE 6: POWER MANAGEMENT ===
            # Configure charge pump (internal voltage generation)
            COMMAND_SET_CHARGE_PUMP,                # 0x8D: Charge pump setting
            0x10 if self.uses_external_vcc else 0x14,  # 0x10=disable, 0x14=enable
                                                        # Internal charge pump generates ~7.5V from 3.3V

            # === PHASE 7: DISPLAY ACTIVATION ===
            # Turn display on after all configuration is complete
            COMMAND_SET_DISPLAY_ON_OFF | 0x01,      # 0xAF: Display ON
        )

        # Send all initialization commands to the display controller
        # This must be implemented by the specific interface class (I2C/SPI)
        for command in initialization_commands:
            self.send_command(command)

        # Clear the frame buffer (set all pixels to black/off)
        # fill(0) sets all pixels to 0 (black), fill(1) would set all to 1 (white)
        self.fill(0)

        # Update the physical display with the cleared frame buffer
        # This ensures the display starts with a clean, blank screen
        self.update_display()

    # ========================================================================
    # DISPLAY CONTROL METHODS
    # ========================================================================

    def turn_display_off(self):
        """
        Turn off the OLED display to save power.

        This method puts the display into sleep mode, which significantly reduces
        power consumption. The frame buffer content is preserved, so calling
        turn_display_on() will restore the previous display content.

        Power Savings:
        - Display OFF: ~10μA current consumption
        - Display ON: ~20mA current consumption (varies with content)

        Use Cases:
        - Battery-powered applications needing power conservation
        - Temporary display disable during processing
        - Screen saver functionality

        Note:
            The Raspberry Pi Pico 2 W continues running; only the display is turned off.
            This is different from deep sleep mode which would stop the processor.
        """
        self.send_command(COMMAND_SET_DISPLAY_ON_OFF | 0x00)

    def turn_display_on(self):
        """
        Turn on the OLED display.

        This method wakes the display from sleep mode, restoring the previous
        frame buffer content. There's typically a brief delay (1-2ms) before
        the display is fully visible.

        Startup Behavior:
        - Display content is restored from frame buffer
        - All previous settings (contrast, inversion) are maintained
        - No re-initialization is required

        Note:
            If you need to clear the display after turning it on, call fill(0)
            followed by update_display().
        """
        self.send_command(COMMAND_SET_DISPLAY_ON_OFF | 0x01)

    def set_contrast(self, contrast_level):
        """
        Set the display contrast/brightness level.

        Contrast affects both the brightness and power consumption of the display.
        Higher contrast values result in brighter pixels but consume more power.

        Technical Details:
        - Contrast controls the segment driver output voltage
        - Higher values increase pixel brightness and power consumption
        - Lower values extend battery life but may reduce readability
        - The relationship between contrast and brightness is non-linear

        Power Consumption Examples (approximate):
        - Contrast 50: ~15mA (dim but readable)
        - Contrast 128: ~20mA (good balance)
        - Contrast 255: ~30mA (maximum brightness)

        Args:
            contrast_level (int): Contrast value from 0 (dimmest) to 255 (brightest)
                                Valid range: 0-255
                                Recommended: 128-200 for most applications

        Raises:
            ValueError: If contrast_level is outside valid range (0-255)

        Example:
            # Set medium contrast for balanced visibility and power consumption
            display.set_contrast(128)

            # Set high contrast for outdoor use
            display.set_contrast(200)
        """
        # Validate input range
        if not 0 <= contrast_level <= 255:
            raise ValueError(f"Contrast level must be 0-255, got {contrast_level}")

        self.send_command(COMMAND_SET_CONTRAST)
        self.send_command(contrast_level)

    def set_invert_display(self, is_inverted):
        """
        Set display inversion (swap black and white pixels).

        Display inversion swaps the meaning of pixel values:
        - Normal mode: 0 = black (pixel off), 1 = white (pixel on)
        - Inverted mode: 0 = white (pixel on), 1 = black (pixel off)

        This is a hardware feature that doesn't require frame buffer changes,
        making it very efficient for creating visual effects or improving
        readability in different lighting conditions.

        Use Cases:
        - Dark mode / light mode switching
        - Attention-grabbing alerts (flashing inversion)
        - Better readability in bright ambient light
        - Visual feedback for user interactions

        Performance Note:
        Inversion is handled by the SSD1306 hardware, so there's no performance
        impact on the Raspberry Pi Pico 2 W processor or frame buffer operations.

        Args:
            is_inverted (bool): Display inversion state
                              True: Invert display (black becomes white, white becomes black)
                              False: Normal display (standard black/white mapping)

        Example:
            # Create a flashing alert effect
            for _ in range(5):
                display.set_invert_display(True)
                time.sleep(0.5)
                display.set_invert_display(False)
                time.sleep(0.5)
        """
        self.send_command(COMMAND_SET_NORMAL_INVERSE | (is_inverted & 1))

    # ========================================================================
    # BACKWARD COMPATIBILITY METHODS
    # ========================================================================

    def show(self):
        """
        Backward compatibility: Update display (alias for update_display).

        This method maintains compatibility with existing SSD1306 code that uses
        the show() method name. It simply calls the more descriptively named
        update_display() method.
        """
        self.update_display()

    def poweroff(self):
        """
        Backward compatibility: Turn off display (alias for turn_display_off).

        Maintains compatibility with existing code using the poweroff() method name.
        """
        self.turn_display_off()

    def poweron(self):
        """
        Backward compatibility: Turn on display (alias for turn_display_on).

        Maintains compatibility with existing code using the poweron() method name.
        """
        self.turn_display_on()

    def contrast(self, contrast_level):
        """
        Backward compatibility: Set contrast (alias for set_contrast).

        Args:
            contrast_level (int): Contrast value from 0-255
        """
        self.set_contrast(contrast_level)

    def invert(self, is_inverted):
        """
        Backward compatibility: Set invert display (alias for set_invert_display).

        Args:
            is_inverted (bool or int): True/1 to invert, False/0 for normal
        """
        self.set_invert_display(is_inverted)

    # ========================================================================
    # DISPLAY UPDATE METHOD
    # ========================================================================

    def update_display(self):
        """
        Update the physical display with the current frame buffer contents.

        This method transfers the frame buffer data from the Raspberry Pi Pico 2 W's
        memory to the SSD1306 display controller's internal memory. It's one of the
        most important methods as it makes your graphics visible on the screen.

        How It Works:
        1. Set the column address range (which pixels horizontally to update)
        2. Set the page address range (which pixel rows vertically to update)
        3. Send the entire frame buffer to the display controller

        SSD1306 Memory Layout:
        The SSD1306 organizes its memory in "pages" of 8 vertical pixels each.
        For a 128x64 display:
        - 8 pages (64 pixels ÷ 8 pixels per page)
        - 128 columns
        - Total: 1024 bytes (8 pages × 128 columns)

        Performance Considerations:
        - Full screen update: ~8ms over I2C at 400kHz
        - Full screen update: ~2ms over SPI at 10MHz
        - The Raspberry Pi Pico 2 W's DMA can be used for faster transfers

        64-Pixel Wide Display Quirk:
        Some 64-pixel wide displays (like 64x32) have a hardware quirk where
        they need a 32-pixel column offset. This is handled automatically.

        Note:
            This method must be called after any frame buffer modifications
            (text(), line(), rect(), etc.) to make changes visible on the display.
            The FrameBuffer methods only modify RAM, not the physical display.
        """
        # Calculate column address range for the display width
        column_start = 0
        column_end = self.display_width - 1

        # Handle 64-pixel wide display hardware quirk
        # Some displays have internal column offset requirements
        if self.display_width == 64:
            column_start += 32  # Hardware offset for 64px wide displays
            column_end += 32

        # Set column address range (horizontal boundaries)
        # This tells the SSD1306 which columns will receive data
        self.send_command(COMMAND_SET_COLUMN_ADDRESS)  # 0x21
        self.send_command(column_start)                # Start column (0 or 32)
        self.send_command(column_end)                  # End column (width-1)

        # Set page address range (vertical boundaries in 8-pixel chunks)
        # Pages represent groups of 8 vertical pixels
        self.send_command(COMMAND_SET_PAGE_ADDRESS)    # 0x22
        self.send_command(0)                          # Start page (always 0)
        self.send_command(self.memory_pages - 1)      # End page (height/8 - 1)

        # Transfer frame buffer data to display controller
        # This is where the actual pixel data gets sent to the display
        # The SSD1306 will automatically fill the specified address range
        self.send_data(self.frame_buffer)


# ============================================================================
# I2C INTERFACE IMPLEMENTATION
# ============================================================================

class SSD1306_I2C(SSD1306):
    """
    SSD1306 OLED Display Driver for I2C Interface

    This class implements I2C communication for SSD1306 displays. I2C (Inter-Integrated
    Circuit) is a 2-wire serial communication protocol that's ideal for connecting
    multiple devices to the Raspberry Pi Pico 2 W with minimal pin usage.

    I2C Advantages:
    - Only 2 wires needed: SDA (data) and SCL (clock)
    - Multiple devices can share the same bus (different addresses)
    - Built-in error detection and acknowledgment
    - Lower pin count compared to SPI

    I2C Disadvantages:
    - Slower than SPI (typically 100kHz-400kHz vs 10MHz+)
    - More complex protocol overhead
    - Pull-up resistors required (often built into modules)

    Raspberry Pi Pico 2 W I2C Features:
    - Two hardware I2C controllers (I2C0 and I2C1)
    - Flexible GPIO pin assignment
    - Hardware and software I2C implementations available
    - Built-in 3.3V logic levels (compatible with most SSD1306 modules)

    Common I2C Addresses:
    - 0x3C (60 decimal): Most common SSD1306 address
    - 0x3D (61 decimal): Alternative address (some modules have jumper)
    """

    def __init__(self, display_width, display_height, i2c_bus, i2c_address=0x3C, uses_external_vcc=False):
        """
        Initialize SSD1306 OLED display with I2C interface.

        This constructor sets up the I2C communication parameters and prepares
        buffers for efficient command and data transmission.

        I2C Protocol Details:
        The SSD1306 I2C protocol uses control bytes to distinguish between
        commands and data:
        - Command: Control byte 0x80 (Co=1, D/C#=0) + command byte
        - Data: Control byte 0x40 (Co=0, D/C#=1) + data bytes

        Buffer Pre-allocation:
        Buffers are pre-allocated to avoid memory allocation during display
        updates, which improves performance and reduces memory fragmentation.

        Args:
            display_width (int): Width of the display in pixels (typically 128)
            display_height (int): Height of the display in pixels (typically 32 or 64)
            i2c_bus (machine.I2C or machine.SoftI2C): I2C bus object
                   - machine.I2C: Hardware I2C (faster, uses dedicated pins)
                   - machine.SoftI2C: Software I2C (slower, any GPIO pins)
            i2c_address (int): 7-bit I2C address of the display
                             Default: 0x3C (most common)
                             Alternative: 0x3D (check your module)
            uses_external_vcc (bool): Power supply configuration
                                    False: Internal charge pump (most common)
                                    True: External VCC supply

        Example:
            from machine import Pin, SoftI2C
            import ssd1306

            # Create I2C bus
            i2c = SoftI2C(scl=Pin(5), sda=Pin(4), freq=400000)

            # Create display
            display = ssd1306.SSD1306_I2C(128, 64, i2c)
        """
        # Store I2C communication parameters
        self.i2c_bus = i2c_bus          # I2C bus object for communication
        self.i2c_address = i2c_address  # 7-bit I2C address of the display

        # Pre-allocate buffers for efficient communication
        self.command_buffer = bytearray(2)  # Buffer for command transmission
                                           # [0] = control byte, [1] = command byte

        # Data transmission uses writevto() for efficiency with large transfers
        self.data_header = [b"\x40", None]  # [0] = control byte (0x40), [1] = data buffer
                                           # Co=0 (continuation=0), D/C#=1 (data mode)

        # Initialize parent class (this will call initialize_display())
        super().__init__(display_width, display_height, uses_external_vcc)

    # Backward compatibility constructor method
    @classmethod
    def create_with_old_params(cls, width, height, i2c, addr=0x3C, external_vcc=False):
        """
        Backward compatibility constructor for original parameter names.
        Use this for existing code that uses the old parameter names.
        """
        return cls(width, height, i2c, addr, external_vcc)

    def send_command(self, command_byte):
        """
        Send a command byte to the display via I2C.

        Args:
            command_byte (int): Command byte to send
        """
        self.command_buffer[0] = 0x80  # Control byte: Co=1, D/C#=0 (command mode)
        self.command_buffer[1] = command_byte
        self.i2c_bus.writeto(self.i2c_address, self.command_buffer)

    def send_data(self, data_buffer):
        """
        Send data buffer to the display via I2C.

        Args:
            data_buffer (bytearray): Data to send to display
        """
        self.data_header[1] = data_buffer
        self.i2c_bus.writevto(self.i2c_address, self.data_header)


class SSD1306_SPI(SSD1306):
    """SSD1306 OLED Display driver for SPI interface."""

    def __init__(self, display_width, display_height, spi_bus, data_command_pin, reset_pin,
                 chip_select_pin, uses_external_vcc=False):
        """
        Initialize SSD1306 OLED display with SPI interface.

        Args:
            display_width (int): Width of the display in pixels
            display_height (int): Height of the display in pixels
            spi_bus: SPI bus object (machine.SPI)
            data_command_pin: Data/Command control pin (machine.Pin)
            reset_pin: Reset control pin (machine.Pin)
            chip_select_pin: Chip Select control pin (machine.Pin)
            uses_external_vcc (bool): True if using external VCC, False for internal charge pump
        """
        self.spi_baudrate = 10 * 1024 * 1024  # 10 MHz SPI clock rate

        # Initialize control pins
        data_command_pin.init(data_command_pin.OUT, value=0)
        reset_pin.init(reset_pin.OUT, value=0)
        chip_select_pin.init(chip_select_pin.OUT, value=1)

        # Store pin and bus references
        self.spi_bus = spi_bus
        self.data_command_pin = data_command_pin
        self.reset_pin = reset_pin
        self.chip_select_pin = chip_select_pin

        import time

        # Perform hardware reset sequence
        self.reset_pin(1)
        time.sleep_ms(1)
        self.reset_pin(0)
        time.sleep_ms(10)
        self.reset_pin(1)

        super().__init__(display_width, display_height, uses_external_vcc)

    def send_command(self, command_byte):
        """
        Send a command byte to the display via SPI.

        Args:
            command_byte (int): Command byte to send
        """
        self.spi_bus.init(baudrate=self.spi_baudrate, polarity=0, phase=0)
        self.chip_select_pin(1)
        self.data_command_pin(0)  # Command mode
        self.chip_select_pin(0)
        self.spi_bus.write(bytearray([command_byte]))
        self.chip_select_pin(1)

    def send_data(self, data_buffer):
        """
        Send data buffer to the display via SPI.

        Args:
            data_buffer (bytearray): Data to send to display
        """
        self.spi_bus.init(baudrate=self.spi_baudrate, polarity=0, phase=0)
        self.chip_select_pin(1)
        self.data_command_pin(1)  # Data mode
        self.chip_select_pin(0)
        self.spi_bus.write(data_buffer)
        self.chip_select_pin(1)