# UART Communication Library for Raspberry Pi Pico

A comprehensive, interrupt-driven UART communication library for Raspberry Pi Pico using MicroPython. This library provides robust serial communication capabilities with built-in interrupt handling, LED status indication, and flexible callback support.

## MIT License

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

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Files in This Module](#files-in-this-module)
- [Hardware Requirements](#hardware-requirements)
- [Wiring Diagrams](#wiring-diagrams)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [API Documentation](#api-documentation)
- [Examples](#examples)
- [Power Supply Considerations](#power-supply-considerations)
- [Troubleshooting](#troubleshooting)
- [Author](#author)

---

## Overview

This library provides three main components for UART communication on the Raspberry Pi Pico:

1. **`uart_handler.py`** - Core UART handler class with interrupt-driven communication
2. **`uart_loopback_test.py`** - Hardware loopback test for UART verification
3. **`uart_full_duplex_test.py`** - Full duplex communication test with XBee/UART devices

The library supports any UART-compatible device including XBee modules, GPS receivers, GSM modules, and other serial devices.

---

## Features

### Core Features
- ✅ **Interrupt-driven communication** - Efficient, non-blocking data reception
- ✅ **Full duplex support** - Simultaneous send and receive capabilities
- ✅ **Built-in LED feedback** - Visual indication of communication activity
- ✅ **Flexible callbacks** - Custom handlers for commands and messages
- ✅ **Buffer management** - Automatic overflow protection
- ✅ **Protocol parsing** - Distinguishes between single-character commands and multi-character messages
- ✅ **Universal compatibility** - Works with any UART device (9600 baud default)

### Safety Features
- ✅ Buffer overflow protection
- ✅ Graceful error handling
- ✅ Interrupt context safety
- ✅ Memory-efficient design

---

## Files in This Module

### 1. `uart_handler.py`
**Core UART handler class for object-oriented UART communication.**

- Encapsulates all UART functionality in a clean class interface
- Automatic hardware initialization (UART, LED, interrupts)
- Interrupt-driven data reception with scheduled callbacks
- Distinguishes between single-character commands and multi-character messages
- Customizable via callback functions

**Use this when:** You need a reusable UART communication component for your project.

### 2. `uart_loopback_test.py`
**Hardware loopback test for UART verification.**

- Tests UART TX and RX functionality
- Requires a physical jumper wire between GP0 and GP1
- Verifies interrupt handling is working correctly
- Displays received data with formatting (printable/hex)

**Use this when:** You want to verify your UART hardware is working correctly before connecting external devices.

### 3. `uart_full_duplex_test.py`
**Full duplex communication test with external UART devices.**

- Demonstrates real-world UART communication
- Includes XBee reset sequence (optional, works with any UART device)
- Command parsing example (commands: 'a', 'b', 'c', 's', 'g')
- Continuous transmission and reception demonstration
- Custom callback implementations

**Use this when:** You want to test communication with external UART devices like XBee modules, GPS receivers, or GSM modules.

---

## Hardware Requirements

### Required
- Raspberry Pi Pico or Pico 2
- MicroPython firmware (v1.26.1 or later recommended)
- USB cable for programming and power

### For Loopback Test
- 1x Male-to-male jumper wire

### For Full Duplex Test
- UART-compatible device (XBee, GPS, GSM module, etc.)
- Jumper wires for connections
- External power supply (recommended for high-current devices)

---

## Wiring Diagrams

### Loopback Test Wiring
```
Raspberry Pi Pico
┌─────────────────┐
│                 │
│  GP0 (TX) ●─────┼──┐
│           │     │  │
│  GP1 (RX) ●─────┼──┘ (Jumper Wire)
│                 │
│  GND      ●     │
│                 │
└─────────────────┘
```

### Full Duplex Test Wiring (Universal UART Device)
```
Raspberry Pi Pico          UART Device
┌─────────────────┐        ┌──────────────┐
│                 │        │              │
│  GP0 (TX) ●─────┼───────→│ RX (Data In) │
│                 │        │              │
│  GP1 (RX) ●─────┼───────→│ TX (Data Out)│
│                 │        │              │
│  GP2      ●─────┼───────→│ RST (Reset)  │ (Optional, for XBee)
│                 │        │              │
│  GND      ●─────┼───────→│ GND          │ (CRITICAL)
│                 │        │              │
└─────────────────┘        └──────────────┘
                                    ↓
                           External Power Supply
                           (Recommended for XBee)
```

### Pin Mapping
| Pico Pin | GPIO | Function | Connection |
|----------|------|----------|------------|
| Pin 1    | GP0  | UART TX  | → Device RX |
| Pin 2    | GP1  | UART RX  | ← Device TX |
| Pin 4    | GP2  | Reset    | → XBee RST (optional) |
| Pin 3/38 | GND  | Ground   | ↔ Device GND |

---

## Installation

### 1. Install MicroPython Firmware
Download and flash MicroPython firmware to your Pico:
- Download: [MicroPython for Raspberry Pi Pico](https://micropython.org/download/RPI_PICO/)
- Use the `.uf2` file for easy installation
- Connect Pico while holding BOOTSEL button
- Copy `.uf2` file to the mounted drive

### 2. Upload Files to Pico
Using Thonny IDE or your preferred method:
```
1. Open Thonny IDE
2. Connect your Pico via USB
3. Upload the following files to the Pico:
   - uart_handler.py
   - uart_loopback_test.py (for testing)
   - uart_full_duplex_test.py (for testing)
```

### 3. Verify Installation
Run the loopback test to verify everything is working:
```python
# In Thonny, open uart_loopback_test.py and click Run
```

---

## Quick Start

### Example 1: Basic UART Handler Usage

```python
from uart_handler import UartHandler

# Create UART handler with default settings (9600 baud)
uart = UartHandler()

# Send data
uart.write(b"Hello, UART Device!\r\n")

# Data reception is handled automatically via interrupts
# Received data will be printed to console
```

### Example 2: Custom Command Handler

```python
from uart_handler import UartHandler

def my_command_handler(character_byte, uart_handler):
    """Handle single character commands"""
    command = chr(character_byte)
    print(f"Received command: {command}")
    
    if command == 'A':
        uart_handler.write(b"Command A acknowledged\r\n")
    elif command == 'B':
        uart_handler.write(b"Command B acknowledged\r\n")

def my_message_handler(message_string, uart_handler):
    """Handle complete messages"""
    print(f"Received message: {message_string}")
    uart_handler.write(f"Echo: {message_string}\r\n".encode())

# Create handler with custom callbacks
uart = UartHandler(
    uart_interface_id=0,
    uart_tx_pin=0,
    uart_rx_pin=1,
    uart_baudrate=9600,
    max_line_buffer_size=256,
    on_command_received=my_command_handler,
    on_message_received=my_message_handler
)
```

### Example 3: Running Loopback Test

```python
# Connect jumper wire between GP0 and GP1
# Run uart_loopback_test.py
# You should see:
# TX: Transmitted test message #0
# RX: 'A'
# RX: 'B'
# RX: 'C'
# RX: 0x0D
# RX: 0x0A
```

### Example 4: Full Duplex Communication

```python
# Connect UART device to GP0 (TX) and GP1 (RX)
# Run uart_full_duplex_test.py
# The Pico will continuously send messages
# Any data received from the device will be displayed
```

---

## API Documentation

### UartHandler Class

#### Constructor

```python
UartHandler(
    uart_interface_id=0,
    uart_tx_pin=0,
    uart_rx_pin=1,
    uart_baudrate=9600,
    max_line_buffer_size=256,
    on_command_received=None,
    on_message_received=None
)
```

**Parameters:**
- `uart_interface_id` (int): UART interface ID (0 or 1). Default: 0
- `uart_tx_pin` (int): GPIO pin for transmission. Default: 0 (GP0)
- `uart_rx_pin` (int): GPIO pin for reception. Default: 1 (GP1)
- `uart_baudrate` (int): Communication speed in bps. Default: 9600
- `max_line_buffer_size` (int): Maximum buffer size in bytes. Default: 256
- `on_command_received` (callable): Callback for single-character commands.
  - Signature: `callback(character_byte: int, uart_handler: UartHandler)`
- `on_message_received` (callable): Callback for multi-character messages.
  - Signature: `callback(message_string: str, uart_handler: UartHandler)`

#### Public Methods

##### `write(data: bytes) -> int`
Write data to the UART device.

```python
bytes_sent = uart.write(b"Hello\r\n")
print(f"Sent {bytes_sent} bytes")
```

**Returns:** Number of bytes written

##### `read() -> bytes or None`
Read available data from UART device.

```python
data = uart.read()
if data:
    print(f"Received: {data}")
```

**Returns:** Bytes object or None if no data available

##### `clear_receive_buffer()`
Clear the internal receive buffer.

```python
uart.clear_receive_buffer()
```

##### `get_receive_buffer() -> bytearray`
Get a copy of the current receive buffer.

```python
buffer = uart.get_receive_buffer()
print(f"Buffer contains: {buffer}")
```

**Returns:** Copy of receive buffer as bytearray

##### `get_buffer_size() -> int`
Get the current size of the receive buffer.

```python
size = uart.get_buffer_size()
print(f"Buffer size: {size} bytes")
```

**Returns:** Number of bytes in buffer

---

## Examples

### Example: GPS Module Integration

```python
from uart_handler import UartHandler
import utime

def parse_gps_data(message, uart_handler):
    """Parse NMEA sentences from GPS module"""
    if message.startswith("$GPGGA"):
        print(f"GPS Position: {message}")
    elif message.startswith("$GPRMC"):
        print(f"GPS Recommended Minimum: {message}")

# Initialize UART for GPS (typically 9600 baud)
gps_uart = UartHandler(
    uart_interface_id=0,
    uart_tx_pin=0,
    uart_rx_pin=1,
    uart_baudrate=9600,
    on_message_received=parse_gps_data
)

print("GPS module initialized. Waiting for data...")

# Main loop
while True:
    utime.sleep(1)
```

### Example: XBee Wireless Communication

```python
from uart_handler import UartHandler
from machine import Pin
import utime

# XBee reset sequence
def reset_xbee():
    reset_pin = Pin(2, Pin.OUT)
    reset_pin.value(1)
    utime.sleep(0.1)
    reset_pin.value(0)
    utime.sleep(0.1)
    reset_pin.value(1)
    utime.sleep(5)  # Wait for XBee to boot

# Initialize XBee
reset_xbee()

def handle_xbee_message(message, uart_handler):
    print(f"XBee received: {message}")
    # Echo back
    uart_handler.write(f"ACK: {message}\r\n".encode())

xbee = UartHandler(
    uart_baudrate=9600,
    on_message_received=handle_xbee_message
)

# Send periodic heartbeat
counter = 0
while True:
    xbee.write(f"Heartbeat #{counter}\r\n".encode())
    counter += 1
    utime.sleep(5)
```

---

## Power Supply Considerations

### CRITICAL: External Power for High-Current Devices

**DO NOT** power high-current devices (like XBee S2C modules) through the Pico's 3.3V pin!

#### Why?
- XBee and similar modules experience current spikes during:
  - Bootup and initialization
  - Network association
  - RF transmission
  - Data bursts
- These spikes can overwhelm the Pico's power supply
- Can cause voltage drops, brownouts, or damage

#### Recommended Setup:
1. **Use external power supply** (3.3V to 5V) for the UART device
2. **Connect grounds together** - Pico GND ↔ Device GND ↔ Power Supply GND
3. **Keep signal wires short** to minimize interference
4. **Add bypass capacitor** (10μF) near device power pins for stability

#### Power Supply Wiring:
```
External Power Supply (3.3V-5V)
       │
       ├─→ UART Device VCC
       └─→ UART Device GND ─→ Pico GND (Common Ground!)
```

---

## Troubleshooting

### No Data Received

**Symptoms:** TX works but no RX data appears

**Solutions:**
1. Verify jumper wire connection (loopback test)
2. Check baud rate matches between devices
3. Ensure common ground connection
4. Verify TX/RX aren't swapped (Pico TX → Device RX)
5. Check if device is powered properly

### Garbled or Corrupted Data

**Symptoms:** Received data shows wrong characters or hex values

**Solutions:**
1. Verify baud rate match (both devices must use same rate)
2. Check for loose connections
3. Add ground wire if missing
4. Reduce cable length
5. Check for electromagnetic interference

### Device Not Responding

**Symptoms:** Can send data but device doesn't respond

**Solutions:**
1. Verify device is powered on
2. Check if device requires initialization commands
3. For XBee: Verify reset sequence is working (GP2 connection)
4. Check device documentation for startup time requirements
5. Use loopback test to verify Pico UART is working

### Buffer Overflow

**Symptoms:** Data loss or incomplete messages

**Solutions:**
1. Increase `max_line_buffer_size` parameter
2. Reduce transmission rate
3. Ensure messages are terminated with `\r\n`
4. Process data faster in callback functions

### LED Not Blinking

**Symptoms:** No visual feedback during communication

**Solutions:**
1. LED control is normal - it only blinks briefly during interrupts
2. Verify Pico has built-in LED (some variants don't)
3. Check if data is actually being received
4. LED issue doesn't affect functionality

---

## Protocol Details

### Command vs Message Detection

The library automatically distinguishes between:

**Single Character Command:**
- Exactly 1 printable ASCII character (32-126)
- Followed by line ending (`\r` or `\n`)
- Triggers `on_command_received` callback
- Example: `'A\r\n'` → Command 'A'

**Multi-Character Message:**
- Multiple characters before line ending
- OR single non-printable character
- Triggers `on_message_received` callback
- Example: `'TEMP:25.3\r\n'` → Message "TEMP:25.3"

### Line Endings

Supported line endings:
- `\n` (Line Feed) - Unix/Linux style
- `\r` (Carriage Return) - Classic Mac style
- `\r\n` (CRLF) - Windows style (most common)

The library handles all three formats automatically.

---

## Performance Characteristics

- **Baud Rate:** 9600 bps default (configurable to 115200+ bps)
- **Latency:** < 1ms interrupt response time
- **Throughput:** ~960 bytes/second @ 9600 baud
- **Buffer Size:** 256 bytes default (configurable)
- **Memory Usage:** ~2KB RAM (including buffers)

---

## Best Practices

1. **Always connect ground** between Pico and UART device
2. **Use external power** for high-current devices
3. **Test with loopback** before connecting real devices
4. **Terminate messages** with `\r\n` for proper parsing
5. **Keep callbacks fast** - avoid long operations in interrupt handlers
6. **Check buffer size** - increase if receiving long messages
7. **Add error handling** in your callbacks
8. **Use descriptive variable names** following PEP 8 style

---

## Supported Baud Rates

Common baud rates (all supported):
- 9600 bps (default, most reliable)
- 19200 bps
- 38400 bps
- 57600 bps
- 115200 bps (high speed)
- 230400 bps (very high speed)

Choose based on your device's requirements and cable length.

---

## Compatibility

### Hardware
- ✅ Raspberry Pi Pico
- ✅ Raspberry Pi Pico W
- ✅ Raspberry Pi Pico 2

### MicroPython Versions
- ✅ v1.19.1 and later (tested)
- ✅ v1.26.1 (recommended)

### UART Devices
- ✅ XBee modules (all variants)
- ✅ GPS modules (Neo-6M, Neo-7M, etc.)
- ✅ GSM modules (SIM800L, SIM900, etc.)
- ✅ Bluetooth modules (HC-05, HC-06)
- ✅ Arduino and other microcontrollers
- ✅ Any device with UART/serial interface

---

## Author

**Ojas Jha**

Date: October 18, 2025

---

## Version History

- **v2.0** (October 18, 2025)
  - Complete rewrite with object-oriented design
  - Added `UartHandler` class for encapsulation
  - Improved interrupt handling
  - Added callback support for commands and messages
  - Enhanced documentation and examples
  - Added loopback and full duplex test utilities

---

## Contributing

Contributions are welcome! Please ensure:
- Code follows PEP 8 style guide
- Functions include docstrings
- Changes are tested on actual hardware
- Documentation is updated

---

## License

This project is licensed under the MIT License - see the [LICENSE](#mit-license) section above for details.

---

## Additional Resources

- [MicroPython Documentation](https://docs.micropython.org/)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/)
- [UART Protocol Overview](https://en.wikipedia.org/wiki/Universal_asynchronous_receiver-transmitter)
- [XBee Configuration Guide](https://www.digi.com/resources/documentation)

---

## Support

For issues, questions, or contributions, please refer to the project documentation or contact the author.

**Happy coding!** 🚀


