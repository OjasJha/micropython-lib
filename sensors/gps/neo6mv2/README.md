# NEO-6M GPS Module - MicroPython Driver

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

## Overview

A comprehensive NMEA 0183 GPS sentence parser library for MicroPython and CPython, specifically designed for the u-blox NEO-6M GPS module and compatible GPS receivers. This library provides efficient, streaming character-by-character parsing optimized for microcontrollers with limited memory.

### Key Features

- **Low Memory Footprint**: Character-by-character streaming parser designed for microcontrollers
- **Robust Parsing**: NMEA checksum validation to ensure data integrity
- **Comprehensive Sentence Support**: Parses RMC, GGA, GSA, GSV, VTG, and GLL sentences
- **Multi-Constellation Support**: Handles GP (GPS), GL (GLONASS), and GN (combined) talker IDs
- **Flexible Coordinate Formats**: Supports DDM, DD, and DMS coordinate representations
- **Timezone Support**: Configurable timezone offset with fractional hour support
- **Satellite Tracking**: Detailed satellite visibility and signal strength information
- **Optional Data Logging**: Built-in NMEA data logging to SD card or filesystem

---

## Hardware Requirements

### GPS Module
- **u-blox NEO-6M** GPS module (or compatible NMEA 0183 GPS receiver)
- Operating voltage: 3.3V - 5V (check your specific module)
- Communication: UART serial (9600 baud default, 8N1)

### Microcontroller
- **Raspberry Pi Pico / Pico W / Pico 2 W** (recommended)
- Any MicroPython-compatible board with UART peripheral
- Minimum 64KB RAM recommended

### Additional Components
- Antenna (usually included with GPS module)
- Clear view of the sky for GPS signal acquisition

---

## Wiring Diagram

### Connection to Raspberry Pi Pico

```
NEO-6M GPS Module    →    Raspberry Pi Pico
─────────────────────────────────────────────
VCC (Power)          →    3V3 (Pin 36) or VBUS (5V, Pin 40)
GND (Ground)         →    GND (any GND pin)
TX (Transmit)        →    GP5 (UART1 RX, Pin 7)
RX (Receive)         →    GP4 (UART1 TX, Pin 6) *optional*
```

**Notes:**
- GPS TX → Pico RX (GPS transmits data to Pico)
- GPS RX is optional (only needed for GPS configuration commands)
- Most NEO-6M modules are 3.3V logic compatible
- Ensure proper voltage level (3.3V) - use level shifter if GPS is 5V logic

### Pin Configuration in Code

Default configuration in `neo6mv2_test.py`:
```python
UART_ID = 1           # UART1 peripheral
UART_BAUDRATE = 9600  # Default NEO-6M baud rate
UART_TX_PIN = 4       # Pico GP4 (UART1 TX)
UART_RX_PIN = 5       # Pico GP5 (UART1 RX)
```

---

## Installation

1. Copy files to your MicroPython device:
   ```
   libs/sensors/gps/neo6mv2/
   ├── neo6mv2_handler.py    # GPS parser library
   └── neo6mv2_test.py       # Test/example script
   ```

2. Ensure proper file placement on your microcontroller's filesystem

3. Import the library in your code:
   ```python
   from neo6mv2_handler import NEO6MV2Handler
   ```

---

## Quick Start Guide

### Basic Usage Example

```python
from machine import Pin, UART
import utime
from neo6mv2_handler import NEO6MV2Handler

# Initialize UART
uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

# Create GPS parser with timezone offset (e.g., +4 for Dubai)
gps = NEO6MV2Handler(timezone_offset_hours=4, coordinate_format="dd")

print("Waiting for GPS fix...")

while True:
    # Read available data from UART
    if uart.any():
        data = uart.read(uart.any())
        
        # Feed each character to the parser
        for byte in data:
            sentence_type = gps.update_from_char(chr(byte))
            
            # When a complete sentence is parsed
            if sentence_type and gps.has_fix:
                print(f"\nLatitude:  {gps.latitude_string()}")
                print(f"Longitude: {gps.longitude_string()}")
                print(f"Altitude:  {gps.altitude_m} m")
                print(f"Speed:     {gps.speed_string('kph')}")
                print(f"Satellites: {gps.satellites_used_for_fix}")
                print(f"Time:      {gps.time_hms_local}")
    
    utime.sleep_ms(100)
```

### Running the Test Script

The included test script provides comprehensive diagnostics:

```python
# Upload neo6mv2_test.py to your Pico
# Run it to see detailed GPS information

# Output includes:
# - Position (multiple coordinate formats)
# - Time and date (with timezone)
# - Speed and course
# - Satellite information
# - Fix quality and DOP values
# - Parser statistics
```

---

## API Reference

### Class: NEO6MV2Handler

#### Constructor

```python
NEO6MV2Handler(timezone_offset_hours=0.0, coordinate_format="ddm")
```

**Parameters:**
- `timezone_offset_hours` (float): Hours offset from UTC (supports fractional hours)
  - Examples: `4` (Dubai/UAE), `5.5` (India), `-5` (US Eastern)
- `coordinate_format` (str): Coordinate display format
  - `"ddm"`: Degrees and Decimal Minutes (default)
  - `"dd"`: Decimal Degrees
  - `"dms"`: Degrees, Minutes, Seconds

#### Main Methods

##### update_from_char(ch)
Feed a single character to the parser.

```python
sentence_type = gps.update_from_char(chr(byte))
```

**Parameters:**
- `ch` (str): Single character from UART stream

**Returns:**
- `str|None`: Sentence type (e.g., "GPRMC") if complete valid sentence parsed, else None

##### Coordinate Conversion

```python
# Get formatted coordinate strings
lat_str = gps.latitude_string()   # e.g., "25.252058° N"
lon_str = gps.longitude_string()  # e.g., "55.309463° E"

# Convert to decimal degrees
lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(gps.latitude)
lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(gps.longitude)
```

##### Speed and Course

```python
speed_kph = gps.speed_string("kph")   # "22.8 km/h"
speed_mph = gps.speed_string("mph")   # "14.2 mph"
speed_knot = gps.speed_string("knot") # "12.3 knots"

course = gps.course_over_ground_deg   # 0-360 degrees
compass = gps.course_compass_direction()  # "NE", "SSW", etc.
```

##### Date and Time

```python
# Time (with timezone applied)
h, m, s = gps.time_hms_local  # [14, 30, 45.5]

# Date formatting
date_long = gps.date_string("long")   # "October 19th, 2025"
date_mdy = gps.date_string("s_mdy")   # "10/19/25"
date_dmy = gps.date_string("s_dmy")   # "19/10/25"
```

##### Satellite Information

```python
# Satellite counts
sats_visible = gps.satellites_in_view
sats_used = gps.satellites_used_for_fix
sat_ids = gps.satellite_ids_used  # List of PRN IDs

# Detailed satellite info
for sat_id, (elevation, azimuth, snr) in gps.satellites_info.items():
    print(f"Sat {sat_id}: Elev={elevation}° Az={azimuth}° SNR={snr}dB")

# Check if satellite data is complete
if gps.is_satellite_data_complete():
    print("All GSV sentences received")
```

##### Fix Quality

```python
# Fix status
has_fix = gps.has_fix  # Boolean
fix_quality = gps.fix_quality_gga  # 0=no fix, 1=GPS, 2=DGPS
fix_type = gps.fix_type_gsa  # 1=no fix, 2=2D, 3=3D

# Dilution of Precision
hdop = gps.hdop  # Horizontal DOP
vdop = gps.vdop  # Vertical DOP
pdop = gps.pdop  # Position DOP

# Time since last fix
ms_since_fix = gps.milliseconds_since_last_fix()
```

##### Data Logging (Optional)

```python
# Start logging raw NMEA data
gps.start_logging("/sd/gps_log.txt", mode="append")  # or mode="new"

# Logging happens automatically during parsing

# Stop logging
gps.stop_logging()
```

#### Public Attributes

**Position & Motion:**
- `latitude` - Coordinate in configured format (property)
- `longitude` - Coordinate in configured format (property)
- `raw_latitude_ddm` - Raw latitude [deg, dec_min, hemisphere]
- `raw_longitude_ddm` - Raw longitude [deg, dec_min, hemisphere]
- `altitude_m` - Altitude in meters (float)
- `geoid_separation_m` - Geoid separation in meters (float)
- `speed_knots_mph_kph` - Speed [knots, mph, kph] (list)
- `course_over_ground_deg` - Course 0-360° (float)

**Time & Date:**
- `time_hms_local` - Local time [hour, minute, second] (list)
- `date_dmy` - Date [day, month, year_2digit] (list)
- `timezone_offset_hours` - Configured timezone offset (float)

**Satellites & Fix:**
- `satellites_in_view` - Number of visible satellites (int)
- `satellites_used_for_fix` - Number of satellites used (int)
- `satellite_ids_used` - List of PRN IDs in use (list)
- `satellites_info` - Dict: {PRN: (elevation, azimuth, SNR)}
- `has_fix` - Boolean fix status
- `fix_quality_gga` - GGA fix quality indicator (int)
- `fix_type_gsa` - GSA fix type (1/2/3) (int)
- `hdop` - Horizontal DOP (float)
- `vdop` - Vertical DOP (float)
- `pdop` - Position DOP (float)

**Parser Statistics:**
- `valid_sentence_count` - Successfully parsed sentences (int)
- `parsed_sentence_count` - Sentences dispatched to handlers (int)
- `checksum_failures` - Failed checksum validations (int)

---

## Supported NMEA Sentences

| Sentence | Description | Data Provided |
|----------|-------------|---------------|
| **RMC** | Recommended Minimum Navigation | Time, date, position, speed, course, fix status |
| **GGA** | GPS Fix Data | Time, position, altitude, fix quality, HDOP, satellites used |
| **GSA** | DOP and Active Satellites | Fix type, satellite PRNs, HDOP, VDOP, PDOP |
| **GSV** | Satellites in View | Satellite visibility, elevation, azimuth, SNR |
| **VTG** | Course Over Ground and Speed | True/magnetic course, speed in knots/kph |
| **GLL** | Geographic Position | Time, position, fix status |

**Talker IDs Supported:**
- `GP` - GPS only
- `GL` - GLONASS only
- `GN` - Combined GPS/GLONASS/other

---

## Coordinate Format Examples

### DDM (Degrees and Decimal Minutes)
```python
gps = NEO6MV2Handler(coordinate_format="ddm")
# Output: "25° 15.1234' N" and "55° 18.5678' E"
```

### DD (Decimal Degrees)
```python
gps = NEO6MV2Handler(coordinate_format="dd")
# Output: "25.252057° N" and "55.309463° E"
```

### DMS (Degrees, Minutes, Seconds)
```python
gps = NEO6MV2Handler(coordinate_format="dms")
# Output: "25° 15' 7\" N" and "55° 18' 34\" E"
```

---

## Troubleshooting

### No Data Received

**Symptoms:** Parser receives no data, no sentences counted

**Solutions:**
1. Verify wiring connections (especially TX/RX)
2. Check UART configuration (correct pins, baud rate)
3. Ensure GPS module has power (LED should blink)
4. Verify UART pins are correct for your board

```python
# Debug: Print raw UART data
if uart.any():
    data = uart.read(uart.any())
    print(data)  # Should see NMEA sentences like b'$GPRMC,...'
```

### No GPS Fix

**Symptoms:** `has_fix` remains False, position is 0

**Solutions:**
1. **Move outdoors** - GPS requires clear sky view
2. Wait 30-60 seconds for "cold start" (first power-on)
3. Check antenna connection
4. Avoid indoor use, metal structures, or dense tree cover
5. Check satellite visibility: `gps.satellites_in_view`

**Typical acquisition time:**
- Cold start (first time): 30-60 seconds
- Warm start (recent fix): 5-30 seconds
- Hot start (recent + same location): 1-5 seconds

### Checksum Failures

**Symptoms:** High `checksum_failures` count

**Solutions:**
1. Check for loose wiring or poor connections
2. Reduce cable length between GPS and microcontroller
3. Add decoupling capacitor near GPS power pins
4. Check for electromagnetic interference sources
5. Ensure proper grounding

```python
# Monitor checksum failures
print(f"Valid: {gps.valid_sentence_count}, Failed: {gps.checksum_failures}")
# Failure rate should be < 1%
```

### Wrong Time/Date

**Symptoms:** Time is incorrect

**Solutions:**
1. Set correct `timezone_offset_hours` in constructor
2. GPS time is always UTC - timezone only applied locally
3. Date comes from satellites - ensure fix is established
4. Time is only valid when `has_fix` is True

```python
# Example for different timezones
gps_dubai = NEO6MV2Handler(timezone_offset_hours=4)    # UTC+4
gps_india = NEO6MV2Handler(timezone_offset_hours=5.5)  # UTC+5:30
gps_us_east = NEO6MV2Handler(timezone_offset_hours=-5) # UTC-5
```

### Inaccurate Position

**Symptoms:** Position jumps around or is incorrect

**Solutions:**
1. Wait for more satellites (aim for 6+ used satellites)
2. Check HDOP value (lower is better, < 2 is excellent)
3. Ensure clear sky view (buildings/trees block signals)
4. GPS accuracy is typically 2-5 meters in open sky
5. Consider using DGPS or RTK for better accuracy

```python
# Check fix quality
if gps.hdop < 2.0 and gps.satellites_used_for_fix >= 6:
    print("High quality fix!")
```

---

## Performance Characteristics

### Memory Usage
- **Parser Object**: ~2-3 KB
- **Per-sentence Buffer**: ~90 bytes max
- **Satellite Info**: ~20 bytes per satellite
- **Total RAM**: < 5 KB typical

### Processing Speed
- **Character processing**: < 10 μs per character
- **Sentence parsing**: < 1 ms per sentence
- **Update rate**: Handles 1-10 Hz GPS update rates easily

### GPS Accuracy
- **Position**: 2.5m CEP (typical, open sky)
- **Altitude**: ±5m typical
- **Speed**: 0.1 m/s typical
- **Time**: ±60 ns with valid fix

---

## Advanced Usage

### Integration with SD Card Logging

```python
from neo6mv2_handler import NEO6MV2Handler
from sdcard_handler import SDCardHandler

# Initialize SD card
sd = SDCardHandler(spi_id=0, cs_pin=17, sck_pin=18, mosi_pin=19, miso_pin=16)
sd.mount()

# Start GPS logging
gps = NEO6MV2Handler(timezone_offset_hours=4)
gps.start_logging("/sd/gps_track.nmea", mode="new")

# Data is logged automatically as sentences arrive
# ...parsing loop...

gps.stop_logging()
sd.unmount()
```

### Filtering by Fix Quality

```python
# Only use high-quality fixes
if (gps.has_fix and 
    gps.fix_type_gsa == gps.FIX_TYPE_3D and
    gps.hdop < 2.0 and
    gps.satellites_used_for_fix >= 6):
    
    # Use GPS data for navigation
    log_position(gps.latitude, gps.longitude)
```

### Combining with Other Sensors

```python
# GPS + IMU for dead reckoning
from neo6mv2_handler import NEO6MV2Handler
from mpu9250_handler import MPU9250Handler

gps = NEO6MV2Handler(timezone_offset_hours=4)
imu = MPU9250Handler(i2c_id=0)

# When GPS fix is lost, use IMU for position estimation
if gps.has_fix:
    last_known_position = (gps.latitude, gps.longitude)
    last_known_heading = gps.course_over_ground_deg
else:
    # Use IMU to estimate position change
    current_heading = imu.get_heading()
    # ... dead reckoning algorithm ...
```

---

## Technical Details

### NMEA Checksum Validation

The library computes XOR checksums over all characters between `$` and `*`:

```python
checksum = 0
for char in sentence_between_dollar_and_star:
    checksum ^= ord(char)
# Compare with hex value after '*'
```

### Streaming Parser Design

Characters are processed one at a time to minimize memory usage:

1. `$` starts new sentence
2. Characters accumulate in current field
3. `,` separates fields
4. `*` ends data, next 2 chars are checksum
5. `\r\n` ignored
6. Checksum validated before dispatching

### Coordinate Conversion

**DDM to DD conversion:**
```
decimal_degrees = degrees + (decimal_minutes / 60.0)
```

**DD to DMS conversion:**
```
degrees = int(decimal_degrees)
minutes = int((decimal_degrees - degrees) * 60)
seconds = ((decimal_degrees - degrees) * 60 - minutes) * 60
```

---

## Examples

### Example 1: Simple Position Logger

```python
from machine import Pin, UART
import utime
from neo6mv2_handler import NEO6MV2Handler

uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
gps = NEO6MV2Handler(timezone_offset_hours=4, coordinate_format="dd")

log_file = open("positions.csv", "w")
log_file.write("Time,Latitude,Longitude,Altitude,Speed\n")

while True:
    if uart.any():
        for byte in uart.read(uart.any()):
            if gps.update_from_char(chr(byte)) and gps.has_fix:
                lat = gps.latitude[0]  # Decimal degrees
                lon = gps.longitude[0]
                alt = gps.altitude_m
                speed = gps.speed_knots_mph_kph[2]  # kph
                time_str = f"{gps.time_hms_local[0]:02d}:{gps.time_hms_local[1]:02d}"
                
                log_file.write(f"{time_str},{lat},{lon},{alt},{speed}\n")
                log_file.flush()
    
    utime.sleep_ms(1000)
```

### Example 2: Geofencing Alert

```python
from neo6mv2_handler import NEO6MV2Handler
import utime

def point_in_circle(lat, lon, center_lat, center_lon, radius_m):
    """Check if point is within circular geofence"""
    # Simplified distance calculation (accurate for small distances)
    dlat = (lat - center_lat) * 111320  # meters per degree latitude
    dlon = (lon - center_lon) * 111320 * cos(radians(center_lat))
    distance = (dlat**2 + dlon**2)**0.5
    return distance <= radius_m

# Define geofence (e.g., home location)
HOME_LAT = 25.2048
HOME_LON = 55.2708
HOME_RADIUS = 100  # meters

gps = NEO6MV2Handler(timezone_offset_hours=4, coordinate_format="dd")

while True:
    # ... UART reading loop ...
    
    if gps.has_fix:
        lat = gps.latitude[0]
        lon = gps.longitude[0]
        
        if point_in_circle(lat, lon, HOME_LAT, HOME_LON, HOME_RADIUS):
            print("Inside geofence")
        else:
            print("⚠ Outside geofence!")
    
    utime.sleep(5)
```

### Example 3: Speed Alarm

```python
from neo6mv2_handler import NEO6MV2Handler

SPEED_LIMIT_KPH = 80

gps = NEO6MV2Handler(timezone_offset_hours=4)

while True:
    # ... UART reading loop ...
    
    if gps.has_fix:
        current_speed = gps.speed_knots_mph_kph[2]  # kph
        
        if current_speed > SPEED_LIMIT_KPH:
            print(f"⚠ SPEEDING! {current_speed:.1f} km/h (limit: {SPEED_LIMIT_KPH})")
            # Trigger alarm, log event, etc.
```

---

## Version History

### Version 1.0 (October 19, 2025)
- Initial release
- Complete NMEA 0183 parser implementation
- Support for RMC, GGA, GSA, GSV, VTG, GLL sentences
- Multiple coordinate format support
- Timezone handling
- Satellite tracking
- Optional data logging
- Comprehensive test suite

---

## License

This project is licensed under the MIT License. See the license text at the top of this file for details.

---

## Author

**Ojas Jha**
- Date: October 19, 2025

---

## Contributing

Contributions, issues, and feature requests are welcome!

---

## References

- [NMEA 0183 Protocol Specification](https://www.nmea.org/)
- [u-blox NEO-6 Documentation](https://www.u-blox.com/en/product/neo-6-series)
- [MicroPython Documentation](https://docs.micropython.org/)
- [Raspberry Pi Pico Datasheet](https://www.raspberrypi.com/documentation/microcontrollers/)

---

## Support

For issues, questions, or suggestions:
1. Check the Troubleshooting section above
2. Review the API Reference for proper usage
3. Examine the test script (`neo6mv2_test.py`) for examples
4. Verify hardware connections and GPS signal conditions

**Happy GPS tracking! 🛰️📍**

