"""
GPS Testing & Diagnostics for Raspberry Pi Pico (MicroPython)

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

GPS Testing & Diagnostics for Raspberry Pi Pico (MicroPython)
=============================================================

What this script does
---------------------
- Opens a UART to read NMEA sentences from a GPS module (e.g., NEO-6M).
- Streams incoming bytes to `NEO6MV2Handler.update_from_char()` as they arrive.
- Every `PRINT_INTERVAL_MS`, prints either:
  * `display_comprehensive_gps_diagnostics()` — when a fix is available, or
  * `display_gps_acquisition_status()` — while waiting for a fix.

MicroPython & Pico (RP2040) quick notes
---------------------------------------
- UART on Pico is **3.3V logic**. Most NEO-6M boards are also 3.3V, but always
  check your breakout's specs. If your GPS TX is 5V, use a level shifter.
- Default NEO-6M serial: **9600 baud, 8N1**.
- Example wiring for UART1 (set below):
    - Pico **GP4**  → UART1 TX
    - Pico **GP5**  → UART1 RX
    - GPS **TX**    → Pico RX (GP5)
    - GPS **RX**    → Pico TX (GP4)
    - GPS **GND**   → Pico GND
    - GPS **VCC**   → Pico 3V3 (if supported by your breakout)

Sensor specifics (NEO-6M / NMEA)
--------------------------------
- Emits NMEA sentences like `$GPRMC,...*hh` ending with `\\r\\n`.
- Talker IDs: GP (GPS), GL (GLONASS), GN (combined).
- Parsers used: RMC, GGA, GSA, GSV, VTG, GLL.

Keep output readable
--------------------
- We use **non-blocking UART reads** (`uart.any()` / `uart.read(n)`).
- We print a snapshot only every `PRINT_INTERVAL_MS` to avoid spamming.

"""

# -----------------------------------------------------------------------------
# Categorized imports
# -----------------------------------------------------------------------------
# MicroPython HAL (hardware access): UART peripheral and pins
from machine import Pin, UART
# Timing utilities (sleep and millisecond ticks)
import utime
# GPS parser library (this repo)
from neo6mv2_handler import NEO6MV2Handler


# -----------------------------------------------------------------------------
# User configuration (adjust to your wiring / time zone)
# -----------------------------------------------------------------------------
UART_ID = 1                 # Pico has UART0 and UART1
UART_BAUDRATE = 9600        # NEO-6M default
UART_TX_PIN = 4             # We use GP4 as TX for UART1 in this example
UART_RX_PIN = 5             # We use GP5 as RX for UART1 in this example

TIMEZONE_OFFSET_HOURS = 4   # Dubai = UTC+4 (supports fractional hours like 5.5)
PRINT_INTERVAL_MS = 2000    # How often to print a status snapshot
UART_IDLE_SLEEP_MS = 10     # Small sleep to keep CPU usage low


# -----------------------------------------------------------------------------
# Diagnostic displays (intentionally verbose; helpful while learning/debugging)
# -----------------------------------------------------------------------------
def display_comprehensive_gps_diagnostics(gps_parser):
    """
    Print a comprehensive diagnostic snapshot.

    What it shows:
        - Position in multiple formats; altitude and geoid separation.
        - Local time/date (timezone applied).
        - Motion: speed in kph/mph/knots, course and compass direction.
        - Satellite visibility: PRNs and per-satellite (elev/az/SNR) if present.
        - Fix and DOP (hdop/vdop/pdop).
        - Parser statistics and internal state (helpful when learning NMEA).

    Args:
        gps_parser: NEO6MV2Handler instance to display diagnostics for

    Returns:
        None
    """
    print("\n" + "=" * 80)
    print("COMPREHENSIVE GPS DIAGNOSTIC & FEATURE TEST")
    print("=" * 80)

    previous_format = gps_parser.coordinate_format
    gps_parser.coordinate_format = "ddm"
    try:
        # -------- POSITION --------
        print("\n📍 POSITION DATA:")
        print("-" * 40)
        lat_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(gps_parser.latitude)
        lon_dd = NEO6MV2Handler.convert_coord_to_decimal_degrees_string(gps_parser.longitude)
        print("Latitude (decimal):      {}°".format(lat_dd))
        print("Longitude (decimal):     {}°".format(lon_dd))
        print("Altitude:                {} m".format(gps_parser.altitude_m))
        print("Geoid Height:            {} m".format(gps_parser.geoid_separation_m))

        # Coordinate format demonstrations
        print("\nCoordinate Format Tests:")
        gps_parser.coordinate_format = "dd"
        print(" - DD  latitude string:  {}".format(gps_parser.latitude_string()))
        print(" - DD  longitude string: {}".format(gps_parser.longitude_string()))
        gps_parser.coordinate_format = "dms"
        print(" - DMS latitude string:  {}".format(gps_parser.latitude_string()))
        print(" - DMS longitude string: {}".format(gps_parser.longitude_string()))
        gps_parser.coordinate_format = "ddm"
        print(" - DDM latitude string:  {}".format(gps_parser.latitude_string()))
        print(" - DDM longitude string: {}".format(gps_parser.longitude_string()))

        # -------- TIME/DATE --------
        print("\n🕐 TIME & DATE DATA:")
        print("-" * 40)
        h, m, s = gps_parser.time_hms_local
        print("GPS Time (local):        {:02d}:{:02d}:{:05.2f}".format(h, m, s))
        print("Raw local time tuple:    {}".format(gps_parser.time_hms_local))
        print("Raw date (D/M/Y2):       {}".format(gps_parser.date_dmy))
        print("Timezone Offset (h):     {}".format(gps_parser.timezone_offset_hours))
        try:
            print("Long Date:               {}".format(gps_parser.date_string("long")))
            print("Short MDY:               {}".format(gps_parser.date_string("s_mdy")))
            print("Short DMY:               {}".format(gps_parser.date_string("s_dmy")))
        except Exception as exc:
            print("Date formatting error:   {}".format(exc))

        # -------- MOTION --------
        print("\n🚗 MOTION DATA:")
        print("-" * 40)
        print("Course/Heading:          {}°".format(gps_parser.course_over_ground_deg))
        print("Compass Direction:       {}".format(gps_parser.course_compass_direction()))
        print("Raw Speed Array:         {}".format(gps_parser.speed_knots_mph_kph))
        print("Speed (km/h):            {}".format(gps_parser.speed_string("kph")))
        print("Speed (mph):             {}".format(gps_parser.speed_string("mph")))
        print("Speed (knots):           {}".format(gps_parser.speed_string("knot")))

        # -------- SATELLITES --------
        print("\n🛰️  SATELLITE DATA:")
        print("-" * 40)
        print("Satellites in View:      {}".format(gps_parser.satellites_in_view))
        print("Satellites in Use:       {}".format(gps_parser.satellites_used_for_fix))
        print("Satellites Used List:    {}".format(gps_parser.satellite_ids_used))
        print("Visible Satellite PRNs:  {}".format(gps_parser.visible_satellite_ids()))
        print("Satellite Data Complete: {}".format(gps_parser.is_satellite_data_complete()))
        print("Total GSV Sentences:     {}".format(gps_parser.gsv_total_sentences))
        print("Last GSV Sentence Index: {}".format(gps_parser.gsv_sentence_index))

        if gps_parser.satellites_info:
            print("\nDetailed Satellite Info:")
            for sat_id, (elev, az, snr) in gps_parser.satellites_info.items():
                elev_txt = "{}".format(elev) if elev is not None else "N/A"
                az_txt = "{}".format(az) if az is not None else "N/A"
                snr_txt = "{}".format(snr) if snr is not None else "N/A"
                print("  SAT {:>2}: Elev={}° Az={}° SNR={}dB".format(sat_id, elev_txt, az_txt, snr_txt))
        else:
            print("No detailed satellite data available")

        # -------- FIX / DOP --------
        print("\n📡 FIX QUALITY DATA:")
        print("-" * 40)
        print("Fix Status (GGA):        {}".format(gps_parser.fix_quality_gga))
        print("Fix Type  (GSA):         {}".format(gps_parser.fix_type_gsa))
        print("Valid Fix:               {}".format(gps_parser.has_fix))
        print("Time Since Fix:          {} ms".format(gps_parser.milliseconds_since_last_fix()))
        print("Fix Timestamp:           {}".format(gps_parser.last_fix_timestamp_ms))

        print("\n📊 DILUTION OF PRECISION:")
        print("-" * 40)
        print("HDOP:                    {}".format(gps_parser.hdop))
        print("VDOP:                    {}".format(gps_parser.vdop))
        print("PDOP:                    {}".format(gps_parser.pdop))

        # -------- PARSER STATS --------
        print("\n🧪 PARSER STATISTICS:")
        print("-" * 40)
        print("Valid Sentences:         {}".format(gps_parser.valid_sentence_count))
        print("Parsed Sentences:        {}".format(gps_parser.parsed_sentence_count))
        print("Checksum Failures:       {}".format(gps_parser.checksum_failures))
        print("Char Count (current):    {}".format(gps_parser.sentence_char_count))
        print("Last Fields Snapshot:    {}".format(gps_parser.nmea_fields))

        # -------- OBJECT STATE --------
        print("\n⚙️  OBJECT STATUS:")
        print("-" * 40)
        print("Sentence Active:         {}".format(gps_parser.nmea_sentence_active))
        print("Active Field Index:      {}".format(gps_parser.field_index))
        print("Collecting Checksum:     {}".format(gps_parser.collect_checksum))
        print("Checksum XOR Value:      {}".format(gps_parser.checksum_value))
        print("Coordinate Format:       {}".format(gps_parser.coordinate_format))
        print("Supported Sentences:     {}".format(list(gps_parser.SUPPORTED_SENTENCE_PARSERS.keys())))

    finally:
        gps_parser.coordinate_format = previous_format

    print("\n" + "=" * 80)
    print("DIAGNOSTIC COMPLETE - Waiting for next update...")
    print("=" * 80 + "\n")


def display_gps_acquisition_status(gps_parser):
    """
    Print a compact status line while waiting for a valid GPS fix.

    Args:
        gps_parser: NEO6MV2Handler instance to display status for

    Returns:
        None
    """
    print("\n🔍 GPS STATUS CHECK:")
    print("   Sentences received: {}".format(gps_parser.valid_sentence_count))
    print("   Sentences parsed:   {}".format(gps_parser.parsed_sentence_count))
    print("   CRC failures:       {}".format(gps_parser.checksum_failures))
    print("   Fix status (GGA):   {}".format(gps_parser.fix_quality_gga))
    print("   Valid fix:          {}".format(gps_parser.has_fix))
    if gps_parser.satellites_in_view > 0:
        print("   Satellites visible: {}".format(gps_parser.satellites_in_view))
    print("   Searching for fix...", end="")


# -----------------------------------------------------------------------------
# Main function: continuous UART read; periodic diagnostics
# -----------------------------------------------------------------------------
def main():
    """
    Main function that runs the GPS reader loop.

    This function:
    - Initializes the GPS reader with configuration
    - Continuously reads from UART and feeds data to GPS parser
    - Periodically displays GPS status or comprehensive diagnostics
    - Handles keyboard interrupt for graceful shutdown

    Args:
        None

    Returns:
        None
    """
    # Initialize runtime objects
    # Configure UART. If you wired different pins, update UART_TX_PIN / UART_RX_PIN.
    uart = UART(UART_ID, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))

    # Create the GPS parser. It does not read from UART itself; we feed it bytes.
    gps_parser = NEO6MV2Handler(timezone_offset_hours=TIMEZONE_OFFSET_HOURS)

    # Initialize LED for blinking
    led: Pin = Pin("LED", Pin.OUT)

    print(
        "GPS reader started (UART{}, baud={}, TX=GP{}, RX=GP{}, tz={}). Press Ctrl+C to stop."
        .format(UART_ID, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN, TIMEZONE_OFFSET_HOURS)
    )

    last_print_ms = utime.ticks_ms()
    fix_counter = 0

    try:
        while True:
            # Blink LED for every cycle
            led.toggle()
            utime.sleep(1)    # Keep LED on for 1 s
            led.toggle()
            # Non-blocking UART read:
            # - `uart.any()` tells how many bytes are waiting.
            # - `uart.read(n)` retrieves that many bytes without blocking.
            available = uart.any()
            if available:
                buf = uart.read(available)
                if buf:
                    # Convert each byte to a single-char string and feed the parser.
                    for byte_val in buf:
                        gps_parser.update_from_char(chr(byte_val))

            # Print either acquisition status or comprehensive diagnostics periodically
            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
                last_print_ms = now_ms
                if gps_parser.has_fix:
                    fix_counter += 1
                    print("\n🎯 FIX #{} ACQUIRED".format(fix_counter))
                    display_comprehensive_gps_diagnostics(gps_parser)
                else:
                    display_gps_acquisition_status(gps_parser)
                    print("")  # newline to end the line nicely

            # Small sleep keeps the loop responsive without busy-spinning the CPU
            utime.sleep_ms(UART_IDLE_SLEEP_MS)

    except KeyboardInterrupt:
        print("\n\n🛑 Program terminated by user")
        print("✅ GPS reader stopped gracefully")

    except Exception as e:
        print(f"\n❌ Unexpected error occurred: {e}")
        print("🔧 Check GPS module connections and try again")


# -----------------------------------------------------------------------------
# Script entry point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    main()
