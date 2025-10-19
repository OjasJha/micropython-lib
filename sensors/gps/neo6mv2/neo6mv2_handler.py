# neo6mv2_handler.py
"""
NMEA 0183 GPS Sentence Parser for MicroPython & CPython

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

MicroPyGPS — NMEA 0183 GPS sentence parser for MicroPython & CPython.

Overview
--------
This module parses the most common NMEA sentences produced by GPS receivers
(e.g., the u-blox NEO-6M). It's designed for streaming character input coming
from a UART (serial) connection on a microcontroller such as the Raspberry Pi
Pico. You feed bytes one by one via `update_from_char()`. When a complete,
checksum-valid sentence arrives, the parser updates its public attributes.

Why this design?
- **Low memory / streaming**: Microcontrollers have limited RAM, so we parse
  one character at a time instead of buffering large strings.
- **Robustness**: We compute and validate the NMEA checksum (XOR over the bytes
  between `$` and `*`) to ignore noise or half sentences.
- **Clarity**: Public attributes have descriptive names; docstrings explain each
  component.

MicroPython / Pico basics (quick primer)
----------------------------------------
- **MicroPython** is a compact implementation of Python for microcontrollers.
  The Raspberry Pi Pico (RP2040) exposes UART peripherals you can read from.
- NEO-6M defaults to **9600 baud, 8 data bits, no parity, 1 stop bit (8N1)**.
- Hardware wiring (typical for 3.3V boards):
    - GPS **TX → Pico RX** (UART RX pin)
    - GPS **RX → Pico TX** (UART TX pin)
    - GPS **GND → Pico GND**
    - GPS **VCC → Pico 3V3** (check your breakout's specs; most are 3.3V logic)
- GPS emits **NMEA text** like: `$GPRMC,...*hh\r\n`

Key data attributes (examples)
------------------------------
- `time_hms_local`: list[int, int, float]
    Example: `[14, 3, 27.50]`  →  2:03:27.50 PM (local, after timezone applied)
- `date_dmy`: list[int, int, int]
    Example: `[26, 9, 25]`     →  26/09/2025  (day / month / two-digit year)
- `raw_latitude_ddm`: list[int, float, str]  (DDM format)
    Example: `[25, 15.1234, 'N']`  →  25° 15.1234' N
- `raw_longitude_ddm`: list[int, float, str] (DDM format)
    Example: `[55, 18.5678, 'E']`  →  55° 18.5678' E
- `speed_knots_mph_kph`: list[float, float, float]
    Example: `[12.3, 14.15, 22.78]`  →  knots / mph / km/h
- `satellites_info`: dict[int, tuple(elev:int|None, az:int|None, snr:int|None)]
    Example: `{ 3: (60, 120, 35), 8: (42, 310, 28) }`

Sentence coverage
-----------------
- RMC, GGA, GSA, GSV, VTG, GLL with talker IDs GP/GL/GN (e.g., GPRMC, GNGGA).

"""

# -----------------------------------------------------------------------------
# Categorized imports
# -----------------------------------------------------------------------------
# Standard math utilities for coordinate conversions and angle bucketing
from math import floor, modf

# MicroPython time compatibility:
# - On microcontrollers use `utime`, on PCs use `time`. We select whichever is
#   available so the code runs in both environments.
try:
    import utime as time_module  # MicroPython
except ImportError:              # CPython fallback
    import time as time_module


class NEO6MV2Handler:
    """
    NMEA sentence parser with a streaming (char-by-char) API.

    How parsing works (high level)
    ------------------------------
    1) You call `update_from_char()` for each incoming character from UART.
    2) When a '$' arrives, we start a new sentence and begin XOR'ing bytes for
       checksum verification.
    3) We split fields on commas, stop checksum collection at '*', then read the
       2 hex checksum digits.
    4) If the computed XOR matches the supplied checksum, we dispatch to the
       specific sentence parser (e.g., RMC) and update public attributes.

    Public attributes (selected)
    ----------------------------
    - time_hms_local: [H, M, S.s] local time with timezone applied.
    - date_dmy: [D, M, YY] date from RMC.
    - raw_latitude_ddm / raw_longitude_ddm: position in DDM + hemisphere.
    - speed_knots_mph_kph: [knots, mph, kph]
    - course_over_ground_deg: float (0..360)
    - altitude_m, geoid_separation_m: floats
    - satellites_in_view / satellites_used_for_fix: ints
    - satellite_ids_used: list[int] from GSA
    - satellites_info: PRN→(elev, az, snr) from GSV
    - hdop / pdop / vdop: floats
    - has_fix: bool, fix_quality_gga: int, fix_type_gsa: int
    """

    # -------------------------------------------------------------------------
    # Constants (NMEA sizes and lookups)
    # -------------------------------------------------------------------------
    MAX_NMEA_SENTENCE_LENGTH = 90  # conservative cap to avoid runaway lines

    VALID_HEMISPHERES = ("N", "S", "E", "W")

    FIX_TYPE_NO_FIX = 1
    FIX_TYPE_2D = 2
    FIX_TYPE_3D = 3

    COMPASS_DIRECTIONS_16 = (
        "N", "NNE", "NE", "ENE",
        "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW",
        "W", "WNW", "NW", "NNW",
    )

    MONTH_NAMES = (
        "January", "February", "March", "April", "May", "June",
        "July", "August", "September", "October", "November", "December",
    )

    # -------------------------------------------------------------------------
    # Construction / state
    # -------------------------------------------------------------------------
    def __init__(self, timezone_offset_hours=0.0, coordinate_format="ddm"):
        """
        Initialize the parser.

        Args:
            timezone_offset_hours (float): Hours offset from UTC. Supports
                fractional hours, e.g., +5.5 for India, +4 for Dubai.
            coordinate_format (str): One of {'ddm', 'dd', 'dms'} controlling
                how latitude/longitude are reported in the string helpers.

        Returns:
            None
        """
        # --- streaming state ---
        self.nmea_sentence_active = False
        self.field_index = 0
        self.collect_checksum = False
        self.nmea_fields = []
        self.checksum_value = 0
        self.sentence_char_count = 0
        self.last_fix_timestamp_ms = 0

        # --- statistics ---
        self.checksum_failures = 0
        self.valid_sentence_count = 0
        self.parsed_sentence_count = 0

        # --- optional raw logging ---
        self._log_file = None
        self.logging_enabled = False

        # --- time & date (local time stored; date taken from RMC as provided) ---
        self.time_hms_local = [0, 0, 0.0]   # [H, M, S.ss] (after timezone applied)
        self.date_dmy = [0, 0, 0]           # [D, M, YY]
        self.timezone_offset_hours = timezone_offset_hours

        # --- position & motion ---
        self.raw_latitude_ddm = [0, 0.0, "N"]     # [deg, dec_min, hemi]
        self.raw_longitude_ddm = [0, 0.0, "W"]    # [deg, dec_min, hemi]
        self.coordinate_format = coordinate_format
        self.speed_knots_mph_kph = [0.0, 0.0, 0.0]  # [knots, mph, kph]
        self.course_over_ground_deg = 0.0
        self.altitude_m = 0.0
        self.geoid_separation_m = 0.0

        # --- satellites / DOP / fix ---
        self.satellites_in_view = 0
        self.satellites_used_for_fix = 0
        self.satellite_ids_used = []
        self.gsv_sentence_index = 0
        self.gsv_total_sentences = 0
        self.satellites_info = {}

        self.hdop = 0.0
        self.pdop = 0.0
        self.vdop = 0.0

        self.has_fix = False
        self.fix_quality_gga = 0
        self.fix_type_gsa = 1  # 1=No fix, 2=2D, 3=3D

    # -------------------------------------------------------------------------
    # Coordinate properties (presentation layer)
    # -------------------------------------------------------------------------
    @property
    def latitude(self):
        """
        Latitude in the configured display format.

        Returns:
            list: One of
                - DDM: [deg:int, dec_min:float, hemi:str]
                - DD:  [decimal_degrees:float, hemi:str]
                - DMS: [deg:int, min:int, sec:int, hemi:str]
        """
        if self.coordinate_format == "dd":
            decimal_degrees = self.raw_latitude_ddm[0] + (self.raw_latitude_ddm[1] / 60.0)
            return [decimal_degrees, self.raw_latitude_ddm[2]]
        if self.coordinate_format == "dms":
            fractional, minutes = modf(self.raw_latitude_ddm[1])
            seconds = round(fractional * 60)
            return [self.raw_latitude_ddm[0], int(minutes), seconds, self.raw_latitude_ddm[2]]
        return self.raw_latitude_ddm  # ddm

    @property
    def longitude(self):
        """
        Longitude in the configured display format.

        Returns:
            list: One of
                - DDM: [deg:int, dec_min:float, hemi:str]
                - DD:  [decimal_degrees:float, hemi:str]
                - DMS: [deg:int, min:int, sec:int, hemi:str]
        """
        if self.coordinate_format == "dd":
            decimal_degrees = self.raw_longitude_ddm[0] + (self.raw_longitude_ddm[1] / 60.0)
            return [decimal_degrees, self.raw_longitude_ddm[2]]
        if self.coordinate_format == "dms":
            fractional, minutes = modf(self.raw_longitude_ddm[1])
            seconds = round(fractional * 60)
            return [self.raw_longitude_ddm[0], int(minutes), seconds, self.raw_longitude_ddm[2]]
        return self.raw_longitude_ddm  # ddm

    # -------------------------------------------------------------------------
    # Optional raw logging (useful for debugging / collecting NMEA logs)
    # -------------------------------------------------------------------------
    def start_logging(self, file_path, mode="append"):
        """
        Open a file for writing raw NMEA data.

        Args:
            file_path (str): Path on the filesystem (e.g., '/sd/gps.log').
            mode (str): 'append' to append or 'new' to overwrite.

        Returns:
            bool: True if logging was enabled, False otherwise.
        """
        file_mode = "w" if mode == "new" else "a"
        try:
            self._log_file = open(file_path, file_mode)
        except (OSError, AttributeError):
            print("Error: Could not open log file")
            return False
        self.logging_enabled = True
        return True

    def stop_logging(self):
        """
        Close the log file if open.

        Returns:
            bool: True on success, False if no valid handle existed.
        """
        try:
            self._log_file.close()
        except AttributeError:
            print("Error: No valid log file handle to close")
            return False
        self.logging_enabled = False
        return True

    def log_write(self, text):
        """
        Write text to the log file (if enabled).

        Args:
            text (str): Line to write (include '\\n' if you want a newline).

        Returns:
            bool: True on success, False if logging disabled or write failed.
        """
        try:
            self._log_file.write(text)
        except (TypeError, AttributeError):
            return False
        return True

    # -------------------------------------------------------------------------
    # Timezone helper
    # -------------------------------------------------------------------------
    def _apply_timezone_offset(self, hour, minute, second):
        """
        Apply timezone offset (supports fractional hours) to hh:mm:ss.

        Args:
            hour (int): Hours (0..23) from NMEA.
            minute (int): Minutes (0..59) from NMEA.
            second (float): Seconds (0..59.999) from NMEA.

        Returns:
            list[int,int,float]: [hour, minute, second] adjusted to local time.
        """
        total_seconds = int(hour) * 3600 + int(minute) * 60 + float(second)
        # shift by whole minutes (handles .5, .75 hour offsets nicely)
        offset_seconds = int(round(self.timezone_offset_hours * 60)) * 60
        adjusted = (total_seconds + offset_seconds) % 86400
        new_hour = int(adjusted // 3600)
        new_minute = int((adjusted % 3600) // 60)
        new_second = float(adjusted % 60)
        return [new_hour, new_minute, new_second]

    # -------------------------------------------------------------------------
    # Individual sentence parsers (update class state from a valid NMEA sentence)
    # -------------------------------------------------------------------------
    def parse_rmc_sentence(self):
        """
        Parse RMC (Recommended Minimum Navigation Information).

        Reads: local time, date, lat/lon (DDM + hemisphere), speed (knots),
        course (degrees), and sets `has_fix`.

        Returns:
            bool: True if parsed OK, False if a field was malformed.
        """
        # time
        try:
            time_field = self.nmea_fields[1]
            if time_field:
                hh = int(time_field[0:2]); mm = int(time_field[2:4]); ss = float(time_field[4:])
                self.time_hms_local = self._apply_timezone_offset(hh, mm, ss)
            else:
                self.time_hms_local = [0, 0, 0.0]
        except (ValueError, IndexError):
            return False

        # date (kept as provided; typically UTC date from the satellite)
        try:
            date_field = self.nmea_fields[9]
            if date_field:
                day = int(date_field[0:2]); month = int(date_field[2:4]); year_2 = int(date_field[4:6])
                self.date_dmy = [day, month, year_2]
            else:
                self.date_dmy = [0, 0, 0]
        except (ValueError, IndexError):
            return False

        # validity + position/speed
        if self.nmea_fields[2] == "A":  # 'A' = valid
            try:
                lat_str = self.nmea_fields[3]; lat_deg = int(lat_str[0:2]); lat_min = float(lat_str[2:]); lat_hemi = self.nmea_fields[4]
                lon_str = self.nmea_fields[5]; lon_deg = int(lon_str[0:3]); lon_min = float(lon_str[3:]); lon_hemi = self.nmea_fields[6]
                if (lat_hemi not in self.VALID_HEMISPHERES) or (lon_hemi not in self.VALID_HEMISPHERES):
                    return False
                speed_knots = float(self.nmea_fields[7]) if self.nmea_fields[7] else 0.0
                course_deg = float(self.nmea_fields[8]) if self.nmea_fields[8] else 0.0
            except (ValueError, IndexError):
                return False

            self.raw_latitude_ddm = [lat_deg, lat_min, lat_hemi]
            self.raw_longitude_ddm = [lon_deg, lon_min, lon_hemi]
            self.speed_knots_mph_kph = [speed_knots, speed_knots * 1.151, speed_knots * 1.852]
            self.course_over_ground_deg = course_deg
            self.has_fix = True
            self._mark_fix_timestamp()
        else:
            self.raw_latitude_ddm = [0, 0.0, "N"]
            self.raw_longitude_ddm = [0, 0.0, "W"]
            self.speed_knots_mph_kph = [0.0, 0.0, 0.0]
            self.course_over_ground_deg = 0.0
            self.has_fix = False

        return True

    def parse_gll_sentence(self):
        """
        Parse GLL (Geographic Position — Latitude/Longitude).

        Updates: local time, lat/lon, has_fix.

        Returns:
            bool: True if parsed OK, False on malformed fields.
        """
        try:
            time_field = self.nmea_fields[5]
            if time_field:
                hh = int(time_field[0:2]); mm = int(time_field[2:4]); ss = float(time_field[4:])
                self.time_hms_local = self._apply_timezone_offset(hh, mm, ss)
            else:
                self.time_hms_local = [0, 0, 0.0]
        except (ValueError, IndexError):
            return False

        if self.nmea_fields[6] == "A":
            try:
                lat_field = self.nmea_fields[1]; lat_deg = int(lat_field[0:2]); lat_min = float(lat_field[2:]); lat_hemi = self.nmea_fields[2]
                lon_field = self.nmea_fields[3]; lon_deg = int(lon_field[0:3]); lon_min = float(lon_field[3:]); lon_hemi = self.nmea_fields[4]
                if (lat_hemi not in self.VALID_HEMISPHERES) or (lon_hemi not in self.VALID_HEMISPHERES):
                    return False
            except (ValueError, IndexError):
                return False

            self.raw_latitude_ddm = [lat_deg, lat_min, lat_hemi]
            self.raw_longitude_ddm = [lon_deg, lon_min, lon_hemi]
            self.has_fix = True
            self._mark_fix_timestamp()
        else:
            self.raw_latitude_ddm = [0, 0.0, "N"]
            self.raw_longitude_ddm = [0, 0.0, "W"]
            self.has_fix = False
        return True

    def parse_vtg_sentence(self):
        """
        Parse VTG (Course Over Ground and Ground Speed).

        Updates: course_over_ground_deg and speed_knots_mph_kph.

        Returns:
            bool: True if parsed OK, False on malformed fields.
        """
        try:
            course_deg = float(self.nmea_fields[1]) if self.nmea_fields[1] else 0.0
            speed_knots = float(self.nmea_fields[5]) if self.nmea_fields[5] else 0.0
        except (ValueError, IndexError):
            return False

        self.speed_knots_mph_kph = [speed_knots, speed_knots * 1.151, speed_knots * 1.852]
        self.course_over_ground_deg = course_deg
        return True

    def parse_gga_sentence(self):
        """
        Parse GGA (GPS Fix Data).

        Updates: local time, satellites_used_for_fix, fix_quality_gga, hdop,
                 position (if fix present), altitude_m, geoid_separation_m.

        Returns:
            bool: True if parsed OK, False on malformed fields.
        """
        try:
            time_field = self.nmea_fields[1]
            if time_field:
                hh = int(time_field[0:2]); mm = int(time_field[2:4]); ss = float(time_field[4:])
                hours, minutes, seconds = self._apply_timezone_offset(hh, mm, ss)
            else:
                hours = minutes = 0; seconds = 0.0

            sats_in_use = int(self.nmea_fields[7]) if self.nmea_fields[7] else 0
            fix_quality = int(self.nmea_fields[6]) if self.nmea_fields[6] else 0
        except (ValueError, IndexError):
            return False

        try:
            hdop = float(self.nmea_fields[8]) if self.nmea_fields[8] else 0.0
        except (ValueError, IndexError):
            hdop = 0.0

        if fix_quality:
            try:
                lat_field = self.nmea_fields[2]; lat_deg = int(lat_field[0:2]); lat_min = float(lat_field[2:]); lat_hemi = self.nmea_fields[3]
                lon_field = self.nmea_fields[4]; lon_deg = int(lon_field[0:3]); lon_min = float(lon_field[3:]); lon_hemi = self.nmea_fields[5]
                if (lat_hemi not in self.VALID_HEMISPHERES) or (lon_hemi not in self.VALID_HEMISPHERES):
                    return False
                altitude_m = float(self.nmea_fields[9]) if self.nmea_fields[9] else 0.0
                geoid_sep_m = float(self.nmea_fields[11]) if self.nmea_fields[11] else 0.0
            except (ValueError, IndexError):
                return False

            self.raw_latitude_ddm = [lat_deg, lat_min, lat_hemi]
            self.raw_longitude_ddm = [lon_deg, lon_min, lon_hemi]
            self.altitude_m = altitude_m
            self.geoid_separation_m = geoid_sep_m

        self.time_hms_local = [hours, minutes, seconds]
        self.satellites_used_for_fix = sats_in_use
        self.hdop = hdop
        self.fix_quality_gga = fix_quality
        if fix_quality:
            self._mark_fix_timestamp()
        return True

    def parse_gsa_sentence(self):
        """
        Parse GSA (GNSS DOP and Active Satellites).

        Updates: fix_type_gsa, satellite_ids_used, hdop, vdop, pdop.

        Returns:
            bool: True if parsed OK, False on malformed fields.
        """
        try:
            fix_type = int(self.nmea_fields[2]) if self.nmea_fields[2] else 1
        except (ValueError, IndexError):
            return False

        used_sat_ids = []
        for i in range(12):  # up to 12 PRN IDs
            idx = 3 + i
            if idx >= len(self.nmea_fields):
                break
            field = self.nmea_fields[idx]
            if not field:
                continue
            try:
                used_sat_ids.append(int(field))
            except ValueError:
                return False

        try:
            pdop = float(self.nmea_fields[15]) if len(self.nmea_fields) > 15 and self.nmea_fields[15] else 0.0
            hdop = float(self.nmea_fields[16]) if len(self.nmea_fields) > 16 and self.nmea_fields[16] else 0.0
            vdop = float(self.nmea_fields[17]) if len(self.nmea_fields) > 17 and self.nmea_fields[17] else 0.0
        except ValueError:
            return False

        self.fix_type_gsa = fix_type
        if fix_type > self.FIX_TYPE_NO_FIX:
            self._mark_fix_timestamp()

        self.satellite_ids_used = used_sat_ids
        self.hdop = hdop
        self.vdop = vdop
        self.pdop = pdop
        return True

    def parse_gsv_sentence(self):
        """
        Parse GSV (Satellites in View).

        Updates: satellites_in_view, satellites_info, and tracks multi-sentence
        GSV batches using gsv_total_sentences and gsv_sentence_index.

        Returns:
            bool: True if parsed OK, False on malformed fields.
        """
        try:
            total_sentences = int(self.nmea_fields[1]) if self.nmea_fields[1] else 0
            sentence_index = int(self.nmea_fields[2]) if self.nmea_fields[2] else 0
            sats_in_view = int(self.nmea_fields[3]) if self.nmea_fields[3] else 0
        except (ValueError, IndexError):
            return False

        sat_chunk = {}
        if total_sentences == sentence_index and sats_in_view > 0:
            remaining = sats_in_view - ((total_sentences - 1) * 4)
            limit = max(5 * remaining, 8)  # exclusive bound
        else:
            limit = 20  # 4 satellites worth of fields

        for s in range(4, limit, 4):
            if s >= len(self.nmea_fields) or not self.nmea_fields[s]:
                break
            try:
                sat_id = int(self.nmea_fields[s])
            except (ValueError, IndexError):
                return False

            try:
                elev = int(self.nmea_fields[s + 1]) if self.nmea_fields[s + 1] else None
            except (ValueError, IndexError):
                elev = None
            try:
                az = int(self.nmea_fields[s + 2]) if self.nmea_fields[s + 2] else None
            except (ValueError, IndexError):
                az = None
            try:
                snr = int(self.nmea_fields[s + 3]) if self.nmea_fields[s + 3] else None
            except (ValueError, IndexError):
                snr = None

            sat_chunk[sat_id] = (elev, az, snr)

        self.gsv_total_sentences = total_sentences
        self.gsv_sentence_index = sentence_index
        self.satellites_in_view = sats_in_view

        if sentence_index == 1:
            self.satellites_info = sat_chunk
        else:
            self.satellites_info.update(sat_chunk)
        return True

    # -------------------------------------------------------------------------
    # Character stream handler (UART → parser)
    # -------------------------------------------------------------------------
    def _begin_new_sentence(self):
        """
        Internal: reset state for a new sentence starting with '$'.

        Returns:
            None
        """
        self.nmea_fields = [""]
        self.field_index = 0
        self.checksum_value = 0
        self.nmea_sentence_active = True
        self.collect_checksum = True
        self.sentence_char_count = 0

    def update_from_char(self, ch):
        """
        Feed a single character from the NMEA stream.

        Args:
            ch (str): One character, e.g., from `uart.read()` bytes iterated
                byte-by-byte and converted with `chr(byte)`.

        Returns:
            str|None: The sentence type (e.g., 'GPRMC') if a complete, checksum-
            valid sentence was parsed and dispatched successfully; otherwise None.

        Notes:
            - This function is deliberately small and fast: call it for every
              incoming byte in your UART loop.
            - Control characters CR/LF/TAB are ignored and not counted toward
              checksum or fields, matching NMEA line endings.
        """
        sentence_valid = False
        code_point = ord(ch)

        # Start of sentence
        if ch == "$":
            self._begin_new_sentence()
            return None

        if not self.nmea_sentence_active:
            return None  # ignore noise until next '$'

        # Ignore CR/LF/TAB completely (not part of checksum or fields)
        if ch in ("\r", "\n", "\t"):
            return None

        self.sentence_char_count += 1

        if ch == "*":
            self.collect_checksum = False
            self.field_index += 1
            self.nmea_fields.append("")
            return None

        if ch == ",":
            self.field_index += 1
            self.nmea_fields.append("")
        else:
            # Accumulate content in current field
            self.nmea_fields[self.field_index] += ch

            # When reading the two hex checksum digits
            if not self.collect_checksum and len(self.nmea_fields[self.field_index]) == 2:
                try:
                    supplied_crc = int(self.nmea_fields[self.field_index], 16)
                    if self.checksum_value == supplied_crc:
                        sentence_valid = True
                    else:
                        self.checksum_failures += 1
                except ValueError:
                    # Malformed hex; ignore and let the overflow guard reset the sentence
                    pass

        if self.collect_checksum:
            # XOR of characters between '$' and '*', including commas/periods/etc.
            self.checksum_value ^= code_point

        if sentence_valid:
            self.valid_sentence_count += 1
            self.nmea_sentence_active = False

            # Dispatch to specific sentence parser (if supported)
            if self.nmea_fields and self.nmea_fields[0] in self.SUPPORTED_SENTENCE_PARSERS:
                if self.SUPPORTED_SENTENCE_PARSERS[self.nmea_fields[0]](self):
                    self.parsed_sentence_count += 1
                    return self.nmea_fields[0]

        # Overflow protection for pathological lines
        if self.sentence_char_count > self.MAX_NMEA_SENTENCE_LENGTH:
            self.nmea_sentence_active = False

        return None

    # Backward-compatibility alias
    def update(self, ch):
        """
        Compatibility wrapper for older code that called `update()`.

        Args:
            ch (str): One character from the NMEA stream.

        Returns:
            str|None: See `update_from_char`.
        """
        return self.update_from_char(ch)

    # -------------------------------------------------------------------------
    # Utility helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def convert_coord_to_decimal_degrees_string(parts):
        """
        Convert one of the coordinate representations to a decimal-degrees string.

        Accepts:
            - [deg:int, dec_min:float, hemi:str]      (DDM)
            - [decimal_degrees:float, hemi:str]       (DD)
            - [deg:int, min:int, sec:int, hemi:str]   (DMS)

        Args:
            parts (list|tuple|None): Coordinate structure.

        Returns:
            str: Decimal degrees string with 6 decimals, or 'N/A' if unavailable.

        Examples:
            convert_coord_to_decimal_degrees_string([25, 15.1234, 'N']) -> '25.252057'
            convert_coord_to_decimal_degrees_string([55.309463, 'E'])   -> '55.309463'
        """
        try:
            if parts is None:
                return "N/A"
            # DDM
            if isinstance(parts, (list, tuple)) and len(parts) == 3:
                deg, dec_min, hemi = parts
                dd = float(deg) + float(dec_min) / 60.0
                if hemi in ("S", "W"):
                    dd = -dd
                return f"{dd:.6f}"
            # DD
            if isinstance(parts, (list, tuple)) and len(parts) == 2:
                dd, hemi = parts
                dd = float(dd)
                if hemi in ("S", "W"):
                    dd = -dd
                return f"{dd:.6f}"
            # DMS
            if isinstance(parts, (list, tuple)) and len(parts) == 4:
                deg, mins, secs, hemi = parts
                dd = float(deg) + float(mins) / 60.0 + float(secs) / 3600.0
                if hemi in ("S", "W"):
                    dd = -dd
                return f"{dd:.6f}"
        except Exception:
            pass
        return "N/A"

    def _mark_fix_timestamp(self):
        """
        Record the timestamp (milliseconds) of the latest fix-related update.

        Returns:
            None
        """
        try:
            self.last_fix_timestamp_ms = time_module.ticks_ms()
        except (NameError, AttributeError):
            # CPython fallback
            self.last_fix_timestamp_ms = int(time_module.time() * 1000)

    def is_satellite_data_complete(self):
        """
        Check whether all expected GSV sentences (for the current cycle) arrived.

        Returns:
            bool: True when `gsv_sentence_index` == `gsv_total_sentences` > 0.
        """
        return (self.gsv_total_sentences > 0) and (self.gsv_total_sentences == self.gsv_sentence_index)

    def reset_satellite_update_flag(self):
        """
        Mark satellite sequence as incomplete (for the next batch).

        Returns:
            None
        """
        self.gsv_sentence_index = 0

    def visible_satellite_ids(self):
        """
        Get the list of PRN IDs visible from the latest GSV set.

        Returns:
            list[int]: Satellite PRN numbers.
        """
        return list(self.satellites_info.keys())

    def milliseconds_since_last_fix(self):
        """
        Milliseconds since the last fix-related sentence was parsed.

        Returns:
            int: Milliseconds since last fix, or -1 if no fix yet.
        """
        if self.last_fix_timestamp_ms == 0:
            return -1
        try:
            return time_module.ticks_diff(time_module.ticks_ms(), self.last_fix_timestamp_ms)
        except (NameError, AttributeError):
            return int((time_module.time() * 1000) - self.last_fix_timestamp_ms)

    def course_compass_direction(self):
        """
        Convert `course_over_ground_deg` to a 16-point compass direction.

        Returns:
            str: One of COMPASS_DIRECTIONS_16 (e.g., 'N', 'NNE', ...).
        """
        if self.course_over_ground_deg >= 348.75:
            offset = 360.0 - self.course_over_ground_deg
        else:
            offset = self.course_over_ground_deg + 11.25
        idx = floor(offset / 22.5)
        return self.COMPASS_DIRECTIONS_16[idx]

    def latitude_string(self):
        """
        Human-readable latitude string according to `coordinate_format`.

        Returns:
            str: e.g., "25.252058° N" (DD) or "25° 15.123' N" (DDM).
        """
        if self.coordinate_format == "dd":
            dd, hemi = self.latitude
            return f"{dd}° {hemi}"
        if self.coordinate_format == "dms":
            d, m, s, hemi = self.latitude
            return f'{d}° {m}\' {s}" {hemi}'
        # ddm
        d, dm, hemi = self.raw_latitude_ddm
        return f"{d}° {dm}' {hemi}"

    def longitude_string(self):
        """
        Human-readable longitude string according to `coordinate_format`.

        Returns:
            str: e.g., "55.309463° E" (DD) or "55° 18.568' E" (DDM).
        """
        if self.coordinate_format == "dd":
            dd, hemi = self.longitude
            return f"{dd}° {hemi}"
        if self.coordinate_format == "dms":
            d, m, s, hemi = self.longitude
            return f'{d}° {m}\' {s}" {hemi}'
        # ddm
        d, dm, hemi = self.raw_longitude_ddm
        return f"{d}° {dm}' {hemi}"

    def speed_string(self, unit="kph"):
        """
        Human-readable speed string.

        Args:
            unit (str): 'kph' (default), 'mph', or 'knot'.

        Returns:
            str: e.g., "22.8 km/h" or "12.3 knots".
        """
        if unit == "mph":
            return f"{self.speed_knots_mph_kph[1]} mph"
        if unit == "knot":
            unit_str = " knot" if self.speed_knots_mph_kph[0] == 1 else " knots"
            return f"{self.speed_knots_mph_kph[0]}{unit_str}"
        return f"{self.speed_knots_mph_kph[2]} km/h"

    def date_string(self, style="s_mdy", century_prefix="20"):
        """
        Format date as a string.

        Args:
            style (str): 'long' -> 'January 1st, 2014'
                         's_mdy' -> 'MM/DD/YY' (default)
                         's_dmy' -> 'DD/MM/YY'
            century_prefix (str): e.g., '20' for 20XX.

        Returns:
            str: Formatted date or '' if date invalid for 'long' style.
        """
        day, month, year_2 = self.date_dmy

        if style == "long":
            if (month < 1) or (month > 12) or (day == 0):
                return ""
            month_name = self.MONTH_NAMES[month - 1]
            if day in (1, 21, 31):
                suffix = "st"
            elif day in (2, 22):
                suffix = "nd"
            elif day in (3, 23):
                suffix = "rd"
            else:
                suffix = "th"
            return f"{month_name} {day}{suffix}, {century_prefix}{year_2}"

        d = f"{day:02d}"; m = f"{month:02d}"; y = f"{year_2:02d}"
        return f"{m}/{d}/{y}" if style == "s_mdy" else f"{d}/{m}/{y}"

    # -------------------------------------------------------------------------
    # Supported sentence mapping (dispatcher)
    # -------------------------------------------------------------------------
    SUPPORTED_SENTENCE_PARSERS = {
        "GPRMC": parse_rmc_sentence, "GLRMC": parse_rmc_sentence, "GNRMC": parse_rmc_sentence,
        "GPGGA": parse_gga_sentence, "GLGGA": parse_gga_sentence, "GNGGA": parse_gga_sentence,
        "GPVTG": parse_vtg_sentence, "GLVTG": parse_vtg_sentence, "GNVTG": parse_vtg_sentence,
        "GPGSA": parse_gsa_sentence, "GLGSA": parse_gsa_sentence, "GNGSA": parse_gsa_sentence,
        "GPGSV": parse_gsv_sentence, "GLGSV": parse_gsv_sentence,
        "GPGLL": parse_gll_sentence, "GLGLL": parse_gll_sentence, "GNGLL": parse_gll_sentence,
    }


if __name__ == "__main__":
    # Library module; no standalone behavior.
    pass
