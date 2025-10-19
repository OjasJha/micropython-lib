"""
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

---

Camera Test Module for ArduCam SPI Mega

This module tests the camera functionality by capturing a single photo
and saving it to the filesystem with an auto-incremented filename.
"""

# Import required modules
from machine import Pin, SPI, reset
from camera_handler import *


# ==================== Hardware Configuration Constants ====================

# SPI Bus Configuration
SPI_BUS_ID = 0
SPI_SCK_PIN = 18   # Serial Clock (GP18) - White wire
SPI_MISO_PIN = 16  # Master In Slave Out (GP16) - Brow wire
SPI_MOSI_PIN = 19  # Master Out Slave In (GP19) - Yellow wire
SPI_BAUDRATE = 8000000  # 8 MHz

# Camera Chip Select Pin
CAMERA_CS_PIN = 17  # Chip Select (GP17) - Orange wire


# ==================== Camera Settings Constants ====================

# Image Resolution
DEFAULT_RESOLUTION = '640x480'

# Image Quality Settings
CAMERA_BRIGHTNESS = 'BRIGHTNESS_PLUS_4'  # Brightness level
CAMERA_CONTRAST = 'CONTRAST_MINUS_3'     # Contrast level


# ==================== Timing Constants ====================

# Delay times in milliseconds
CAMERA_INIT_DELAY_MS = 2000      # Initial delay after camera object creation


# ==================== File Management Constants ====================

# Default base filename for saved images
DEFAULT_IMAGE_BASENAME = 'image'


# ==================== Function Definitions ====================

def initialize_camera_hardware():
    """
    Initialize the SPI bus and camera chip select pin.

    Returns:
        tuple: (camera_spi_bus, camera_chip_select) - Initialized SPI and CS pin objects
    """
    # Configure SPI bus for camera communication
    camera_spi_bus = SPI(
        SPI_BUS_ID,
        sck=Pin(SPI_SCK_PIN),
        miso=Pin(SPI_MISO_PIN),
        mosi=Pin(SPI_MOSI_PIN),
        baudrate=SPI_BAUDRATE
    )

    # Chip select pin for camera
    camera_chip_select = Pin(CAMERA_CS_PIN, Pin.OUT)

    return camera_spi_bus, camera_chip_select


def configure_camera_settings(camera):
    """
    Configure camera resolution, brightness, and contrast settings.

    Args:
        camera: Camera object to configure
    """
    # Set image resolution
    camera.resolution = DEFAULT_RESOLUTION

    # Adjust brightness and contrast for better image quality
    camera.set_brightness_level(getattr(camera, CAMERA_BRIGHTNESS))
    camera.set_contrast(getattr(camera, CAMERA_CONTRAST))


def capture_and_save_photo(camera, file_manager, base_filename=DEFAULT_IMAGE_BASENAME):
    """
    Capture a photo and save it with an auto-incremented filename.

    Args:
        camera: Camera object used to capture the photo
        file_manager: FileManager object for handling filenames
        base_filename: Base name for the image file (default: DEFAULT_IMAGE_BASENAME)

    Note:
        FileManager adds _# suffix to prevent overwriting existing files.
        Without this, images would stack in the same file, causing corruption.
    """
    # Capture JPEG image from camera
    camera.capture_jpg()
    # Wait to remove the green hue in the image. Refer to https://forum.arducam.com/t/mega-3mp-micropython-driver/5708
    sleep_ms(CAMERA_INIT_DELAY_MS)

    # Save with auto-incremented filename and progress indicator
    output_filename = file_manager.new_jpg_filename(base_filename)
    camera.save_JPG(output_filename, progress_bar=True)


def main():
    """
    Main function to test camera functionality.

    This function:
    1. Initializes the onboard LED for status indication
    2. Sets up camera hardware (SPI and CS pin)
    3. Configures camera with optimal settings
    4. Captures and saves a single photo
    """
    # Initialize status LED (onboard LED on Raspberry Pi Pico)
    status_led = Pin("LED", Pin.OUT)

    # Initialize file manager for auto-incrementing filenames
    file_manager = FileManager()

    # Set up camera hardware
    camera_spi_bus, camera_chip_select = initialize_camera_hardware()

    # Create camera object with debug output enabled
    camera = Camera(camera_spi_bus, camera_chip_select, debug_text_enabled=True)
    # Wait to remove the green hue in the image. Refer to https://forum.arducam.com/t/mega-3mp-micropython-driver/5708
    sleep_ms(CAMERA_INIT_DELAY_MS)

    # Configure camera settings
    configure_camera_settings(camera)

    # Turn on status LED to indicate photo capture in progress
    status_led.on()

    # Capture and save photo
    capture_and_save_photo(camera, file_manager)

    # Turn off status LED to indicate completion
    status_led.off()


if __name__ == "__main__":
    main()

