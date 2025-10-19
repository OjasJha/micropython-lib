# ArduCam SPI Mega - MicroPython Driver

A MicroPython driver for the ArduCam SPI Mega camera modules (3MP and 5MP) designed for Raspberry Pi Pico.

## License

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

## Features

- **Camera Support**: Compatible with ArduCam 3MP and 5MP SPI Mega modules
- **Auto-detection**: Automatically detects camera sensor type (3MP/5MP)
- **Multiple Resolutions**: Support for various resolutions from 96x96 to 2592x1944
- **Image Effects**: Brightness, contrast, saturation, exposure, and color filters
- **White Balance**: Multiple white balance modes (Auto, Sunny, Office, Cloudy, Home)
- **File Management**: Auto-incrementing filename system to prevent overwrites
- **Burst Reading**: Efficient image capture with progress bar display

## Hardware Requirements

- Raspberry Pi Pico or Pico 2
- ArduCam SPI Mega (3MP or 5MP variant)
- MicroPython firmware (v1.26.1 or later)

## Wiring

| ArduCam Pin | Raspberry Pi Pico Pin | Wire Color (Example) |
|-------------|----------------------|----------------------|
| SCK         | GP18                 | White                |
| MISO        | GP16                 | Brown                |
| MOSI        | GP19                 | Yellow               |
| CS          | GP17                 | Orange               |
| VCC         | 3.3V                 | Red                  |
| GND         | GND                  | Black                |

## Files

### `camera_handler.py`

Main driver module containing:
- **`FileManager` class**: Manages auto-incrementing filenames for captured images
- **`Camera` class**: Core camera driver with full hardware control

### `camera_test.py`

Test module demonstrating basic camera usage:
- Hardware initialization
- Camera configuration
- Single photo capture with status LED indication

## Installation

1. Copy `camera_handler.py` to your Raspberry Pi Pico
2. Copy `camera_test.py` if you want to run the test example

## Quick Start

```python
from machine import Pin, SPI
from camera_handler import Camera, FileManager

# Initialize SPI bus
spi_bus = SPI(0, sck=Pin(18), miso=Pin(16), mosi=Pin(19), baudrate=8000000)
cs_pin = Pin(17, Pin.OUT)

# Initialize camera
camera = Camera(spi_bus, cs_pin, debug_text_enabled=True)

# Initialize file manager
file_manager = FileManager()

# Configure camera settings
camera.resolution = '640x480'
camera.set_brightness_level(camera.BRIGHTNESS_PLUS_4)
camera.set_contrast(camera.CONTRAST_MINUS_3)

# Capture and save photo
camera.capture_jpg()
filename = file_manager.new_jpg_filename('image')
camera.save_JPG(filename, progress_bar=True)
```

## API Reference

### Camera Class

#### Initialization

```python
Camera(spi_bus, cs, skip_sleep=False, debug_text_enabled=False)
```

- `spi_bus`: Initialized SPI bus object
- `cs`: Chip select pin object
- `skip_sleep`: Skip white balance warmup delay (default: False)
- `debug_text_enabled`: Enable debug output (default: False)

#### Resolution Settings

Set resolution using the `resolution` property:

```python
camera.resolution = '640x480'  # Set resolution
```

**Supported Resolutions:**

**3MP Camera:**
- `'320x240'`, `'640x480'`, `'1280x720'`, `'1600x1200'`
- `'1920x1080'`, `'2048x1536'`
- `'96x96'`, `'128x128'`, `'320x320'`

**5MP Camera:**
- `'320x240'`, `'640x480'`, `'1280x720'`, `'1600x1200'`
- `'1920x1080'`, `'2592x1944'`
- `'96x96'`, `'128x128'`, `'320x320'`

#### Image Capture

```python
camera.capture_jpg()  # Capture image to FIFO buffer
camera.save_JPG(filename, progress_bar=True)  # Save to file
```

#### Camera Settings

**Brightness:**
```python
camera.set_brightness_level(camera.BRIGHTNESS_PLUS_4)
```
Options: `BRIGHTNESS_MINUS_4` to `BRIGHTNESS_PLUS_4`, `BRIGHTNESS_DEFAULT`

**Contrast:**
```python
camera.set_contrast(camera.CONTRAST_MINUS_3)
```
Options: `CONTRAST_MINUS_3` to `CONTRAST_PLUS_3`, `CONTRAST_DEFAULT`

**Saturation:**
```python
camera.set_saturation_control(camera.SATURATION_PLUS_2)
```
Options: `SATURATION_MINUS_3` to `SATURATION_PLUS_3`, `SATURATION_DEFAULT`

**Exposure:**
```python
camera.set_exposure_control(camera.EXPOSURE_PLUS_1)
```
Options: `EXPOSURE_MINUS_3` to `EXPOSURE_PLUS_3`, `EXPOSURE_DEFAULT`

**Color Effects:**
```python
camera.set_filter(camera.SPECIAL_NORMAL)
```
Options: `SPECIAL_NORMAL`, `SPECIAL_COOL`, `SPECIAL_WARM`, `SPECIAL_BW`, 
`SPECIAL_YELLOWING`, `SPECIAL_REVERSE`, `SPECIAL_GREENISH`, 
`SPECIAL_LIGHT_YELLOW` (3MP only)

**White Balance:**
```python
camera.set_white_balance('sunny')
```
Options: `'auto'`, `'sunny'`, `'office'`, `'cloudy'`, `'home'`

**Image Quality:**
```python
camera.set_image_quality(camera.IMAGE_QUALITY_HIGH)
```
Options: `IMAGE_QUALITY_HIGH`, `IMAGE_QUALITY_MEDI`, `IMAGE_QUALITY_LOW`

**Sharpness (3MP only):**
```python
camera.set_sharpness(camera.SHARPNESS_NORMAL)
```
Options: `SHARPNESS_NORMAL` to `SHARPNESS_8`

### FileManager Class

Manages auto-incrementing filenames to prevent file overwrites.

#### Initialization

```python
FileManager(file_manager_name='filemanager.log')
```

- `file_manager_name`: JSON log file storing filename counters

#### Methods

```python
# Get new filename with .jpg extension
filename = file_manager.new_jpg_filename('image')  # Returns 'image.jpg', 'image_1.jpg', etc.

# Get new filename without extension
filename = file_manager.new_filename('data')  # Returns 'data', 'data_1', etc.
```

## Important Notes

### White Balance Warmup

Both 3MP and 5MP cameras require a warmup period for automatic white balance:
- Default warmup time: 500ms
- The driver automatically handles this delay during initialization
- For best results, wait before capturing the first photo

### Green Hue Issue (3MP Camera)

The 3MP camera may produce images with a green hue on first capture:
- **Solution**: Add a 2000ms delay after camera initialization
- The test module includes this delay automatically
- Alternatively, capture and discard a dummy image during startup

### Memory Considerations

- FIFO buffer read uses 255-byte chunks for memory efficiency
- Large images (>2MP) require adequate free RAM
- Monitor available memory when capturing high-resolution images

## Example Usage

See `camera_test.py` for a complete working example that demonstrates:
1. Hardware initialization
2. Camera configuration
3. Photo capture with LED status indication
4. Auto-incrementing filename management

## Troubleshooting

**FIFO Length Error (>5MB):**
- Indicates camera didn't capture properly
- Add delays after initialization
- Check SPI connections and baudrate

**Import Resolution Not Working:**
- Ensure resolution string matches supported format
- Check camera type (3MP vs 5MP) for resolution compatibility

**Image Quality Issues:**
- Adjust brightness, contrast, and exposure settings
- Set appropriate white balance for environment
- Increase warmup time for 3MP cameras

## References

- [ArduCam MEGA Application Note](https://www.arducam.com/downloads/datasheet/Arducam_MEGA_SPI_Camera_Application_Note.pdf)
- [ArduCam Forum Discussion](https://forum.arducam.com/t/mega-3mp-micropython-driver/5708)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)
- [MicroPython Documentation](https://docs.micropython.org/en/latest/rp2/quickref.html)

## Contributing

Contributions are welcome! Please ensure code follows PEP 8 guidelines and is compatible with MicroPython.

## Author

Ojas Jha

## Changelog

### 2025-10-19
- Initial release with MIT License
- Support for 3MP and 5MP ArduCam SPI Mega modules
- Auto-incrementing file management system
- Comprehensive camera control API

