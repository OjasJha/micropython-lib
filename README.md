# MicroPython Libraries Collection

Welcome to this comprehensive collection of MicroPython libraries for sensors, actuators, communication modules, and breakout boards! This repository serves as a one-stop resource for the DIY maker community, hobbyists, students, and educators looking to quickly get started with embedded systems and IoT projects.

## 🎯 Purpose

This library collection brings together tested, ready-to-use MicroPython drivers and handlers for a wide range of hardware components. Whether you're building your first IoT project, creating a robotics system, or developing a complex sensor network, you'll find the building blocks you need here.

## ✅ Tested Platform

All libraries in this collection have been tested with **Raspberry Pi Pico** running MicroPython. While many libraries may work with other MicroPython-compatible boards, compatibility is guaranteed for Raspberry Pi Pico.

## 📁 Organization

The libraries are organized into logical categories for easy navigation:

- **actuators/** - Motors, servos, and other motion control components
- **com/** - Communication protocols and modules (I2C, SPI, UART, GSM, ZigBee, etc.)
- **converter/** - ADC, DAC, and PWM converters
- **io/** - Input/output devices (cameras, displays, keyboards, joysticks)
- **mcu/** - Microcontroller-specific utilities and board handlers
- **microsd/** - SD card storage handlers
- **power/** - Power management and battery monitoring
- **sensors/** - Comprehensive collection of sensors organized by type (motion, temperature, GPS, IMU, environmental, etc.)
- **switch/** - Various switch and trigger modules

Each subdirectory typically contains:
- Handler/driver files (`*_handler.py`) - Main interface to the hardware
- Test files (`*_test.py`) - Example code and testing routines
- Library files (`*.py`) - Core implementation
- Documentation (`readme.txt`, `README.md`) - Setup instructions and usage notes

## 🚀 Getting Started

1. **Choose your component** - Browse the category folders to find the hardware you're working with
2. **Read the documentation** - Check the readme files in the component's directory for wiring diagrams and pin configurations
3. **Copy the required files** - Upload the necessary `.py` files to your Raspberry Pi Pico
4. **Run the test file** - Start with the `*_test.py` file to verify your hardware connections
5. **Integrate into your project** - Use the `*_handler.py` files as building blocks for your application

## 💡 For Newcomers

If you're new to MicroPython or embedded systems:
- Start with simple components like basic sensors or displays
- Read through the test files to understand how each component is initialized and used
- Experiment with the examples before building your own projects
- Check the documentation folder for additional getting started resources

## 🤝 Community

This collection is designed to help you learn, build, and create. Feel free to use these libraries in your personal projects, educational activities, or commercial applications under the terms of the MIT License.

## 📋 Requirements

- Raspberry Pi Pico (or compatible MicroPython board)
- MicroPython firmware installed
- Basic understanding of Python programming
- Appropriate hardware components and wiring

## 🔧 Best Practices

- Always check pin configurations before connecting hardware
- Test individual components with the provided test files before integration
- Refer to component datasheets for electrical specifications
- Use appropriate power supplies and current limiting when needed

## 📝 License

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

**Happy Making! 🎉**

*This library collection is maintained as a resource for the DIY and educational community. Contributions, improvements, and feedback are always welcome.*

