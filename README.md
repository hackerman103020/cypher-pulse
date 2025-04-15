# 2x CC1101 Jammer Display ESP32

![Project Banner](img/banner.png)
*Project banner showing the device in action*

## Overview

This project is an enhanced version of the Evil Crow RF tool, featuring a dual CC1101 radio system with an ESP32 microcontroller. The original work by Adam Loboda has been expanded with a modern OLED display interface, button controls, and dual radio capabilities by Cypher of Little Hakr.

![Hardware Setup](img/setup.png)
*Hardware setup showing the ESP32, CC1101 modules, and OLED display*

## Features

- **Dual CC1101 Radio System**
  - Independent control of two CC1101 modules
  - Simultaneous or individual operation modes
  - Enhanced signal coverage and flexibility

- **Modern User Interface**
  - 128x64 OLED display
  - Intuitive menu system
  - Real-time status updates
  - Signal strength visualization

- **Button Controls**
  - Up/Down navigation
  - Select button for mode activation
  - LED feedback for button presses

- **RF Capabilities**
  - Signal jamming (single or dual radio)
  - Frequency scanning (433.60MHz - 434.20MHz)
  - Signal recording and playback
  - Raw data manipulation
  - RSSI monitoring

- **Frequency Presets**
  - 433.90MHz
  - 434.00MHz
  - 434.30MHz
  - 434.40MHz

## Hardware Requirements

![Components](img/components.png)
*Required components for the project*

- ESP32 development board
- Two CC1101 radio modules
- SSD1306 OLED display (128x64)
- Three push buttons
- LED indicator
- Jumper wires
- Breadboard or custom PCB

## Pin Configuration

![Pinout Diagram](img/pinout.png)
*Pin configuration diagram*

### CC1101 #1
- SCK: GPIO14
- MISO: GPIO12
- MOSI: GPIO13
- SS: GPIO15
- GDO0: GPIO26
- GDO2: GPIO27

### CC1101 #2
- SCK: GPIO18
- MISO: GPIO19
- MOSI: GPIO23
- SS: GPIO17
- GDO0: GPIO25
- GDO2: GPIO33

### Display & Controls
- OLED I2C: SDA=GPIO21, SCL=GPIO22
- Buttons: UP=GPIO34, DOWN=GPIO39, SELECT=GPIO32
- LED: GPIO27

## Installation

1. Clone this repository
2. Install required libraries:
   - ELECHOUSE_CC1101_SRC_DRV
   - ELECHOUSE_CC1101_SRC_DRV2
   - Adafruit_GFX
   - Adafruit_SSD1306
   - U8g2_for_Adafruit_GFX
   - EEPROM

3. Connect hardware according to pin configuration
4. Upload the sketch to your ESP32

## Usage

1. Power on the device
2. Navigate through the menu using UP/DOWN buttons
3. Select modes using the SELECT button
4. Monitor operations on the OLED display
5. Use serial monitor for additional control (115200 baud)

## Menu Options

- **2X CC JAM**: Activate both radios for jamming
- **CC#1 JAM**: Activate first radio only
- **CC#2 JAM**: Activate second radio only
- **SCAN**: Scan for RF signals
- **TEST_CC1101**: Test radio connections
- **REC RAW**: Record raw RF data
- **PLAY RAW**: Playback recorded data
- **SHOW RAW**: Display recorded data
- **SHOW BUFF**: Show buffer contents
- **GET RSSI**: Display signal strength
- **FLUSH BUFF**: Clear recording buffer
- **STOP ALL**: Stop all operations
- **RESET CC**: Reset radio modules
- **Frequency Presets**: Quick frequency selection

## Credits

- Original CC1101 implementation by Adam Loboda (adam.loboda@wp.pl)
- Display, button interface, and dual radio implementation by Cypher of Little Hakr
- Based on the Evil Crow RF project

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Disclaimer

This tool is intended for educational and research purposes only. Always ensure you have proper authorization before testing RF equipment. The authors are not responsible for any misuse of this tool.

## Support

For issues and feature requests, please open an issue on GitHub.

---

*"The best way to predict the future is to create it." - Cypher of Little Hakr* 