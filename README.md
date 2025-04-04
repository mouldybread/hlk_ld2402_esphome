# HLK-LD2402 ESPHome Component

This component integrates the HLK-LD2402 24GHz radar module with ESPHome, providing presence detection, micromovement sensing, and distance measurement. It comprehensively integrates all the documented features.

The code here is generated by AI! I've taken time to make sure it all works but it may be doing very stupid stuff I don't know about. That said if not for AI support for this module may not exist at all.

The code here was developed on a module running v3.5.5 firmware. During development there were various incongruities and undocumented features that have been worked around or in to this component. My intended application uses an ESP8266 so the code here is written with that in mind, if you are using another board remember to change things accordingly. 

Version G - that is, HLK-LD2402G includes an LD which allows this module to run from 5V. Other versions should be 3.3v. The documentation for this module seems to lag behind the actual implementation and even then is only available in Chinese, it is though in this reposity should you wish to refer to it. Hilink have made various related downloads available [here](https://www.hlktech.com/en/Goods-248.html#datum). I do note that their copy of sscom5.13.1 was flagged by virus total and the sandboxing picked up some fairly creepy behaviour, and it's superfluous anyway, so just avoid it. The LD2402G specific tool was clean but do always run it against virus total or use it in a sandboxed environment. 

This module is the only one i've seen that implements a kind of adjustable calibration, where it will tune the thresholds to a baseline sensitivity. The power interference check is also pretty cool. It does run hot to the touch. 

We'll be using the ESP8266 TX/RX pins for this so don't forget to disable uart logging!
   ```yaml
logger:
  baud_rate: 0
   ```

## Table of Contents
- [Module Overview](#module-overview)
- [ESPHome Integration Features](#esphome-integration-features)
- [Installation](#installation)
- [Configuration Options](#configuration-options)
  - [Complete Configuration](#complete-configuration)
  - [Minimal Configuration](#minimal-configuration)
- [Available Sensors](#available-sensors)
- [Control Functions](#control-functions)
- [Calibration Guide](#calibration-guide)
- [Threshold Management](#threshold-management)
- [Engineering Mode](#engineering-mode)
- [Common Issues & Solutions](#common-issues--solutions)
- [Technical Reference](#technical-reference)

## Module Overview

The HLK-LD2402 is a 24GHz millimeter-wave radar sensor from Hi-Link that can detect human presence, motion, and micromovement. It's particularly useful for IoT applications where accurate presence detection is required even when subjects are stationary.

### Key Features from Technical Documentation

- **Detection Capabilities**:
  - Movement detection up to 10m
  - Micromovement detection up to 6m
  - Static human presence detection up to 5m
  - Horizontal detection angle of ±60°

- **Technical Specifications**:
  - 24GHz ISM frequency band (24.0-24.25GHz)
  - Distance measurement accuracy: ±0.15m
  - Power supply: 3.0-3.6V (standard 3.3V) or 4.5-5.5V with LDO
  - Average current consumption: 50mA
  - Communication: UART (115200 baud rate, 8N1)
  - Operating temperature: -40°C to 85°C
  - Compact size: 20mm × 20mm

## ESPHome Integration Features

- **Core Sensors**:
  - Distance sensor (cm) - Measures distance to detected person
  - Presence binary sensor - Detects human presence (motion + stationary)
  - Micromovement binary sensor - Detects subtle movements like breathing
  - Power interference binary sensor - Monitors power supply quality

- **Diagnostic Sensors**:
  - Energy gate sensors (15 gates) - Show raw signal strength at different distances
  - Motion threshold sensors - Display configured motion sensitivity by distance gate
  - Micromotion threshold sensors - Display configured micromotion sensitivity
  - Calibration progress sensor - Shows percentage completion during calibration
  - Firmware version text sensor - Displays device firmware version
  - Operating mode text sensor - Shows current operating mode (normal/engineering/config)

- **Control Features**:
  - Calibration - Automatically optimizes detection thresholds
  - Auto-gain function - Adjusts signal amplification for environment
  - Configuration saving - Persists settings to radar module's flash memory
  - Engineering mode - Enables advanced testing and raw signal monitoring
  - Gate threshold management - Fine-tune detection sensitivity for each distance gate
  - Custom calibration - Generate thresholds with adjustable sensitivity multipliers
  - Settings reset - Restore factory default configuration

- **Modes of Operation**:
  - Normal mode - Standard presence detection
  - Engineering mode - Detailed signal and threshold analysis
  - Configuration mode - Parameter adjustment and calibration

- **Configuration Options**:
  - Maximum detection distance
  - Target disappearance timeout
  - Individual threshold adjustment for all 15 distance gates
  - Calibration sensitivity multipliers

## Installation

1. Add this repository to your ESPHome external components:
   ```yaml
   external_components:
     - source:
         type: git
         url: https://github.com/mouldybread/hlk_ld2402_esphome
       refresh: 0ms
   ```

2. Configure the component with one of the example YAML files below
   - Use `hlk_ld2402_complete.yaml` for setup and calibration
   - Use `hlk_ld2402_minimal.yaml` for everyday operation after calibration

3. Flash your ESP device with the chosen configuration

## Configuration Options

### Complete Configuration
The complete configuration (`hlk_ld2402_complete.yaml`) includes all available sensors, diagnostic tools, and calibration controls. This is recommended when:
- First setting up the device
- Calibrating the radar
- Fine-tuning detection thresholds
- Troubleshooting detection issues

### Minimal Configuration
After your device is properly calibrated and configured, switch to the minimal configuration (`hlk_ld2402_minimal.yaml`) which includes only:
- Distance sensor
- Presence detection
- Micromovement detection
- Power interference detection
- Basic calibration button
- Save configuration button

This streamlined configuration reduces memory usage, improves responsiveness, and presents a cleaner UI for everyday use.

## Available Sensors

### Binary Sensors

| Sensor | Description | Usage |
|--------|-------------|-------|
| Presence | Detects human presence (motion + static presence) | Primary detection sensor for occupancy |
| Micromovement | Detects subtle movements (breathing, typing) | Useful for detecting stationary people |
| Power Interference | Monitors power supply quality | Helps diagnose detection issues |

### Distance Sensor

Measures the distance to the detected person in centimeters (accuracy: ±15cm). Range varies by detection mode:
- Motion detection: 0-10m
- Micromovement detection: 0-6m 
- Static presence detection: 0-5m

### Diagnostic Sensors

Available in complete configuration for troubleshooting:

#### Energy Gate Sensors
Display raw signal strength at different distances, useful for:
- Visualizing detection range and sensitivity
- Identifying false detection sources
- Troubleshooting detection issues

#### Threshold Sensors
Show the configured threshold values for each gate:
- Motion thresholds: Controls sensitivity for large movements
- Micromotion thresholds: Controls sensitivity for subtle movements

### Text Sensors

| Sensor | Description | Usage |
|--------|-------------|-------|
| Firmware Version | Displays the radar module's firmware | Useful for compatibility troubleshooting |
| Operating Mode | Shows current mode (Normal/Engineering/Config) | Indicates radar operating status |

## Control Functions

### Basic Controls

| Button | Function | When to Use |
|--------|----------|-------------|
| Calibrate | Auto-calibrates based on environment | After installation or environment changes |
| Auto Gain | Adjusts signal amplification | When experiencing poor detection range |
| Save Config | Stores settings to radar's flash memory | After changing any settings |
| Reset Settings | Restores default detection thresholds | When detection becomes unreliable |

### Advanced Controls

| Control | Function | Usage |
|---------|----------|-------|
| Generate Thresholds with Sensitivity Inputs | Creates custom threshold settings | Fine-tuning detection sensitivity |
| Set Engineering Mode | Enables detailed signal data | Troubleshooting and advanced configuration |
| Set Normal Mode | Returns to standard operation | After completing engineering diagnostics |
| Read Motion/Micromotion Thresholds | Updates threshold display values | When thresholds appear outdated |

### Sensitivity Settings

| Setting | Description | Recommended Range |
|---------|-------------|-------------------|
| Trigger Multiplier | Controls initial motion detection | 2.0-5.0 (higher = less sensitive) |
| Hold Multiplier | Controls presence retention | 2.0-4.0 (higher = shorter retention) |
| Micromotion Multiplier | Controls small motion sensitivity | 3.0-6.0 (higher = less sensitive) |

## Calibration Guide

### When to Calibrate
- After initial installation
- When changing the radar's mounting position
- After significant changes to the room layout
- If experiencing false detections or missed detections

### Calibration Procedure
1. Ensure the room is empty of people and moving objects
2. Press "Calibrate" button
3. Wait for calibration to complete (10-15 seconds)
4. Press "Save Config" to store calibration permanently

### Advanced Calibration
For environments with special requirements:
1. Adjust the sensitivity multipliers based on your needs:
   - Lower values (1.0-2.0): Higher sensitivity, good for large rooms
   - Medium values (2.0-4.0): Balanced detection, suitable for most rooms
   - Higher values (4.0+): Lower sensitivity, reduces false positives
2. Press "Generate Thresholds with Sensitivity Inputs"
3. Wait for completion
4. Press "Save Config"

## Threshold Management

Each distance "gate" (0.7m segment) has independent threshold settings for:
- Motion detection (larger movements)
- Micromotion detection (subtle movements)

### Threshold Adjustment
You can modify individual gate thresholds:
1. Switch to complete configuration
2. Adjust the slider for the specific gate and type (motion/micromotion)
3. Press "Save Config" to store changes

### Recommended Threshold Values
- Motion thresholds: 40-60 dB (lower = more sensitive)
- Micromotion thresholds: 35-50 dB (lower = more sensitive)
- Gates farther from the sensor typically need lower thresholds

## Engineering Mode

Engineering mode provides direct access to raw radar data for advanced troubleshooting and calibration.

### Using Engineering Mode
1. Press "Set Engineering Mode"
2. View real-time energy values for each distance gate
3. The readings show reflected signal strength in dB
4. Higher values indicate stronger reflections (potential detection)
5. Return to normal mode by pressing "Set Normal Mode"

### Interpreting Energy Values
- Background noise: Typically 10-25 dB
- Human presence: Usually causes 10-30 dB increase above background
- Values consistently above threshold trigger detection
- Compare energy values to threshold values to understand detection behavior

## Common Issues & Solutions

### No Detection / Poor Range
- **Check power supply** - The module requires clean 3.3V or 5V power
- **Run Auto Gain** - Adjusts amplification for your environment
- **Lower thresholds** for affected gates
- **Verify installation height** - Should be 1.5-2.0m for wall mount, 2.7-3.0m for ceiling

### False Detections
- **Recalibrate** with room empty
- **Increase thresholds** for problematic gates
- **Check for moving objects** (curtains, plants near vents)
- **Check for electronic interference** - Keep away from Wi-Fi routers, motors

### Detection Stops After Motion
- **Check timeout setting** - Increase if detection drops too quickly
- **Verify micromotion thresholds** - Lower for better static detection

### Power Interference Warnings
- **Use a clean power supply** - Linear power supply or good quality LDO
- **Add capacitors** to power input (100μF + 0.1μF recommended)
- **Separate power supply** from noisy components (motors, relays)

### Configuration Doesn't Save
- **Ensure Save Config completes** successfully
- **Check logs for error messages**
- **Reset the module** if persisting

## Technical Reference

### Gate Distance Reference

| Gate Index | Distance Range |
|------------|---------------|
| 0 | 0.0m - 0.7m |
| 1 | 0.7m - 1.4m |
| 2 | 1.4m - 2.1m |
| 3 | 2.1m - 2.8m |
| 4 | 2.8m - 3.5m |
| 5 | 3.5m - 4.2m |
| 6 | 4.2m - 4.9m |
| 7 | 4.9m - 5.6m |
| 8 | 5.6m - 6.3m |
| 9 | 6.3m - 7.0m |
| 10 | 7.0m - 7.7m |
| 11 | 7.7m - 8.4m |
| 12 | 8.4m - 9.1m |
| 13 | 9.1m - 9.8m |
| 14 | 9.8m - 10.5m |

### Detection Range by Installation

**Wall Mount Configuration**:
- Movement detection: up to 10m
- Micromovement detection: up to 6m
- Static presence: up to 5m
- Detection angle: ±60°

**Ceiling Mount Configuration**:
- Movement detection: up to 5m radius
- Micromovement detection: up to 4m radius
- Static presence: up to 4m radius
- Static lying person: up to 3m radius

### Configuration Requirements

For optimal performance:
- UART TX/RX pins should be hardware UART, not software emulated
- Baud rate must be exactly 115200, 8N1
- Power supply should be stable 3.3V or 5V with proper regulation
- Allow 30-60 seconds after power-up for radar module initialization
- Mount securely to avoid vibrations that trigger false detections

## License

This ESPHome component is released under the  GNU GENERAL PUBLIC LICENSE Version 3.

## Credits

- Based on Hi-Link HLK-LD2402 technical documentation
- Developed for integration with ESPHome and Home Assistant
