# Cyrus Pre Volume Replacement Project

This project provides a modern, drop-in replacement for the **motorised volume potentiometer** used in the original **Cyrus Pre preamplifier**.  
It restores full functionality while eliminating the mechanical failure points of the original design.  
The replacement board reads the original MCU’s control data directly and uses it to drive a **PGA2310 analog volume control IC**, maintaining all original features including balance and mute.

---

## Project Overview

The original Cyrus Pre used a custom motorised potentiometer controlled by a feedback loop.  
In that design, the MCU sent volume data to a stereo R-2R DAC ladder that generated a voltage reference for positioning the pot motor.  
Over time, these potentiometers often fail or become worn beyond the ability of the control loop to function.

This project replaces that system with a **digital control PCB** that intercepts the MCU’s data stream, decodes the requested volume setting, and maps it to **logarithmic dB steps**.  
The decoded levels are then sent to a **PGA2310** dual-channel analog volume IC, chosen for its:

- Direct compatibility with the existing ±15 V rails.
- Low noise and distortion.
- Wide signal level capability (no headroom loss).

Balance and mute commands remain fully functional, preserving the original user experience.

---

## Implementation Notes

- The Cyrus MCU applies per-unit potentiometer calibration, typically only ±1 step.  
  Since the original calibration no longer applies, the firmware snaps all decoded values to the nearest volume step for consistent operation.
- The replacement PCB fits the **original motorised pot mounting points** for simple installation.
- A full installation guide is included.

---

## Repository Contents

### 📂 **Cyrus Pre Volume KiCAD**
- Complete PCB design files and schematics.

### 📂 **Cyrus Pre Volume Code Arduino**
- Firmware that reads the Cyrus MCU’s shift-register data, decodes volume/balance/mute, and controls the PGA2310.

### 📂 **Cyrus Pre Audio Measurements**
- Objective test data verifying performance of the replacement circuit.

### 📂 **Cyrus Pre Images**
- Reference and build photos of the original and replacement assemblies.

### 📄 **Cyrus Pre HA7L Service Manual.pdf**
- Official service documentation for the original preamp.

### 📦 **Cyrus Pre Volume Gerbers.zip**
- Gerber files ready for PCB manufacture.

### 📄 **Cyrus Pre Volume Repair Guide.pdf**
- Installation instructions with images.

### 📄 **Cyrus Pre Volume Replacement Schematic.pdf**
- PDF schematic for the replacement board.

---

## Usage

1. Fabricate the PCB using the provided Gerber files.  
2. Program the microcontroller with the supplied Arduino firmware. (Arduino Pro Micro: Micro-USB, 5V version) 
3. Remove the original potentiometers.
4. Follow the repair guide for wiring.
5. Install the board using the existing motorised pot mounting holes.
6. Confirm operation.

---

## Credits

Project by **Simon Ashton / Tenson on DIYAudio.com**, developed to bring the Cyrus Pre preamplifier back to life with a modern, reliable volume control system.  
Four years sitting dissasembled in my draw — but finally repaired 😎
