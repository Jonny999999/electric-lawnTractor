# Throttle Tune – Throttle Adapter for BLDC Motor Controllers

A microcontroller-based board with a PWM-based analog output (RC filter + buffer) to emulate a DAC.
The goal is to improve throttle response and add configurable driving modes to cheap BLDC motor controllers (e.g. 3000 W BLDC "go-kart kit" controllers).  

Currently it is used in the [electric Kettcar conversion](https://pfusch.zone/electric-kettcar.html) with a 3000 W BLDC drive, this board fixes the controller’s unusable throttle mapping and makes the vehicle smooth and controllable adding a lot of flexibility.

<img src="doc/images/2025.04.21_control-box.jpg" width="45%">  

*Control box with the custom pcb connected between throttle and motor controller*  
*Contains: PCB, key switch, battery indicator, several UI switches*

---

## Problem

Cheap BLDC controllers (such as KT-style 3000 W kits) have several issues:
- No configuration options at all
- Speed levels provided by the controller are usually defined poorly
- Only ~2 cm (~60 %) of gas pedal travel is actually used
- Jerky, hard-to-control acceleration (tiny pedal movements cause extreme changes in power - positive feedback when accelerating thus pressing the pedal more makes this even worse)

Using only the original setup makes smooth low-speed driving or child-friendly operation nearly impossible


---


## Solution

The **Throttle Tune** board sits between the analog throttle pedal (or handle) and the motor controller.  
It rescales, filters, and manipulates the signal (fading, modes...) while adding safety and convenience features.


---


## Features

### Firmware
- **Throttle remap**: use full pedal range (configure in/out ranges)
- **Ramp-up / fade**: prevents stuttering, smoother control (configurable max rate of change)
- **Speed modes**: 3 levels + hidden "Sport" mode (highly configurable - several parameters on per-mode-basis)
- **Reverse scaling**: reduced throttle in reverse
- **Buzzer alerts**: startup-, reverse-, mode switch beeps

### Hardware
- ATmega8 microcontroller
- Custom DAC output (0–5 V) to motor controller (implementation: PWM output -> lowpass -> buffer)
- CNC-milled single-sided PCB
- Through-hole components (easy to build/repair)
- 3D-printed enclosure
- Inputs: Hall pedal, reverse switch, speed selector
- Outputs: DAC to controller, buzzer


---


## Hardware

### Schematic
[pcb_throttle-tune/export/schematic.pdf](pcb_throttle-tune/export/schematic.pdf)  
<img src="pcb_throttle-tune/export/schematic.svg" width="55%">

### Layout
<img src="pcb_throttle-tune/export/layout.svg" width="55%">

### PCB Photo
<img src="doc/images/pcb.jpg" width="49%">


---


## Example Applications

### Electric Kettcar
Documentation: [pfusch.zone/electric-kettcar](https://pfusch.zone/electric-kettcar)

<img src="doc/images/2025.04.21_kettcar.jpg" width="54%">
<img src="doc/images/2025.04.21_kettcar_pcb.jpg" width="42%">

*Wiring of PCB between pedal and controller (right) and the finished vehicle (left).*


---


### Electric Lawn Tractor (temporary use-case)
Documentation: [pfusch.zone/electric-lawn-tractor](https://pfusch.zone/electric-lawn-tractor)

<img src="doc/images/2024.08.20_lawn-tractor_box.jpg" width="60%">

*Throttle Tune board inside in box on top of the motor controller, battery powered (earlier project - initial but temporary use).*


---


## Repository Content
- `pcb_throttle-tune/` – KiCad project, schematic, layout exports
- `fw_throttle-tune/` – AVR firmware (ATmega8)
- `doc/images` – photos of the build and control box


---


## Usage

### Wiring
See [pcb_throttle-tune/export/layout.pdf](pcb_throttle-tune/export/layout.pdf) for what needs to be connected to which terminals.

### Adjust thresholds
Several config options (macro variables) are defined at the top of `main.c` (see detailed comments).
These include controller start/max voltage, pedal min/max voltage, idle voltage, etc.  
To calibrate:  
- Wire everything up  
- Enable `DEBUG_UART_DEBUG_OUTPUT_ENABLED`  
- Build and flash the firmware
- Connect a USB-UART adapter (header pins next to mcu, TX + GND are enough)  
- Play with the throttle and observe the UART logs to determine the values  
- Adjust the macros in `main.c` accordingly, compile, flash, try again

### Configure / Adjust modes
Driving modes are selected via 2 GPIO pins, each with its own config in the `modeConfigs[]` array.  
Parameters can be tuned per mode using this struct:
```c
//=============================
//==== Mode configuration =====
//=============================
typedef struct {
    const char *name;
    uint8_t maxPercent;     // percent of max possible speed applied at full throttle
    uint8_t maxPercentReverse;
    uint8_t beepCount;      // count beeped when entering this mode
    uint16_t rampUpStep;    // max duty increment per interval (max 1023)
    uint16_t rampUpIntervalMs; // note: must be larger than cycle time (consider when UART used alot)
    uint8_t pedalAverageWindowSize; // window size of the moving average to smooth pedal input (0 = disabled)
    // Future: pedal averaging, throttle curve, etc.
} kettcarConfig_t;
```
See comments in main.c and adjust as needed.

### Build & Flash
#### Install Requirements
```bash
yay -S avr-gcc avr-libc make
```

#### Build
```bash
cd fw_throttle-tune/
make
```

#### Flash
Connect avr controller using USBASP V2.0 ISP programmer using the 10pin connector on the pcb
```bash
cd fw_throttle-tune/
make                # compile firmware first
sudo make upload    # flash to MCU via USBasp
```
Note: Be sure to recompile before flashing, or you may upload an old build...


---


## TODO (future hardware rev)
- Add TVS diode  
- Add buzzer footprint (currently wired to spare pin and implemented in firmware, but not documented in pcb design)  
- Drop unused terminals (see notes design)
- Add reverse polarity protection
- Add onboard buck converter to be used with e.g. 72V directly not requiring external converter
