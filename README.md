# Work Firmware Collection

This repository collects several standalone Arduino/ESP32 sketches used across the electric-boat project. Each file targets a specific controller or diagnostic task. The table below provides a quick overview, followed by per-sketch details covering hardware assumptions, communications, and notable features.

| Sketch | Purpose |
| --- | --- |
| `Motor_read_old.ino` | Legacy ESP32 motor telemetry bridge with dual CAN, Nextion display support, and wind calculations. |
| `Switchbank_relay.ino` | ESP32-based NMEA 2000 switch bank controller that drives eight relays and speaks the relevant PGNs. |
| `throttlecontroller.ino` | Dual-channel DAC throttle generator with analog smoothing and parabolic mapping. |
| `Nissan_ev_to_NMEA_and_wifi.ino` | XIAO ESP32-C6 gateway that translates Nissan EV CAN data onto NMEA 2000, drives a virtual switch bank, and hosts a Wi-Fi dashboard. |
| [autopilot-controller/](autopilot-controller/README.md) | Bidirectional ESP32 NMEA 2000 / SeaTalk-1 bridge for the original ST4000, with measured feedback and OpenCPN AutoTrack support. |
| `xboxcontroller.ino` | Bluepad32-powered wireless controller that maps Xbox gamepad input to throttle DAC output and relay control. |

## Sketch details

### `Motor_read_old.ino`
* Hardware: ESP32 with on-board TWAI and an MCP2515 transceiver for a second CAN bus plus a Nextion HMI on `Serial2`.
* Functionality: subscribes to EV CAN IDs (`0x1DB`, `0x1DA`, `0x55A`, `0x539`), exposes the data on NMEA 2000 PGNs (`128259`, `130306`), and computes true/apparent wind along with history buffers.
* Extras: maintains min/max statistics, pushes waveform data to the Nextion display, and protects shared telemetry with a FreeRTOS semaphore.

### `Switchbank_relay.ino`
* Hardware: ESP32 using the native TWAI peripheral and eight GPIO-driven relays (configurable for active-high or active-low boards).
* Functionality: implements a complete N2K switch bank node—address claims, responds to ISO requests, broadcasts 127501 status, receives 127502 control frames, and periodically sends 127504 short-form labels.
* Extras: handles boot-time "burst" updates, caches state per channel, and exposes helper routines for PGN parsing and transmission.

### `throttlecontroller.ino`
* Hardware: Arduino-compatible MCU with a TCA9548A I²C multiplexer and two MCP4725 DACs (channels 3 and 4) that produce throttle voltages.
* Functionality: reads a primary potentiometer (`A0`) with fallback (`A1`), applies exponential moving averages, and feeds both DACs with a parabolic scaling curve to match target voltage ranges (0.76–3.8 V and 0.38–1.9 V).
* Extras: prints consolidated diagnostics (raw readings, filtered values, DAC codes, computed voltages) at 10 Hz.

### `Nissan_ev_to_NMEA_and_wifi.ino`
* Hardware: Seeed XIAO ESP32-C6 using two independent TWAI controllers (500 kbps EV bus, 250 kbps N2K bus) plus a captive-portal Wi-Fi soft AP serving HTML/JS dashboards.
* Functionality: reads Nissan EV CAN metrics and publishes them as engine-related N2K PGNs, implements a 24-channel switch bank (60928/59904/127501/127502/127504), and serves both a responsive dashboard and switch control page with a WebSocket JSON stream.
* Extras: provides WebSocket commands to adjust broadcast cadence, tracks connected clients, and keeps consistent string representations for gear/regen states used by the UI.

### [Autopilot controller](autopilot-controller/README.md)

* Hardware: classic ESP32 with CAN GPIO5/4 and a shared level-shifted SeaTalk signal on TX17/RX16. GPIO18 is reserved for an unconnected transmission monitor.
* Functionality: reports measured pilot mode and heading, accepts feedback-confirmed remote controls, and forwards valid navigation, apparent wind and water speed. Includes OpenCPN AutoTrackRaymarine compatibility.
* Build and tests: this is now a complete PlatformIO project. See its [README](autopilot-controller/README.md) for wiring, configuration, build/upload commands and host tests. The former root-level sketch has been replaced by this project.
* Commissioning: SeaTalk transmission starts enabled; rudder reporting remains disabled. Dockside AUTO, +1-degree heading and remote STANDBY tests passed. Intermittent background wire errors remain unresolved; compass alignment and underway route/waypoint tests remain outstanding.

### `xboxcontroller.ino`
* Hardware: ESP32 running the Bluepad32 stack, driving DAC1 (`GPIO25`) for throttle voltage plus relays on GPIO33/26/27/14/32/35 for continuous control, gear selection, and joystick-based aux functions.
* Functionality: maps controller buttons/D-pad/axes to throttle percentages via an exponential curve, pulses gear relays with debounced inputs, and toggles horizontal joystick relays using hysteresis.
* Extras: prints periodic status messages with gear, throttle, DAC value, and raw axis readings; supports up to four simultaneous gamepads.

## Getting started

All sketches target the Arduino ecosystem. Install the required libraries noted at the top of each file (e.g., `Bluepad32`, `Adafruit_MCP4725`, `arduinoWebSockets`). Adjust pin assignments and CAN transceiver wiring to match your hardware, then compile and upload the sketch that fits your use case.


For the autopilot, run build and test commands from `autopilot-controller/` using its pinned PlatformIO dependencies.
