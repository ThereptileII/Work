#pragma once
#include <stdint.h>

namespace config {
// Confirmed target: standard classic ESP32 dev board + original Autohelm ST4000.
// Reported interface: generic HV/LV converter, HV=12 V and LV=3.3 V.
// Default assumes the usual bidirectional BSS138 circuit; verify the board.
constexpr int canTx = 5, canRx = 4;
constexpr int seaTalkTx = 17, seaTalkRx = 16;
constexpr int seaTalkTxMonitor = 18; // Hardware waveform copy; LEAVE UNCONNECTED.
enum class SeaTalkInterface { SharedMosfet, SeparateInvertingTx };
constexpr SeaTalkInterface seaTalkInterface = SeaTalkInterface::SharedMosfet;
constexpr bool txOpenDrain = seaTalkInterface == SeaTalkInterface::SharedMosfet;
// Shared LV signal: LOW pulls down, HIGH releases. Separate NPN: HIGH pulls down.
constexpr bool txInverted = !txOpenDrain;
constexpr bool rxInverted = false;
static_assert(!txOpenDrain || !rxInverted, "Shared MOSFET RX must be non-inverted");
static_assert(seaTalkTx != seaTalkRx && seaTalkTx != seaTalkTxMonitor &&
              seaTalkRx != seaTalkTxMonitor && canTx != canRx &&
              seaTalkTx != canTx && seaTalkTx != canRx &&
              seaTalkRx != canTx && seaTalkRx != canRx &&
              seaTalkTxMonitor != canTx && seaTalkTxMonitor != canRx,
              "CAN, SeaTalk and monitor GPIOs must be distinct");
constexpr bool controlEnabled = true;
constexpr bool seaTalkListenOnly = false; // Normal operation; serial "listen on" disables TX until reboot.
constexpr bool publishRudder = false; // Enable only after validating a real sensor.
constexpr uint8_t navigationSource = 255; // 255: lock to first valid nav sender.
constexpr uint8_t windSource = 255;
constexpr uint8_t controllerSource = 255; // 255: any correctly addressed controller.
constexpr bool autoTrackCompatibility = true; // OpenCPN AutoTrackRaymarine 2.3 Evolution profile.
constexpr uint8_t preferredAddress = autoTrackCompatibility?204:33;
constexpr uint32_t feedbackTimeoutMs = 3000;
constexpr uint32_t navigationTimeoutMs = 3000;
constexpr uint32_t windTimeoutMs = 3000;
constexpr uint32_t variationTimeoutMs = 60000;
constexpr uint32_t commandTimeoutMs = 60000;
constexpr uint32_t confirmationTimeoutMs = 3500;
constexpr uint32_t queueTimeoutMs = 1500;
constexpr uint16_t manufacturer = 1851; // Raymarine compatibility, not certification.
}
