# Boat propulsion bridge — Beta marine contract

This is an explicit adapter for the inspected ESP32-C6 marine producer, not a PC
EV-CAN decoder. OpenCPN owns the connection and N2K reassembly. The adapter sees
copied marine messages and an actual observed 60928 NAME. Bind the exact
interface/NAME under Data Sources; source address alone is insufficient.
Manufacturer 2046, function 130, class 25, industry 4 identify the inspected
device class, not cryptographic authentication or a guarantee of firmware.

## Field meanings

- Standard 127506 supplies SOC. 127508 contains V/I, not SOC. 127751 carries
  the high-voltage pack at unsigned 0.1 V and signed 0.01 A. Its pair is preferred
  over 127508. Positive source current charges the pack; configure that convention
  explicitly before deriving whole-pack discharge power. This is not motor-only
  electrical or shaft power.
- The inspected instance-zero 127489 coolant field carries motor temperature.
  Only the explicitly bound bridge maps it to motor temperature. Generic devices
  retain standard coolant meaning. No winding-temperature claim is made.
- The instance-zero 127505 fuel level is a compatibility presentation of SOC with
  fictional capacity. The bound adapter suppresses this physical-tank presentation;
  SOC comes from 127506. Other devices/real tank instances remain available.
- Vendor 61184 regeneration values 0/1/2 mean Off/One bar/Two bars. They describe
  the reported setting, not proof of actual regenerative current or torque.
  Current/power bytes in this vendor message are not used as an additional power
  estimate. Missing/invalid regeneration stays unavailable.

## Producer freshness repair

Pinned source `9baf01bca09522794a9678dfb3f3c0720d5c9943`, file
`Nissan_ev_to_NMEA_and_wifi.ino`, SHA-256
`a7fa8d8ad1c26f5f72401f84e42996242d375822f301f2ad4a930b5e718fc53d`,
retransmitted retained values indefinitely. Changing SID or recent PC receipt
does not prove a fresh EV observation. A gateway unplug test cannot detect this.

`hardware/leaf-bridge/boat-bridge-v2-expiry.patch` and `FreshTelemetry.h` repair
the boat-side producer. Independent timestamps cover power/SOC, RPM, temperature
and gear/regeneration. At 2500 ms each group expires and its periodic marine
fields publish NA. A gear update cannot keep battery data live. Snapshotting and
expiry occur under the existing telemetry lock, including web display snapshots.
Expiry latches until new input, including across the millisecond-counter wrap.
The patch also corrects 127751 missing voltage to unsigned `0xffff`.

61184 remains eight bytes. Byte 1 becomes version 2. Byte 3 keeps original low
bits (0 regen available, 1 current available, 2 power available; bit 3 reserved)
and adds group freshness bits 4 power/SOC, 5 RPM, 6 temperature, 7 gear/regen.
The periodic stream is 10 Hz. The PC requires a v2 heartbeat less than 500 ms old
and the relevant group bit; loss/old firmware makes readings uncertain, and an
expired group suppresses its values. Recovery requires new observations after
the expiry transition. Read/paint cannot renew timestamps. These bounds describe
producer expiry plus observed transport age, not exact original sensor latency.

The existing v1 bridge remains inspectable through the bound adapter, with
UNCERTAIN values and dependent battery power/energy withheld. Binding removal
returns generic PGN meanings and clears candidates; do not interpret the known
virtual fuel tank as a real fuel measurement. There is no silent firmware update.

## Build and qualification

`python3 tools/prepare-boat-firmware.py` downloads only the hash-pinned source,
checks/applies the reviewed patch in a disposable repository and creates a host
oracle from the actual producer's PGN functions. Both desktop contract platforms
compile those functions and verify NA, voltage, v2 extension interoperability,
independent expiry, recovery and counter wrap. The full sketch also compiles for
`esp32:esp32:XIAO_ESP32C6` using Arduino ESP32 3.2.1, CLI 1.5.1 and WebSockets 2.7.2.
Use `python3 tools/build-boat-firmware-linux.py` for the isolated reproducible
compile. It never uploads or flashes. Tool/package state stays under `.local/`.
Board setup follows [Espressif's installation documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
and [Seeed's XIAO ESP32-C6 board instructions](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/).

Physical flashing/acceptance remains a separate deliberate dockside procedure:
back up the existing firmware/settings, verify board/pins and network settings,
compile the exact prepared source, and have a recovery method before uploading.
Verify each sensor group independently, then stop its EV input while keeping N2K
connected. Compare NA, mask, PC age and suppressed energy against instruments.
Verify normal recovery requires new EV input. No steering code is changed.
The baseline sketch's Wi-Fi/switch-control configuration is not hardened by this
telemetry patch; review its existing default credentials and access aboard the
boat before deployment. No physical success is claimed from the compile or tests.
