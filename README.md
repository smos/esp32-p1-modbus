This is a ESP32 project vibe coded (CoPilot, Gemini)to perform the following:

Connect to P1 DMSR5 meter via TCP or Serial as input.

Output available via
- Modbus energy meter via TCP 502, Eastron SDM72 or Fronius SunPower
- TCP Server via TCP 8088 as raw P1 datagrams
- MQTT
- Livedata via WebUI
- Expects serial ingest via GPIO16 or GPIO18 depending on ESP32 or ESP32S3
- Broadcasts SMA/Speedwire to multicast or unicast
- Runtime statistics with median update interval
- Pushes current readings to SmartEVSE REST API endpoint
- Watchdog reboots when no P1 data for 60s
- HomeWizard HWE-P1 iscovery, API v1 /api/data endpoint

Possible to update firmware via WebUI on config page

By no means perfect, but room in the distribution panel was limited, and with all the energy systems wanting their own energy meter in the distribution panel that seems silly.
Flashes led on parsing of the P1 datagram, which should be once per second.

By also providing a TCP server interface for the datagrams, integration with HomeAsisstant still works. Also handy during development to daisychain another ESP off it.

Added a printable lipstick enclosure for the Waveshare esp32-s3-eth-poe board, as long as the pins are not soldered. See the enclosure directory for a Openscad drawing and STL files.

Initial flashing with the Espressif upload tool.
https://github.com/smos/esp32-p1-modbus/blob/main/screenshots/flasher.png

Some runtime statistics displayed for mean process/push latency
https://github.com/smos/esp32-p1-modbus/blob/main/screenshots/.png

Live data displayed on the Status tab
https://github.com/smos/esp32-p1-modbus/blob/main/screenshots/livedata.png

Config for general settings, default pin 1234.
https://github.com/smos/esp32-p1-modbus/blob/main/screenshots/config.png

SmartEVSE config option for pushing data straight from P1 bridge to the EVSE without extra steps.
https://github.com/smos/esp32-p1-modbus/blob/main/screenshots/smartevse.png
