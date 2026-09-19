# ESP32 LiDAR ECU

**Status: source pending recovery.**

The final hardware architecture uses an ESP32 as the LiDAR ECU and sends LiDAR information to the Raspberry Pi gateway over UART.

No authoritative ESP32 LiDAR source file was found in the available project file archive during repository assembly. This directory is therefore intentionally a placeholder rather than invented code.

When the actual firmware is recovered, add:

- sensor driver / UART setup
- packet format
- object-distance fields
- checksum/framing if used
- update period
- pin mapping
- build instructions

Do not replace this file with generated or assumed firmware without marking it as a new version.
