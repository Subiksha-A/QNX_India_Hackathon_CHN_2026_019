# Current Physical Topology

## Gateway

**Raspberry Pi 4/5 + QNX Neutrino**

## CAN interfaces

The current physical arrangement for the final demo is:

- **CAN1:** MCP2515 CAN HAT ↔ FRDM-MCXA156 sensor ECU.
- **CAN0:** MCP2515 CAN HAT ↔ MCXN236 congestion/traffic ECU.

The MCP2515 controllers are accessed by the QNX application through the SPI resource-manager paths used by the gateway code. Historical sources may label the two logical channels differently; the current physical wiring is authoritative for the final demo.

## MCXA156

The supplied MCXA156 source implements CAN at 500 kbps and provides temperature, humidity, speed, battery voltage/current, ultrasonic and IR messages. The ultrasonic sensor uses TRIG=P3_27 and ECHO=P3_28 in that source; CAN RX/TX are P1_12/P1_13 and IR is P1_14. fileciteturn13file0L11-L28

## MCXN236

Used as the controlled CAN traffic/congestion generator for the final stress test.

## ESP32 LiDAR

The final architecture includes an ESP32 LiDAR ECU connected to the Raspberry Pi through UART. The source code for that firmware was not found in the current repository/file archive search, so it is intentionally **not fabricated** here. Add the actual firmware under `src/ecu/esp32_lidar/` when recovered.
