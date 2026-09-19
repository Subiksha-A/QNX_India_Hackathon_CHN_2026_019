# MCXN236 Traffic / Congestion ECU

The current stress setup uses the MCXN236 as the CAN0 traffic generator feeding the Raspberry Pi gateway.

In the gateway-side code, synthetic congestion traffic is represented by CAN ID `0x450`. Some gateway versions include the congestion generator directly inside the QNX application rather than as a standalone MCXN236 firmware file.

If the dedicated MCXN236 firmware is recovered, place it here and document its:

- CAN bitrate
- message IDs
- payload format
- frame generation period/burst size
- start/stop mechanism
- pin/board configuration

Until then, this README is the source-of-truth note; no firmware is invented here.
