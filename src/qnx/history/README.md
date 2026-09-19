# QNX Source Snapshots

These files are preserved as snapshots. Do not rename them to `final.c` and delete the rest; the repository's purpose is to show the engineering evolution.

| Snapshot | Intent |
|---|---|
| `01_qnx_vehicle_gateway_realtime.c` | Real-time gateway foundation: SCHED_FIFO, bounded queues, MCP2515, load-shedding/metrics, affinity/telemetry concepts |
| `02_QNX_GATEWAY_MASTER_CORRECTED.c` | Clean safety/normal/telemetry separation and fixed priority hierarchy |
| `03_qnx_master_code_sensor_aware.c` | MCXA156 protocol integration and ultrasonic/IR application safety logic |
| `04_qnx_master_momentics.c` | Expanded multicore/runmask, multiple RT threads, congestion generation and fault-analysis architecture |
| `05_updated_gateway_source.c` | Later consolidated gateway source preserving the real-time architecture |
