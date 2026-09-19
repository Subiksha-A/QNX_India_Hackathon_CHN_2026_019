# Changelog / Engineering Narrative

## Phase 01 — Real-time gateway foundation

Established QNX `SCHED_FIFO`, bounded queues, MCP2515 SPI access, telemetry and timing instrumentation.

## Phase 02 — Priority isolation

Separated safety, normal and telemetry paths and explicitly protected the safety thread from lower-priority queueing work.

## Phase 03 — Sensor-aware gateway

Integrated the MCXA156 sensor protocol and added application-level safety detection for ultrasonic/IR conditions.

## Phase 04 — Expanded multicore/Momentics architecture

Added stronger multicore/runmask concepts, additional transmit/receive scheduling, congestion generation and fault-analysis storage.

## Phase 05 — Final architecture direction

Move from fixed CAN-ID criticality toward a deterministic sensor-derived severity score. This is the next implementation step and is not to be represented as complete until tested.
