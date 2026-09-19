# Problem Statement

## One-line problem

A vehicle gateway must continue servicing time-critical information even when multiple ECU data streams compete for CPU, queue and communication resources.

## Real-world motivation

A gateway sits between heterogeneous vehicle subsystems. A temperature report, a diagnostic record and an event that may require immediate vehicle action should not be treated as equally urgent when resources are constrained.

The prototype therefore studies **deterministic degradation**:

> When the gateway is overloaded, can it discard or delay low-criticality information before allowing it to compromise the safety-critical processing path?

## What the prototype proves

1. Heterogeneous ECU traffic can be ingested by a QNX gateway.
2. Traffic can be separated into criticality-specific queues.
3. QNX priority scheduling can enforce a deterministic ordering among runnable workloads.
4. Low-priority telemetry can be shed at a defined queue threshold.
5. Safety-path timing can be measured rather than asserted.
6. The architecture can evolve from fixed message classification toward sensor-value-derived criticality.

## What it does not prove

- Production collision avoidance.
- Functional-safety certification.
- Guaranteed vehicle-level end-to-end latency.
- Production CAN network scalability.
- That QNX is the only possible RTOS for this problem.
