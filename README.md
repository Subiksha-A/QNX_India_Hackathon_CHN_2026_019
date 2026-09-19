# QNX Vehicle Gateway — Deterministic Automotive Gateway Prototype

> **From CAN message routing to criticality-aware real-time execution.**

This repository documents the complete engineering evolution of our QNX vehicle-gateway prototype. It intentionally contains **multiple versions**, not only the final code, because the progression is part of the project: we started by proving CAN routing and priority separation, introduced deterministic load shedding and timing instrumentation, integrated the MCXA156 sensor ECU, and then moved toward a sensor-value-driven criticality model for the final architecture.

## 1. The problem

Modern vehicle gateways bring together information from multiple ECUs and interfaces. Not every message has the same timing importance. Under normal load, ordinary telemetry, diagnostics and vehicle-state traffic can coexist. Under congestion, however, a gateway should not allow low-criticality work to delay the processing of information that can require an immediate vehicle response.

Our prototype asks a focused real-time systems question:

> **When heterogeneous ECU traffic competes for gateway resources, can a QNX-based gateway preserve a bounded response path for safety-critical information while degrading low-criticality traffic first?**

The prototype uses a Raspberry Pi running QNX Neutrino as the gateway, with NXP sensor/traffic nodes representing vehicle ECUs. The current physical test arrangement is:

- **CAN1:** MCXA156 sensor ECU → Raspberry Pi through the MCP2515 CAN HAT.
- **CAN0:** MCXN236 traffic/congestion generator → Raspberry Pi through the MCP2515 CAN HAT.
- **LiDAR ECU:** ESP32 → Raspberry Pi through UART (integration source is not included in this repository yet; see `src/ecu/esp32_lidar/README.md`).

The gateway-side code has historically also supported a dual-CAN configuration in which CAN0 and CAN1 were treated as separate vehicle networks. Those versions are preserved under `src/qnx/history/`.

## 2. Why the problem matters

This is not a claim that the prototype is a production automotive safety system. It is a controlled demonstration of principles that matter in automotive real-time software:

- **CAN arbitration:** AUTOSAR defines CAN L-PDU priority through the CAN identifier; lower numerical identifiers have higher priority. https://www.autosar.org/fileadmin/standards/R24-11/CP/AUTOSAR_CP_SWS_CANDriver.pdf
- **ECU software scheduling:** QNX uses priority-based preemptive thread scheduling, and its architecture is designed around predictable real-time execution. https://www.qnx.com/developers/articles/article_298_1.html
- **Fault isolation:** QNX Neutrino places drivers and services outside the microkernel in protected user-space processes, supporting fault isolation and restartable components. https://www.qnx.com/products/intl/neutrino_rtos/

The project therefore separates three different decisions:

```text
Physical / application state
        ↓
"How critical is this information?"
        ↓
QNX thread scheduling
"Which processing workload runs first?"
        ↓
CAN transmission
"Which CAN frame wins bus arbitration?"
```

These are related but are **not the same mechanism**.

## 3. Project evolution

| Stage | Main question | Key mechanism | Repository location |
|---|---|---|---|
| 01 — RT gateway foundation | Can we create a deterministic gateway path? | QNX `SCHED_FIFO`, bounded queues, CAN/SPI | `src/qnx/history/01_*` |
| 02 — Corrected priority architecture | Can safety traffic be isolated from normal/telemetry traffic? | Safety/Normal/Telemetry queues, P50/P30/P15 | `src/qnx/history/02_*` |
| 03 — Sensor-aware gateway | Can physical sensor state drive safety decisions? | MCXA156 ultrasonic/IR decoding and safety policy | `src/qnx/history/03_*` + `src/ecu/mcxa156/` |
| 04 — Momentics / multicore architecture | Can the gateway expose stronger resource isolation and engineering instrumentation? | CPU/runmask architecture, multiple RT threads, diagnostics, congestion generation | `src/qnx/history/04_*` |
| 05 — Final architecture direction | Can sensor values be mapped to a deterministic severity score before QNX scheduling? | LiDAR + ultrasonic + temperature → severity → criticality class → QNX priority | `src/qnx/proposed/` |

**Important:** the files are preserved as engineering snapshots. The exact chronological order is not asserted beyond what the source headers and project discussions establish; use `docs/evolution/` for the intent of each stage.

## 4. What QNX contributes

QNX is particularly well matched to this prototype because the project needs **priority-based preemption, bounded execution paths, resource-manager access to hardware, POSIX threading, priority inheritance, process isolation and real-time instrumentation**. QNX documents priority-based scheduling, preemption and priority inheritance as core real-time mechanisms. 
https://www.qnx.com/developers/articles/article_298_1.html

The project deliberately uses QNX features rather than treating QNX as simply a Linux replacement:

- `SCHED_FIFO` and explicit thread priorities
- priority-based preemption
- priority-inheritance mutexes
- QNX SPI resource-manager access to MCP2515 controllers
- QNX native channels/pulses where used by the selected version
- `CLOCK_MONOTONIC` for timing
- per-thread CPU-time measurement
- isolated bounded queues
- deterministic load shedding
- CPU/runmask or affinity mechanisms in the versions that implement them
- user-space service/resource-manager architecture

QNX is **not literally irreplaceable** for every implementation of this problem. The defensible claim is that its architecture is unusually well aligned with the project's requirement for deterministic priority-driven execution and fault/resource isolation. A production system would still require platform, safety, timing, certification and network-stack validation.

## 5. Current scheduling model

The established gateway versions use the following hierarchy:

| Workload | Priority | Role |
|---|---:|---|
| Safety | 50 | Highest-priority safety processing |
| CAN RX | 40 | Hardware ingestion and classification |
| Gateway | 30 | Normal routing |
| Diagnostics | 15 | Logging/telemetry/monitoring |

The baseline code also defines a **5 ms safety deadline target** and an **85% telemetry queue load-shedding threshold**. These are experimental design parameters, not universal automotive limits.

## 6. Stress experiment

The project uses controlled CAN congestion rather than claiming a cybersecurity exploit. A low-priority congestion workload is generated and the gateway is observed for:

- safety-path latency
- maximum/average latency and jitter
- queue occupancy
- dropped low-priority traffic
- safety drops/deadline misses
- CAN bus load
- CPU utilisation

The repository should contain **measured results only** under `results/`. Dashboard demo values are explicitly synthetic and must never be presented as jury measurements. The dashboard source itself warns that demo data is for presentation development and that actual jury measurements should come from the QNX gateway.

## 7. Final design direction: criticality-aware scheduling

The original gateway classified CAN frames largely from their identifiers. That is useful for demonstrating communication priority, but it does not by itself express the urgency of the current physical state.

The final architecture therefore moves toward:

```text
LiDAR distance / object information
Ultrasonic distance
Temperature
        ↓
Severity evaluator
        ↓
Severity score
        ↓
Safety / Warning / Normal
        ↓
QNX priority class
        ↓
Deterministic processing / routing
```

For the final prototype, this evaluator should remain **deterministic and rule-based**. It should not be described as an ML model or as a production collision-avoidance algorithm.

The implementation plan is documented in `src/qnx/proposed/` and `docs/decisions/criticality-scoring.md`. Until that code is merged and tested, it is explicitly marked **proposed** rather than implemented.

## 8. Repository philosophy

This repository is deliberately structured as an engineering narrative:

**Problem → requirements → first architecture → failures/corrections → sensor integration → stress testing → final criticality model → measured evidence → automotive comparison.**

Do not delete old versions merely because they are superseded. They show why the final architecture exists.

## 9. Reproducibility

See:

- `docs/BUILD.md` — QNX and dashboard build/run instructions
- `docs/hardware/current-topology.md` — current physical setup
- `docs/evolution/` — design evolution
- `docs/experiments/stress-test.md` — congestion test protocol
- `docs/automotive/comparison.md` — CAN arbitration vs ECU scheduling
- `docs/decisions/criticality-scoring.md` — final severity-score design
- `results/README.md` — how measured evidence is stored

## 10. Status

**Prototype / hackathon engineering project.**

This repository is not a certified automotive safety product and does not claim ISO 26262 compliance, production ECU qualification, or a guaranteed end-to-end 5 ms vehicle response.
