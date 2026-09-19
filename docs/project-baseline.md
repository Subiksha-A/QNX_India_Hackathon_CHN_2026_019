# Project Baseline — Numbers We Can Defend

These numbers are **prototype configuration or measured-target parameters**, not universal automotive specifications.

| Item | Value | Meaning |
|---|---:|---|
| CAN bitrate | **500 kbps** | Configured classic CAN rate in the gateway/MCXA156 sources |
| Safety deadline target | **5 ms** | Experimental gateway safety-path target |
| Telemetry shed threshold | **85%** | Queue occupancy at which low-priority telemetry is discarded |
| Congestion workload | **300 frames** | Finite stress workload used by the congestion demonstration |
| Safety queue | **64 frames** | Bounded safety queue in sensor-aware/master variants |
| Normal queue | **128 frames** | Bounded normal queue in sensor-aware/master variants |
| Noise/telemetry queue | **256 frames** | Bounded low-priority queue in sensor-aware/master variants |
| Error queue | **512 frames** | Bounded diagnostic/error storage in expanded variants |
| QNX safety priority | **50** | Highest application priority in the baseline corrected gateway |
| QNX CAN RX priority | **40** | Hardware ingestion/classification |
| QNX gateway priority | **30** | Normal routing |
| QNX diagnostics priority | **15** | Monitoring/logging |

The 500 kbps configuration, 5 ms target, 85% threshold and thread priorities are present in the gateway source. fileciteturn10file0L68-L90

The sensor-aware source defines 64/128/256/512-frame queue capacities and the 85% noise-shed threshold. fileciteturn10file1L215-L222

The finite 300-frame congestion demonstration is present in the gateway stress logic. fileciteturn10file8L994-L1011

## External automotive facts

- AUTOSAR's CAN Driver specification states that CAN L-PDU priority is represented by the CAN identifier and that a lower numerical identifier means higher priority. See the official AUTOSAR CAN Driver specification.
- QNX documents priority-based preemptive real-time scheduling and priority inheritance as core real-time mechanisms.

External references are collected in `docs/sources.md`.
