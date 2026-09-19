# Engineering Evolution

The repository preserves versions because the design itself is part of the evidence.

## 01 — `01_qnx_vehicle_gateway_realtime.c`

**Intent:** establish the real-time gateway foundation.

The source header describes QNX `SCHED_FIFO`, bounded queues, MCP2515 TX buffers, load-shedding metrics, CPU-time monitoring, CPU affinity, UDP telemetry and deterministic storage/archival concepts. fileciteturn10file0L11-L40

## 02 — `02_QNX_GATEWAY_MASTER_CORRECTED.c`

**Intent:** make the criticality hierarchy explicit and protect the safety path.

The architecture separates Safety, Normal and Telemetry queues and assigns Safety P50, Gateway P30 and Diagnostics P15, with CAN RX at P40. fileciteturn10file3L381-L405

The RX thread classifies safety IDs, normal IDs and low-priority congestion/telemetry traffic into isolated paths. fileciteturn10file8L933-L1050

## 03 — `03_qnx_master_code_sensor_aware.c`

**Intent:** connect the gateway policy to actual MCXA156 sensor semantics.

The source is explicitly matched to the supplied MCXA156 protocol and includes temperature, humidity, speed, battery voltage/current, ultrasonic and IR messages. It describes gateway safety detection from ultrasonic/IR and a high-priority stop command. fileciteturn13file2L247-L295

This is an important conceptual step: the gateway starts interpreting **sensor state**, not merely forwarding generic CAN frames.

## 04 — `04_qnx_master_momentics.c`

**Intent:** expand the engineering demonstration toward a multicore gateway architecture, with explicit CPU roles, additional scheduling layers, congestion generation, diagnostics and fault-analysis storage. fileciteturn10file1L123-L198

The source explicitly warns that its data/address/memory buses are logical software abstractions, not physical Raspberry Pi CPU buses. That warning must remain in the repository and presentation. fileciteturn10file1L192-L194

## 05 — Final architecture direction

The final design direction is to compute a deterministic **severity score** from physical sensor values before assigning a criticality class.

This is a design evolution, not something to falsely attribute to every historical version:

```text
Sensor values
    ↓
Severity score
    ↓
Safety / Warning / Normal
    ↓
QNX priority path
```

The scoring policy is documented separately in `docs/decisions/criticality-scoring.md`.

## 05 — `05_updated_gateway_source.c`

**Intent:** a later consolidated/updated gateway implementation that retains the same core real-time architecture: QNX SPI resource-manager access, bounded queues, explicit priorities, safety-first forwarding, load shedding and measurement/telemetry support. It is retained separately so later edits can be compared rather than silently overwriting the baseline.
