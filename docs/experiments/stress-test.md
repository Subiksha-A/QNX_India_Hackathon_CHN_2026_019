# Controlled CAN Congestion Experiment

## Objective

Determine whether the gateway protects the safety processing path when low-priority CAN traffic creates contention.

## Current physical setup

```text
MCXN236 traffic generator
          │
         CAN0
          │
          ▼
   MCP2515 CAN HAT
          │
          ▼
    Raspberry Pi / QNX
          │
        CAN1
          │
          ▼
      MCXA156 ECU
```

The current experiment uses MCXN236 as the congestion source and MCXA156 on the other CAN channel. Historical code versions may use different logical CAN0/CAN1 roles; preserve those mappings in their version notes rather than rewriting history.

## Stress

Use controlled low-priority traffic. The current gateway code uses `0x450` as the synthetic congestion/noise identifier. Some test code generates a finite burst of 300 frames. Treat that number as a **test workload**, not as a real automotive traffic rate. fileciteturn10file8L994-L1011

## Mitigation

The gateway protects the safety path using:

- isolated safety/normal/telemetry queues
- fixed priority scheduling
- safety-first processing
- 85% telemetry load shedding
- minimal work before safety forwarding
- logging after the critical forwarding operation

## Measurements

Record at minimum:

| Metric | Meaning |
|---|---|
| Safety latency | Gateway receive → safety forwarding |
| Max latency | Worst observed safety-path latency |
| Average latency | Typical safety-path latency |
| Jitter | Variation in safety-path latency |
| Deadline misses | Events exceeding the configured 5 ms target |
| Queue occupancy | Resource pressure |
| Dropped telemetry | Cost of graceful degradation |
| Bus load | Communication pressure |
| CPU utilisation | Compute pressure |

## Interpretation

The result should answer:

> **Does the system degrade low-criticality traffic before safety-critical processing becomes the first casualty?**

Do not call the 5 ms target an automotive universal requirement. It is the project's experimental deadline.
