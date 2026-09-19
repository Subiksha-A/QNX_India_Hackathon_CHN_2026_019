# Proposed Final QNX Architecture

This directory is reserved for the **sensor-criticality version** of the gateway.

Do not copy one of the historical files here and call it final. The intended final flow is:

```text
UART LiDAR + CAN1 MCXA156 + CAN0 traffic
                    ↓
              Parse / timestamp
                    ↓
             Sensor state model
                    ↓
             Severity evaluator
                    ↓
           NORMAL / WARNING / SAFETY
                    ↓
           Existing QNX priority paths
                    ↓
             Route / forward / log
```

The score policy is specified in `docs/decisions/criticality-scoring.md`.

The first implementation should be deterministic, explainable and computationally small. The repository should retain the pre-score implementation as an archived baseline so that the effect of the change can be measured.
