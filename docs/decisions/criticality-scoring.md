# Criticality Scoring — Final Architecture Direction

**Status: proposed / to be implemented and measured.**

## Why change the original classification?

The earlier gateway classified frames primarily by CAN identifier. That is useful for demonstrating communication priority, but the same sensor message type can represent very different physical situations.

Example:

```text
LiDAR message + object far away     → lower urgency
LiDAR message + object very close   → higher urgency
```

The final architecture should therefore derive criticality from **sensor values** before QNX scheduling.

## Recommended first implementation

Keep it deterministic and lightweight. Do not introduce ML during the final integration window.

### Per-sensor score

Normalize each sensor's risk contribution to a bounded 0–100 score.

```text
LiDAR score       = distance-based risk
Ultrasonic score  = distance-based risk
Temperature score = threshold-based thermal risk
```

### Fusion rule

Use the maximum active risk for the first prototype:

```text
overall_score = max(lidar_score,
                    ultrasonic_score,
                    temperature_score)
```

This avoids a dangerous situation being diluted by unrelated normal measurements.

### Criticality bands

Use configurable thresholds rather than claiming universal automotive thresholds:

```text
0–39    NORMAL
40–69   WARNING
70–100  SAFETY
```

These numbers are **prototype policy values** and must be validated against the chosen sensor ranges before being presented as measured safety criteria.

## Scheduling mapping

The score determines the class; QNX enforces the processing priority.

```text
NORMAL   → lower-priority processing
WARNING  → normal gateway processing
SAFETY   → highest-priority safety path
```

The existing QNX thread priorities can remain fixed. The message/work item changes criticality; the RTOS does not need to dynamically rewrite thread priorities for every sensor sample.

## Why this is realistic

This keeps three layers separate:

1. **Physical/application state:** sensor values.
2. **Software criticality:** safety/warning/normal.
3. **Execution scheduling:** QNX priority.

CAN arbitration remains a separate network-layer mechanism.

## Required validation

Before calling this implemented, record:

- raw sensor inputs
- calculated score
- selected class
- selected queue
- processing timestamp
- forwarding timestamp
- end-to-end gateway latency
- behavior during CAN congestion

Do not claim the score is a production collision-risk model. It is a deterministic prototype policy used to demonstrate criticality-aware scheduling.
