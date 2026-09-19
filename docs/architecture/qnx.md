# QNX Architecture Rationale

## Why an RTOS?

The project is evaluating **timing behavior under contention**, not merely throughput. A scheduler that preserves explicit priority relationships is therefore more relevant than a design optimized only for average throughput.

QNX documents fixed-priority thread scheduling, preemption and priority inheritance as part of its real-time architecture. citehttps://www.qnx.com/developers/articles/article_298_1.html

## Why the microkernel model matters

QNX Neutrino keeps drivers, protocol stacks and other services outside the microkernel in memory-protected user-space components. QNX presents this as a basis for fault isolation and restartable services. citehttps://www.qnx.com/products/intl/neutrino_rtos/

For this project, that architecture is useful because hardware access and gateway logic can be treated as services rather than one monolithic privileged program.

## Why `SCHED_FIFO`?

The gateway has a known criticality ordering. `SCHED_FIFO` makes that ordering explicit: among runnable threads, the higher-priority runnable thread is serviced before lower-priority work.

The prototype priority model is:

```text
P50  Safety
P40  CAN RX
P30  Gateway
P15  Diagnostics
```

## Why isolated queues?

A safety event should not wait behind a large diagnostic/telemetry queue. Separate queues reduce head-of-line blocking between criticality classes.

The safety thread in the corrected implementation deliberately does not inspect or wait on the normal queue; it forwards the safety frame first and performs logging afterward. This keeps file/telemetry operations outside the critical forwarding path. fileciteturn8file2L338-L350 fileciteturn8file4L767-L781

## Why load shedding?

If resources are exhausted, retaining every low-value frame can increase queueing delay and indirectly harm more important traffic. The prototype therefore uses an 85% telemetry-queue threshold as a controlled load-shedding point. fileciteturn10file0L83-L90

## Why QNX is a strong fit — not an 'irreplaceable' claim

QNX is particularly suitable because several required mechanisms are native to its design: real-time priority scheduling, preemption, priority inheritance, microkernel/service isolation and resource-manager based hardware access. QNX also documents adaptive partitioning for resource guarantees under overload. citeturn0search9turn0search11

Other RTOSs can implement many of these principles. The project's defensible position is therefore **architectural fit**, not exclusivity.
