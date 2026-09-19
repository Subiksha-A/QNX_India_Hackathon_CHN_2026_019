# Automotive Reality Check

## Three different priority decisions

The project must not blur together application criticality, RTOS scheduling and CAN arbitration.

### 1. Application layer

The application interprets sensor state and decides what information is urgent.

For the final design:

```text
LiDAR + ultrasonic + temperature
              ↓
       severity evaluation
              ↓
    safety / warning / normal
```

### 2. ECU software scheduling

The RTOS decides which runnable/thread gets CPU execution first. Our QNX prototype uses explicit thread priorities and `SCHED_FIFO`.

### 3. CAN bus arbitration

CAN arbitration is a network-level mechanism. AUTOSAR's CAN Driver specification states that CAN L-PDU priority is represented by the CAN identifier and that lower numerical identifier values have higher priority. citehttps://www.autosar.org/fileadmin/standards/R24-11/CP/AUTOSAR_CP_SWS_CANDriver.pdf

This means the prototype should say:

> **The application decides what matters. The RTOS decides what runs first. CAN decides which contending frame wins bus access.**

## Why this resembles automotive architecture

Automotive software stacks distinguish application behavior, ECU scheduling and network communication. AUTOSAR is a standardized automotive software and E/E architecture framework, while production ECU platforms provide real-time communication, diagnostics, timing and safety-related infrastructure. citehttps://www.autosar.org/

The prototype is intentionally smaller. It demonstrates the separation of concerns rather than reproducing a complete AUTOSAR stack.

## Important limitation

The current prototype's sensor-value criticality evaluator is an application-level policy. It must not be described as the mechanism by which CAN itself arbitrates traffic. CAN arbitration remains identifier-based.
