# RP2040 Crank/Cam Simulator and ECU Output Analyzer

## 1. Objective

Build an RP2040-based test device that:

- Generates programmable crank, cam, and optional tertiary trigger signals.
- Uses PIO and DMA for deterministic, low-jitter waveform generation.
- Supports RPM profiles that vary during one complete 720° four-stroke engine cycle.
- Captures up to six ECU output signals.
- Converts captured edge timestamps into crank angles.
- Checks ECU outputs against expected angle windows, such as 120°–300°.

## 2. System concept

```text
RPM profile + trigger pattern
            │
            ▼
   Event-table generation
            │
            ▼
      DMA buffer(s)
            │
            ▼
 PIO state machine: crank/cam output
            │
            ▼
           ECU
            │
            ▼
 Protected ECU output inputs
            │
            ▼
 PIO state machine: edge capture
            │
            ▼
      DMA capture buffer
            │
            ▼
 Angle conversion and pass/fail analysis
```

## 3. 720° RPM profile

A four-stroke engine cycle covers 720° of crankshaft rotation. The profile defines the desired RPM at each crank position or trigger event.

Example:

| Crank angle | Desired RPM |
|---:|---:|
| 0° | 1,000 |
| 180° | 1,500 |
| 360° | 2,500 |
| 540° | 3,500 |
| 720° | 4,000 |

The profile is converted into individual event intervals. For a trigger wheel with `N` nominal positions per crank revolution:

\[
\Delta t_i = \frac{60}{RPM_i \cdot N}
\]

The interval is then converted into PIO clock cycles:

\[
C_i = \Delta t_i \cdot f_{PIO}
\]

The PIO receives the resulting event sequence rather than an RPM value.

## 4. Event-table format

Each generated event should contain:

- Output state or GPIO mask.
- Delay until the next event.
- Optional event flags, such as synchronization or cycle boundary.

Conceptual structure:

```c
typedef struct {
    uint32_t delay_cycles;
    uint32_t set_mask;
    uint32_t clear_mask;
    uint16_t angle_index;
    uint8_t flags;
} TriggerEvent;
```

The actual DMA format may use multiple 32-bit words per event because PIO FIFOs transfer 32-bit values.

## 5. PIO generation

A PIO state machine performs the timing-critical work:

1. Pull the next event from its FIFO.
2. Update crank and cam GPIO outputs.
3. Load or execute the event delay.
4. Repeat for the next event.

DMA continuously transfers event data from RAM to the PIO TX FIFO. The CPU does not need to toggle GPIO pins for every tooth.

```text
RAM event buffer → DMA → PIO TX FIFO → GPIO pins
```

The crank pattern includes missing-tooth gaps by using a longer delay for the missing positions. Cam transitions are included at their specified crank positions and are repeated over the 720° cycle.

## 6. Buffering strategy

### Precomputed cycle

For a fixed test, generate all events for 0°–720° before starting. DMA plays the buffer repeatedly or stops at the cycle boundary.

### Double buffering

For continuous operation:

```text
Buffer A → PIO/DMA
Buffer B → CPU prepares next profile
```

DMA chaining switches between buffers. While one buffer is being consumed, the CPU calculates the next buffer with updated RPM values. This permits RPM changes during a cycle without interrupting the waveform.

## 7. ECU output capture

A second PIO state machine monitors up to six protected ECU output inputs. On every selected edge it records:

- Input state or changed-pin mask.
- PIO timestamp or event counter.
- Cycle/reference information if required.

DMA transfers capture records into RAM:

```text
ECU inputs → PIO RX FIFO → DMA → capture buffer
```

The CPU later pairs rising and falling edges and calculates their crank angles.

## 8. Angle calculation

Define a reference event as 0°, for example the first crank tooth after the missing-tooth gap. If `C_rev` is the number of PIO cycles for the current crank revolution and `C_event` is the event offset from the reference:

\[
angle = 360° \cdot \frac{C_{event}}{C_{rev}}
\]

For a complete four-stroke cycle, retain the revolution number so that angles from 0° to 720° can be distinguished.

For changing RPM, use the actual generated reference-to-reference duration for each revolution rather than one fixed conversion factor.

## 9. Pass/fail evaluation

For each ECU output, define an expected window:

```text
Output 1:
  Rising edge: 120° ± 1°
  Falling edge: 300° ± 1°
  Expected cycle: 0
```

The analyzer should detect:

- Missing edges.
- Extra edges.
- Incorrect polarity.
- Incorrect rising or falling angle.
- Incorrect pulse width.
- Output active in the wrong 720° revolution.

## 10. RP2040 resource allocation

| RP2040 resource | Function |
|---|---|
| PIO state machine 0 | Crank/cam/tertiary generation |
| DMA channel(s) | Transfer event data to PIO |
| PIO state machine 1 | Capture ECU input transitions |
| DMA channel(s) | Transfer capture data to RAM |
| Core 0 | User interface, configuration, result processing |
| Core 1 | Buffer preparation and DMA supervision, optional |
| GPIO pins | Trigger outputs and protected ECU inputs |
| UART/USB | Configuration and result reporting |
| Optional ADC | Potentiometer or analog RPM input |

## 11. Key design constraints

- Use 3.3 V-compatible, protected input circuitry for ECU outputs.
- Verify GPIO assignments and PIO pin mapping.
- Keep PIO programs short and deterministic.
- Use integer or fixed-point timing calculations where possible.
- Ensure DMA buffers are aligned and located in suitable RAM.
- Use a safe buffer-switching mechanism so the PIO never reads partially updated data.
- Validate timing with a logic analyzer or oscilloscope.
- Model RPM changes smoothly; do not create physically impossible instantaneous changes unless intentionally testing that behavior.

## 12. Recommended implementation sequence

1. Generate a fixed-frequency GPIO waveform with PIO.
2. Add a crank trigger pattern.
3. Add missing-tooth handling.
4. Add cam output over 720°.
5. Feed events through DMA.
6. Add a predefined RPM profile.
7. Implement double buffering for changing profiles.
8. Add six-input PIO capture.
9. Add capture DMA and angle conversion.
10. Implement configurable pass/fail limits.

## Summary

The RP2040 solution uses the CPU to prepare trigger events and analyze results, while PIO and DMA perform the deterministic real-time work:

```text
CPU: profile and event preparation
PIO + DMA: crank/cam waveform generation
PIO + DMA: ECU output capture
CPU: angle conversion and validation
```

This architecture supports programmable crank/cam patterns, RPM variation throughout a 720° cycle, and accurate ECU output verification with minimal software timing jitter.
