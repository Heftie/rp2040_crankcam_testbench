# CAM AND CRANK SIGNAL TIMING ANALYSIS

## Engine Trigger Wheel Configuration Comparison

## Overview

This document provides a detailed analysis of the crankshaft (crank) and camshaft (cam) signal timing characteristics across three different engine configurations. The data is presented as signal waveforms plotted against crank angle (in degrees after top dead center). Understanding these timing relationships is critical for engine control unit (ECU) calibration and diagnosis.

---

## FAW Diesel Engine with iFlexAir 58/7 Teeth Trigger Wheels

### Configuration Overview

The FAW Diesel engine uses a dual-trigger wheel configuration: a 58-tooth wheel on the crankshaft and a 7-tooth wheel on the camshaft.

### Crankshaft Signal Characteristics

| Characteristic | Description | Crank Angle (°CrA) | Notes |
|---|---|---|---|
| Tooth Count | 58 teeth total | ~6.2° per tooth | 360° / 58 = 6.2° |
| Signal Type | Square wave pulses | Repetitive at ~6.2° | High resolution timing |
| Missing Tooth | Gap around -72°CrA | -78° to -72° | Synchronization reference |
| Operating Range | Spans full cycle | -240° to +480° | Two complete revolutions |

### Camshaft Signal Characteristics

| Characteristic | Description | Crank Angle (°CrA) | Notes |
|---|---|---|---|
| Tooth Count | 7 teeth total | ~51.4° per tooth | 360° / 7 = 51.4° |
| Signal Type | Discrete pulses | ~7 pulses per cycle | Lower resolution timing |
| Pulse Positions | -192°, -162°, -72°, +48°, +168°, +288°, +408° | Rising edges | Cylinder identification |

### Key Observations

- The crank signal provides continuous high-resolution timing reference with 58 teeth providing fine angular resolution (6.2° per tooth)
- The missing tooth at approximately -72°CrA serves as the synchronization point for the ECU
- The cam signal (7 teeth) provides lower-resolution cylinder identification and valve timing confirmation
- The spacing between cam pulses is approximately 51.4° of crankshaft rotation, which corresponds to the 4-stroke cycle distribution
- The two signals are phase-locked: the cam signal repeats once per two crankshaft revolutions

---

## FAW CNG Engine with iFlexAir 58/7 Teeth Trigger Wheels

### Configuration Overview

The FAW CNG engine also uses the same 58/7 tooth trigger wheel configuration as the diesel variant, but with different timing characteristics due to fuel type and combustion parameters.

### Crankshaft Signal Characteristics

| Characteristic | Description | Crank Angle (°CrA) | Notes |
|---|---|---|---|
| Tooth Count | 58 teeth total | ~6.2° per tooth | Identical to diesel |
| Operating Range | Spans full cycle | -90° to +630° | Extended range monitoring |

### Camshaft Signal Characteristics

| Characteristic | Description | Crank Angle (°CrA) | Notes |
|---|---|---|---|
| Tooth Count | 7 teeth total | Uneven spacing | Variable tooth pitch |
| Pulse Positions | -42°, +51°, +81°, +158°, +278°, +398°, +518° | Rising edges | Asymmetric pattern |

### Key Differences from Diesel Version

- The CNG version extends monitoring range to -90° to +630° (compared to diesel -240° to +480°)
- Cam signal shows uneven tooth spacing/unequal intervals between pulses
- First cam pulse appears at -42°CrA (different from diesel at -192°)
- CNG engines require different ignition timing and cylinder pressure management due to lower energy density of natural gas

---

## Perkins Engine with iFlexAir 59/11 Teeth Trigger Wheels

### Configuration Overview

The Perkins engine uses a higher-resolution configuration with a 59-tooth crankshaft wheel and an 11-tooth camshaft wheel, providing enhanced timing precision and diagnostic capability.

### Crankshaft Signal Characteristics

| Characteristic | Description | Crank Angle (°CrA) | Notes |
|---|---|---|---|
| Tooth Count | 59 teeth total | ~6.1° per tooth | 360° / 59 = 6.1° |
| Signal Type | Square wave pulses | Continuous | Highest resolution option |
| Signal Gap | Null at top dead center | 0 °CrA (@ 10 teeth falling) | Zero-crossing reference |

### Camshaft Signal Characteristics

| Characteristic | Description | Crank Angle (°CrA) | Notes |
|---|---|---|---|
| Tooth Count | 11 teeth total | Highest count | Enhanced resolution |
| Falling Edges | PES to PEA = 6 edges | 18° to 378° | Cylinder 1 identification |
| PEA to PES | 5 falling edges | 378° to end of cycle | Cylinders 2-6 identification |

### Cylinder Identification Scheme

The Perkins cam signal uses a unique falling-edge counting method to identify cylinders:

- **Cylinder NULL** (crank hole): Zero falling edges (CrS Hole to PES)
- **Cylinder 1**: PES at 18°CrA (Cylinder NULL = PES)
- **Cylinders 2-6**: Identified by 6 falling edges from PES to PEA (18° to 378°CrA)
- **PEA** (Primary Event Area): Marks end of cylinder 1 identification at 378°CrA
- **Remaining cylinders**: 5 additional falling edges from PEA back to PES

### Cam Signal Pulse Positions

| Position | Crank Angle (°CrA) | Edge Type | Description |
|---|---|---|---|
| PES | 18° | Falling | Primary Event Start |
| Edge 1-6 | 38°, 98°, 158°, 218°, 278°, 338° | Falling | Cylinder 1 phase edges |
| PEA | 378° | Falling | Primary Event Area (end of cyl 1) |
| Edge 7-11 | 486°, 516°, 578°, 636° | Falling | Cylinders 2-6 identification |

### Key Differences and Advantages

- 59-tooth configuration offers superior resolution (6.1° per tooth) versus FAW 58-tooth (6.2°)
- 11-tooth cam wheel provides more cylinder identification resolution than FAW 7-tooth design
- Unique falling-edge counting scheme enables robust cylinder identification without ambiguity
- Primary Event Start (PES) and Primary Event Area (PEA) markers provide diagnostic waypoints
- The null (gap) at 0°CrA serves as a precise top-dead-center reference

---

## Comparative Analysis and ECU Implications

### Resolution Comparison

| Engine Type | Crank Teeth | Cam Teeth | Resolution |
|---|---|---|---|
| FAW Diesel | 58 | 7 | 6.2° |
| FAW CNG | 58 | 7 (unequal) | 6.2° |
| Perkins | 59 | 11 | 6.1° |

### Signal Synchronization Strategy

All three engines use a similar two-tier synchronization approach:

1. **Crankshaft Signal (High Frequency)**: Provides continuous position feedback and timing reference for injection, ignition, and fuel calculations

2. **Camshaft Signal (Low Frequency)**: Confirms proper valve timing and cylinder identification; serves as secondary synchronization check

### Practical ECU Implications

- **Injection Timing**: The high-resolution crank signal enables precise fuel injection scheduling at sub-6° accuracy

- **Misfire Detection**: Multiple crank edges allow detection of combustion anomalies within a few degrees of crank rotation

- **Cam Phasing**: The cam signal confirms expected valve timing and can detect variances indicating mechanical wear or misalignment

- **Fault Diagnosis**: Asymmetric cam pulses (as in FAW CNG) provide diagnostic signatures for identifying specific sensor or mechanical faults

- **Cranking Control**: The initial crank signal teeth patterns help ECU identify engine position during start and prevent mis-sequencing of fuel injectors

---

## Conclusion

The three engine configurations analyzed in this document represent different approaches to crank and cam signal timing for engine control. While the FAW engines use a more common 58/7 tooth configuration that balances cost and complexity, the Perkins engine takes a higher-resolution approach with 59/11 teeth and an advanced falling-edge counting scheme for cylinder identification.

Understanding these timing relationships is essential for:

- Accurate ECU calibration and parameter tuning
- Effective diagnostic and fault code analysis
- Troubleshooting sensor failures and signal integrity issues
- Supporting aftermarket modifications and performance optimization

Each configuration demonstrates how engineers optimize sensor integration and timing strategies for specific engine types, fuel systems, and application requirements.
