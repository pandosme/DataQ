# DataQ

DataQ is an MQTT Client for Axis cameras that enables custom data-driven solutions when standard camera data formats are insufficient. It processes, filters, and transforms data before MQTT publishing to optimize resource usage.

## Key Features

**Data Processing**
- Optimized data transformation
- Resource-efficient publishing
- Real-time analytics processing

**Target Users**
- System integrators
- Custom solution developers
- Data-driven application builders

## Prerequisites

- Axis device (ARM7HF or AARCH64)
- The later the firmware provides better data
- MQTT Broker with WebSocket support
- MQTT client for data consumption

### [Download](https://www.dropbox.com/scl/fi/3z5ruobn27nvt2rwebqym/DataQ.zip?rlkey=etnpo7yvp2u6vqxi9d50hqpik&st=ian3s4md&dl=1)
Pre-compiled version for ARMv7-HF and AARCH64

---
If you find this ACAP valuable, please consider [buying me a coffee](https://buymeacoffee.com/fredjuhlinl).  

---
## Data Types

### Events
Event-based triggers for actions, offering streamlined MQTT message publishing compared to standard Axis device capabilities.

### Object Analytics

**Detection Data**
- Real-time object detection
- Bounding box information
- Classification data
- High data throughput

**Tracker Data**
- Movement-based updates (and 2-second intervals)
- Direction, speed, and distance metrics
- Optimized bandwidth usage

**Path Data**
- Ideal for post-processing applications
- Support for:
  - Flow heatmaps
  - Dwell analysis
  - Forensic searching
  - Object counting
- Minimal bandwidth consumption

**Occupancy Data**
- How many and what objects are currently in the scene
- 

**Status Data**
- Network load monitoring
- CPU usage tracking
- Uptime statistics

## Technical Specifications

### Coordinate System
- Relative coordinates: [0,0] to [1000,1000]
- Origin: Top-left corner
- Aspect ratio independent

### Object Properties

| Property | Description |
|----------|-------------|
| id | Unique object identifier |
| type | Object class ID |
| class | Object classification (Human, Vehicle, etc.) |
| active | Tracking status boolean |
| x, y, w, h | Bounding box coordinates |
| cx, cy | Center of gravity coordinates |
| dx, dy | Travel distance from origin |
| birth | Object detection timestamp (EPOCH) |
| bx, by | Initial position coordinates |
| age | Duration since detection (seconds) |
| confidence | Detection confidence (0-100) |
| timestamp | Current/last detection time |
| topVelocity | Maximum tracked speed |
| color, color2 | Primary/secondary object colors |
| attributes | Additional object characteristics |
| path | Position and duration array |

## MQTT Configuration

### Broker Setup Options
- [Mosquitto](https://mosquitto.org/)
- [Node-RED AEDES](https://flows.nodered.org/node/node-red-contrib-aedes)

### Requirements
- WebSocket support for visualization
- Topics and payload specifications (Documentation pending)

> **Note**: Use DataQ only when standard Axis device data formats don't meet your requirements. This tool deprecates and replaces SIMQTT, ObjectTracker, and ObjectPath.

# History

### 1.1.0	January 18, 2025
- Objects are now only tracked withing Area-Of-Intrest
- Added Scene Max Age (Detections) that defines how old an object needs to be before being ignored.  Typically used to Occupancy.
- Added Occupancy

### 1.0.2	January 14, 2025
- Initial commit
