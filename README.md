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

## Known limitations
- 

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
  - Flow heatmaps
  - Dwell analysis
  - Forensic searching
  - Object counting
- Minimal bandwidth consumption and post processing needs

**Occupancy Data**
- Provides updates when number of detected objects in the scene changes
	- Que management
	- Loitering

### Monitoring Data

**Status Data**
MQTT Heartbeat published every 15 minutes
- Network load monitoring
- CPU usage tracking
- Uptime statistics

## Integration Specifications

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
  Add the following to your mosquitto.conf
  ```
  listener 1884
  protocol websockets
  ```
- [Node-RED AEDES](https://flows.nodered.org/node/node-red-contrib-aedes)  
  Bind WS port 1884 in the AEDES settings

### Requirements
- WebSocket support for visualization
- Topics and payload specifications (Documentation pending)

> **Note**: This tool deprecates and replaces SIMQTT, ObjectTracker, ObjectPath and Occupancy
> **Note**: Use DataQ when standard Axis device data formats don't meet your requirements.

# History

### 1.2.9	March 2, 2025
- Refactoring MQTT client
  * Announcement reatined message
  * Disconnect retained message
- User interface updates

### 1.2.8	February 27, 2025
- Created a new Menu "Scene" to be used to monitor and configure scene behaviour
- Added support for enabling low confident trackers.
- Added temporarily enabling publishing for objects needed for a specific page if the publishing is disabled. The publishing will be disabled again when leaving the page.
- Fixed MQTT stability (recurring disconnects)
- Added WSS port and a way to enforce the client to use WSS event for pages accessed with HTTP.  WSS will always be used when a paged os accessed over HTTP.

### 1.2.7	February 12, 2025
- Fixed logic and refernce flaws.
  Pull Request from InSupport

### 1.2.6	February 9, 2025
- Fixed support for data visualization when accessin cemar over HTTPS with Secure WebSockets in client

### 1.2.5	February 3, 2025
- Adjustments on MQTT payload for path
- Allow multiple web browsers (and tabs) visualize MQTT messages

### 1.2.0	February 3, 2025
- Fixed "hanging-objects" due to changes in Axis OS12
- Fixed a memory leak (event processing)
- Fixed faulty center-of-gravity when camera is rotated
- Detections filter will only imapct Occupancy but no longer Trackers & Paths
- Tracker filter impacts Path

### 1.1.1	January 23, 2025
- Corrected Object Detection post-processeing 

### 1.1.0	January 18, 2025
- Objects are now only tracked withing Area-Of-Intrest
- Added Scene Max Age (Detections) that defines how old an object needs to be before being ignored.  Typically used to Occupancy.
- Added Occupancy

### 1.0.2	January 14, 2025
- Initial commit
