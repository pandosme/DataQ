# DataQ

DataQ is an MQTT Client for Axis cameras that enables custom data-driven solutions when standard camera data and formats are insufficient. It processes, filters, and transforms data before MQTT publishing to optimize resource usage.

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
Detections are used for analytics visualization, primarily to validate the beaviour and to understand what and how to filter unessesery data before published.
- Real-time object detection
- Bounding box information
- Classification data
- High bandwidth utilization

**Tracker Data**
Tracker data are typically used for real-time automation, counting or occupnacy based on one or more object class, placement, size, direction, age, distans.
- Movement-based updates (and 2-second intervals)
- Includes Age, direction, speed, and distance metrics
- Optimized bandwidth comapred to Detection Data

**Path Data**
Typically used for storing the data in a database.  Properties are otimized for quering.  Typical use are Flow heatmap, Dwell analysis, Forensic search, forensic counting
- Minimal bandwidth consumption compared to Detections and Path

**Occupancy Data**
- Provides real-time updates when number of detected objects in the scene changes
	- Queue management
	- Loitering

**Geospace Data**  
  
Geospace transforms the object detections x/y space in video to longitude and latitude. The technology used is homography.  
In order to get a good result, it is recommended that the calibration markers cover the area where objects move while maximizing the 4-corner area. It is also recommended to enable the camera's Barrel distortion correction (Menu | Installation | Image correction).  
You may add more than 4 markers if needed, but it may also make things worse.  

Geospace data uses Trackers for the transformations. Trackers do not need to be published but you may need to adjust both "Detections" and "Trackers".  
Recommended settings:  
* Enable the cameras Barrel distortion correction.
* Set Detections "Max idle" to 5 or 10 seconds to prevent sending location for stationary objects.
* Set Detection COG (Center-of-Gravity) to bottom-center.
* Disable all labels under Detections you are not interested in.
* Set Tracker "Minimum distance" to 5% or more to prevent stationary objects from being falsely detected as moving.

1. Click "Edit Markers".  
  - Use the mouse to move the map to the area the camera is located. Click "Save map"
  - Add 4 markers by clicking the left mouse in the video. A corresponding marker will be displayed in the map. 
	Move the markers to distinct positions (e.g. corner of a building)
	Marker is removed by right-clicking on the marker in the video view.  
  - Click "Save and calibrate"
2. Click "Verify"
  - Use the mouse and left-click in the video view. A marker will be shown in the map. See how well they correlate. You may need to go back to step 1 and adjust calibration.  
3. Click "Monitor"
  - Detected moving Objects will be displayed in both video and map view.

Check out the Node-RED example worldmap-flow.json under examples. Import the worldmap node and the worldmap-flow.json to your Node-RED. Configure the MQTT client.  

### Monitoring Data

**Status Data**
MQTT Heartbeat published every 15 minutes
- Network load monitoring
- CPU usage tracking
- Uptime statistics

## Integration Specifications

### Coordinate System
- Relative coordinates: to[1000][1000]
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
| cx, cy | Center of gravity coordinates. Either middle of the object or bottom-center |
| dx, dy | Travel distance from origin |
| birth | Object detection timestamp (EPOCH) |
| bx, by | Initial position coordinates |
| age | Duration since detection (seconds) |
| confidence | Detection confidence (0-100) |
| timestamp | Current/last detection time |
| topVelocity | Maximum tracked speed (% of view/seconds)|
| color, color2 | Primary/secondary object colors |
| attributes | Additional object characteristics |
| path | Position and duration array |
| lat,lon | Longitude and Latitude |

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
- WebSocket support for data visualization in the user interface


> **Note**: This ACAP deprecates and replaces SIMQTT, ObjectTracker, ObjectPath and Occupancy  
> **Note**: Use DataQ when standard Axis device services and data formats do not meet your requirements.

# History

### 1.4.6	May 14, 2025
- Fixed a bug that caused respawn of the ACAP

### 1.4.5	May 6, 2025
- Fixed MQTT reconnect issue

### 1.4.4	April 14, 2025
- Perodically force re-connection if needed as lostConnection may be missed
- Fixed LWT topic

### 1.4.3	April 14, 2025
- Reset the client on re-connect
- Fixed LWT topic

### 1.4.2	March 23, 2025
- Improved MQTT Reconnection stability

### 1.4.1	March 13, 2025
- Added Geospace data (Check documentation above).
- Added web page MQTT connection message box to know if the web page is connected or not.
- Bug fixes

### 1.3.0	March 3, 2025
- Replaced "Max age" with "Max idle".  
  When Max idle time is set, detections are not published when after not moving X seconds.  
  When the object starts moving, the same object ID will be used when publishing. Age, birth and distance will be preserved.    
  Occupancy is impacted and will not count idle objects.  
  Path will be published when object is idle.  
- Restructuring GUI

### 1.2.10	March 3, 2025
- Fixed MQTT TLS flaws introduced in refactoring (1.2.9)

### 1.2.9	March 2, 2025
- Refactoring MQTT client
  * Announcement retained message
  * Disconnect retained message
- User interface updates

### 1.2.8	February 22, 2025
- Created a new Menu "Scene" to be used to monitor and configure scene behavior
- Added support for enabling low confident trackers.
- Added temporarily enabling publishing for objects needed for a specific page if the publishing is disabled. The publishing will be disabled again when leaving the page.
- Fixed MQTT stability (recurring disconnects)
- Added WSS port and a way to enforce the client to use WSS even for pages accessed with HTTP. WSS will always be used when a page is accessed over HTTP.

### 1.2.7	February 12, 2025
- Fixed logic and reference flaws.
  Pull Request from InSupport

### 1.2.6	February 9, 2025
- Fixed support for data visualization when accessing camera over HTTPS with Secure WebSockets in client

### 1.2.5	February 3, 2025
- Adjustments on MQTT payload for path
- Allow multiple web browsers (and tabs) visualize MQTT messages

### 1.2.0	February 3, 2025
- Fixed "hanging-objects" due to changes in Axis OS12
- Fixed a memory leak (event processing)
- Fixed faulty center-of-gravity when camera is rotated
- Detections filter will only impact Occupancy but no longer Trackers & Paths
- Tracker filter impacts Path

### 1.1.1	January 23, 2025
- Corrected Object Detection post-processing 

### 1.1.0	January 18, 2025
- Objects are now only tracked within Area-Of-Interest
- Added Scene Max Age (Detections) that defines how old an object needs to be before being ignored. Typically used for Occupancy.
- Added Occupancy

### 1.0.2	January 14, 2025
- Initial commit
