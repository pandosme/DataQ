# DataQ MQTT Topics

All topics are prefixed with the **preTopic** configured in MQTT Settings (e.g. `dataq`).  
Full topic form: `{preTopic}/{topic}/{serial}`

Every payload published via `MQTT_Publish_JSON` automatically receives three extra fields appended by the MQTT layer:

| Field | Type | Description |
|---|---|---|
| `serial` | String | Device serial number |
| `name` | String | Camera name (if configured in MQTT settings) |
| `location` | String | Camera location label (if configured in MQTT settings) |

---

## connect/{serial}

**Retained:** yes  
**Trigger:** On MQTT broker connect / disconnect

### Connected payload
```jsonc
{
  "connected": true,
  "model": "P3245-V",
  "address": "192.168.1.100",
  "labels": [
    { "id": "human",   "name": "Human",   "enabled": true },
    { "id": "car",     "name": "Car",     "enabled": true },
    { "id": "bike",    "name": "Bike",    "enabled": true },
    { "id": "bus",     "name": "Bus",     "enabled": true },
    { "id": "truck",   "name": "Truck",   "enabled": true }
  ],
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

### Disconnected payload (LWT)
```jsonc
{
  "connected": false,
  "address": "192.168.1.100",
  "serial": "B8A44F7ADD87"
}
```

---

## status/{serial}

**Retained:** no  
**Trigger:** Every 15 minutes and once on connect  
**Enable/disable:** `publish.status`

```jsonc
{
  "model": "P3245-V",
  "Network_Kbps": 1024,
  "CPU_average": 42,
  "Uptime_Hours": 72,
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

---

## detections/{serial}

**Retained:** no  
**Trigger:** Every detection cycle (~10 Hz). An empty `list` is published once when the scene clears.  
**Enable/disable:** `publish.detections`

```jsonc
{
  "list": [
    {
      "id": "abc123",
      "class": "Human",
      "confidence": 87,
      "x": 412, "y": 530, "w": 80, "h": 200,
      "age": 3.2,
      "idle": 0.0,
      "active": true,
      "timestamp": 1772276400318
    }
  ],
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique tracking ID |
| `class` | String | Detected class label |
| `confidence` | Integer | Detection confidence 0–100 |
| `x`, `y` | Integer | Top-left corner in [0,1000] view space |
| `w`, `h` | Integer | Width and height in [0,1000] view space |
| `age` | Float | Seconds the object has been tracked |
| `idle` | Float | Seconds since last significant movement |
| `active` | Boolean | True while tracked, false on delete |
| `timestamp` | Float | Epoch milliseconds |

---

## tracker/{serial}

**Retained:** no  
**Trigger:** Every detection cycle per tracked object. Final message has `active: false`.  
**Enable/disable:** `publish.tracker`

```jsonc
{
  "id": "abc123",
  "class": "Human",
  "confidence": 87,
  "x": 412, "y": 530, "w": 80, "h": 200,
  "cx": 452, "cy": 630,
  "age": 3.2,
  "idle": 0.0,
  "distance": 12.5,
  "dx": 45, "dy": -20,
  "bx": 300, "by": 650,
  "active": true,
  "timestamp": 1772276400318,
  "birth": 1772276397000,
  "color": "Blue",
  "color2": "White",
  "face": false,
  "hat": "None",
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique tracking ID |
| `class` | String | Detected class label |
| `confidence` | Integer | Confidence 0–100 |
| `x`, `y`, `w`, `h` | Integer | Bounding box in [0,1000] view space |
| `cx`, `cy` | Integer | Center-of-gravity (bottom-center of box) |
| `age` | Float | Total seconds in scene |
| `idle` | Float | Seconds since last significant movement |
| `distance` | Float | Percent of 2D view traversed |
| `dx`, `dy` | Integer | Net displacement from birth position (right/down = positive) |
| `bx`, `by` | Integer | Birth position in [0,1000] |
| `active` | Boolean | False on final delete message |
| `timestamp` | Float | Epoch milliseconds of this update |
| `birth` | Float | Epoch milliseconds when first detected |
| `color` | String | Primary color label _(optional)_ |
| `color2` | String | Secondary color label _(optional)_ |
| `face` | Boolean | Face visible _(optional, humans only)_ |
| `hat` | String | Hat type _(optional, humans only)_ |
| `anomaly` | String | Anomaly reason _(optional)_ |

---

## path/{serial}

**Retained:** no  
**Trigger:** When a tracked object leaves the scene, if it has ≥ 3 sampled positions.  
**Enable/disable:** `publish.path`

```jsonc
{
  "class": "Car",
  "confidence": 91,
  "age": 8.4,
  "distance": 34.7,
  "dx": 312, "dy": 18,
  "bx": 120, "by": 720,
  "timestamp": 1772276395000,
  "dwell": 1.2,
  "maxSpeed": 4.1,
  "maxIdle": 0.5,
  "id": "abc123",
  "color": "Silver",
  "stitched": false,
  "anomaly": "Wrong way",
  "path": [
    { "x": 120, "y": 720, "d": 0.0, "t": 1772276395.000 },
    { "x": 210, "y": 715, "d": 0.3, "t": 1772276396.200 },
    { "x": 432, "y": 718, "d": 0.0, "t": 1772276398.100, "lat": 55.6034, "lon": 13.0021 }
  ],
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

| Field | Type | Description |
|---|---|---|
| `class` | String | Detected class label |
| `confidence` | Integer | Max confidence during tracking |
| `age` | Float | Total seconds in scene |
| `distance` | Float | Percent of 2D view traversed (can exceed 100) |
| `dx`, `dy` | Integer | Net x/y displacement (right/down = positive) |
| `bx`, `by` | Integer | Birth position in [0,1000] |
| `timestamp` | Float | Epoch milliseconds at birth |
| `dwell` | Float | Max seconds at any single sample point |
| `maxSpeed` | Float | Highest speed detected |
| `maxIdle` | Float | Longest idle period in seconds |
| `id` | String | Unique tracking ID |
| `color`, `color2` | String | Primary/secondary color labels _(optional)_ |
| `face` | Boolean | Face visible _(optional, humans only)_ |
| `hat` | String | Hat type _(optional, humans only)_ |
| `anomaly` | String | Anomaly reason _(optional)_ |
| `stitched` | Boolean | True if merged from multiple segments _(optional)_ |
| `path[].x`, `.y` | Integer | Sample position in [0,1000] |
| `path[].d` | Float | Seconds dwelled at this position |
| `path[].t` | Float | Epoch seconds when sampled |
| `path[].lat`, `.lon` | Float | Geographic coordinates _(optional, requires Geospace)_ |

---

## occupancy/{serial}

**Retained:** no  
**Trigger:** On change (stabilised over configured integration window). Empty object published when scene clears.  
**Enable/disable:** `publish.occupancy`

```jsonc
{
  "occupancy": {
    "Human": 3,
    "Car": 1
  },
  "timestamp": 1772276400318,
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

`occupancy` is a dynamic object keyed by class name. Classes with count zero are omitted.

---

## event/{serial}/{eventTopic}

**Retained:** no  
**Trigger:** On subscribed camera event  
**Enable/disable:** `publish.events`

`{eventTopic}` mirrors the Axis event topic suffix (e.g. `acap/DataQ/anomaly`, `VideoSource/DayNightVision`).

```jsonc
{
  "timestamp": 1772276400318,
  "active": true,
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

Additional fields are event-specific and come from the Axis event system.

---

## geospace/{serial}

**Retained:** no  
**Trigger:** Every tracker update when Geospace calibration is active  
**Enable/disable:** `publish.geospace`

```jsonc
{
  "class": "Human",
  "lat": 55.603421,
  "lon": 13.002187,
  "age": 3.2,
  "idle": 0.0,
  "id": "abc123",
  "active": true,
  "confidence": 87,
  "distance": 12.5,
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

---

## image/{serial}

**Retained:** no  
**Trigger:** Immediately when enabled; then daily at 12:00 local time  
**Enable/disable:** `publish.image`

Resolution: **640×360** for 16:9 cameras. For other aspect ratios the smallest supported resolution with width ≥ 640 is used.

```jsonc
{
  "rotation": 0,
  "aspect": "16:9",
  "timestamp": 1772276400318,
  "image": "<base64-encoded JPEG>",
  "serial": "B8A44F7ADD87",
  "name": "Front entrance",
  "location": "Sweden"
}
```

| Field | Type | Description |
|---|---|---|
| `rotation` | Integer | Camera rotation in degrees (0, 90, 180, 270) |
| `aspect` | String | Camera aspect ratio (e.g. `16:9`, `4:3`) |
| `timestamp` | Float | Epoch milliseconds of capture |
| `image` | String | Base64-encoded JPEG |
