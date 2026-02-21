# Path Structure Description

The DataQ ACAP installed in a camera uses the camera's object detection capabilities to process bounding-box, timestamp, object UID, class, and confidence.
The Path service tracks objects, samples positions when movement is more than 5% of the camera view, and builds a path array of sampled positions.
To manage perspective views, x and y represent the center-of-gravity for the object, typically at the bounding-box middle-bottom, as close to the ground as possible.
The coordinate system is normalized [0,0][1000,1000] regardless of video aspect ratio.

Paths with fewer than 3 sampled positions are discarded and not published.

```jsonc
{
  "class": "String, object label (e.g. Car, Human, Bike, Bus, Truck)",
  "confidence": "Integer, detection confidence (0-100)",
  "age": "Float, total seconds present in scene",
  "distance": "Float, percent of 2D view traversed (may exceed 100)",
  "directions": "Integer, number of distinct direction changes",
  "color": "String, primary detected color label (optional)",
  "color2": "String, secondary detected color label (optional)",
  "dx": "Integer, x displacement (last x - first x). Right is positive",
  "dy": "Integer, y displacement (last y - first y). Down is positive",
  "bx": "Integer, birth x in [0,1000] view space",
  "by": "Integer, birth y in [0,1000] view space",
  "timestamp": "Float, epoch milliseconds at birth",
  "dwell": "Float, max seconds at any single sample point",
  "maxSpeed": "Float, highest speed detected during the object's lifetime",
  "maxIdle": "Float, longest idle period (seconds) during the object's lifetime",
  "id": "String, unique tracking ID per object",
  "face": "Boolean, whether human face is visible (optional, humans only)",
  "hat": "String, hat type label (optional, humans only)",
  "anomaly": "String, reason for anomaly flag (optional, only if anomaly detected)",
  "stitched": "Boolean, true if path was merged from multiple segments (optional)",
  "path": [
    {
      "x": "Integer, center-of-gravity x in [0,1000]",
      "y": "Integer, center-of-gravity y in [0,1000]",
      "d": "Float, seconds stayed at this position before moving",
      "t": "Float, epoch seconds when this position was sampled",
      "lat": "Float, latitude (optional, requires Geospace calibration)",
      "lon": "Float, longitude (optional, requires Geospace calibration)"
    }
    // ... repeated for each movement sample
  ],
  // The following properties are automatically added by the MQTT publish layer:
  "name": "String, camera name (if configured in MQTT settings)",
  "location": "String, camera location (if configured in MQTT settings)",
  "serial": "String, device serial number"
}
```
