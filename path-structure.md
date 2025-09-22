# Path Structure sescription
The DataQ APCAP installed in a camera uses the cameras object detection capabilities to process bounding-box, timestamp, object UID, class, configdence.
The Path service tracks objects, samples positions when movement is more than 5% of the camera view, and builds a path aryay of sampled positions.
To manage perspective views, x and y represetns the conter-of-gtavity for the object, yypically at the boundin-box middle-bottom, as close to the gound as possible.
The coordinate system is normalized [0,0][1000,1000] regardless of video aspect ratio.


{
  "class": "Object label (Car, Human, Bike, Bus, Truck)",  // Object type classification 
  "confidence": "Integer, percent confidence (0-100)",     // Detection certainty 
  "age": "Float, seconds present in scene",                // Duration tracked 
  "distance": "Float, percent of 2D view traversed",       // Movement as % of view, may exceed 100% if looping 
  "color": "String, primary color label",                  // Detected color (optional) 
  "dx": "Integer, x displacement (last x - first x)",      // Net movement: right is positive, left negative 
  "dy": "Integer, y displacement (last y - first y)",      // Net movement: down is positive, up negative 
  "bx": "Integer, birth x in [1000,1000] view space",      // Entry point x coordinate 
  "by": "Integer, birth y in [1000,1000] view space",      // Entry point y coordinate 
  "timestamp": "Float, epoch milliseconds at birth",       // First detection time (from camera) 
  "dwell": "Float, max seconds at any sample point",       // Greatest dwell time in path samples 
  "id": "String, unique tracking ID per object",           // Unique per tracking session 
  "path": [
    {
      "x": "Integer, center-of-gravity x in [1000,1000]",  // Sample position 
      "y": "Integer, center-of-gravity y in [1000,1000]",  // Sample position 
      "d": "Float, seconds stayed at position",            // Local dwell time 
      "lat": "Float, optional latitude (if homograph matrix set)", // May be omitted 
      "lon": "Float, optional longitude (if homograph matrix set)" // May be omitted 
    }
    // ... repeated for each movement sample
  ],
  "name": "String, camera name",                           // Camera location/identifier 
  "serial": "String, device serial number"                 // Camera identity 
}
