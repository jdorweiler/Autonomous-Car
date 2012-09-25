/// Waypoints are entered into an array.  Starting at 0 with the first waypoint.
// check the distance and move to the next waypoint if close enough.

void waypoint(float dist) {
  
  if (dist < 5) {
    num = num + 1;
    localData[10] = num; // Store the current waypoint number for logging
  }
  
WayPT_Lat = waypoints[num][0];
WayPT_Lon = waypoints[num][1];
return; 
}
