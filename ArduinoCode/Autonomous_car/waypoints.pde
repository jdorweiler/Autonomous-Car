/// Waypoints are entered into an array.  Starting at 0 with the first waypoint.
// check the distance and move to the next waypoint if close enough.

void waypoint(int num,float dist) {
  
  if (dist < 0.01) {
    num += num;
  }
  
 waypoints[0][0] = 40.588853;
 waypoints[0][1] = -74.892592;
  
 waypoints[1][0] = 40.589036;
 waypoints[1][1] = -74.892361;
 
 WayPT_Lat = waypoints[num][0];
 WayPT_Lon = waypoints[num][1];
}
