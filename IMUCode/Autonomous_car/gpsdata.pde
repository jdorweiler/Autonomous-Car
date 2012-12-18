// TinyGPS library by MIkal Hart, http://arduiniana.org/libraries/tinygps/
// Adapted to use on the autonomous car
static void gpsdump(TinyGPS &gps, float WayPt_Lat, float WayPt_Lon)
{
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  localData[0] = flat*1000; //Latitude.  mutiply to get correct decimal number
  localData[1] = flon*1000; //Longitude. mutiply to get correct decimal number
  localData[2] = gps.f_course();// Heading, two copies in 2 & 4. 2 for GPS comparison and 4 for GYRO only angles
  localData[4] = localData[2];
  localData[7] = float(gps.distance_between(flat, flon, WayPt_Lat, WayPt_Lon)); //Distance to waypoint
  localData[11] = 1;
  
  if (gpsFlag) 
      {
      localData[6] = gps.course_to(flat, flon, WayPt_Lat, WayPt_Lon);// Heading to waypoint is checked less frequently
      gpsFlag = 0;
   } 
  return;
}  

// Checks for new GPS signal
static bool feedgps()
{
  while (Serial1.available()){
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
}
