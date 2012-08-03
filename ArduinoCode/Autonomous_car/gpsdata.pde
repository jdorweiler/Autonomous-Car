// TinyGPS library by MIkal Hart, http://arduiniana.org/libraries/tinygps/
// Adapted to use on the autonomous car
static void gpsdump(TinyGPS &gps, float WayPt_Lat, float WayPt_Lon)
{
  float flat, flon;
  
  gps.f_get_position(&flat, &flon);
  localData[0] = flat*1000; //mutiply to get correct decimal number
  localData[1] = flon*1000; //mutiply to get correct decimal number
  localData[2] = gps.f_course();
  localData[3] = gps.course_to(flat, flon, WayPt_Lat, WayPt_Lon);
  localData[4] = float(gps.distance_between(flat, flon, WayPt_Lat, WayPt_Lon)); //Distance to waypoint
  localData[5] = gps.f_speed_kmph();
  localData[16] = 1; //new gps update

}  

static bool feedgps()
{
  while (Serial1.available())
  {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
  
  
}
