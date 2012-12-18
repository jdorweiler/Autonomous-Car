/* 
This function moves the FIFO data in the gps heading array.  Then checks to see the differance between the new
heading and the average heading (kept in a global variable).  If it's greater than 180, subtract 360 to get the 
actual heading.  As an example, this takes care of the case where the average heading is 359 degree and an update 
is 1 degree.  This means that the new average could be greater than 360 in some cases but the averaged heading 
is clamped to 0-360 later on anyway. 




*/
void gpsAverage() {
     // Moving the values in the array up one to make room for the new update in gpsAvg[0]
     for (int i = 5; i > 0; --i){
         gpsAvg[i] = gpsAvg[(i-1)];
      }
      
     // Check to see if the new difference between the average heading and new heading is greater than 180. 
     // This is looking for the case where the new heading jumps around the azimuth past 360 throwing the average off. 
     // i.e. the average of 359 deg and 1 deg should be 359.5 deg, not 180 deg.  
     if (((gpsSum/5) - localData[2]) >= 180){
         localData[2] -= 360.0;
     }
     //Updates the FIFO GPS heading array (gpsAvg).  Puts the new heading into gpsAvg[0].
     //localData[2] holds the new GPS heading from the gpsdump function 
     gpsAvg[0] = localData[2];
     
     
    // Calculates the average of the heading array 
     
     for (int i = 1; i < 6; i++){
         gpsSum = gpsSum + gpsAvg[i];
     }
     
     localData[2] = gpsSum/5; // put the averaged heading back into here.  localData[2] holds the smooth heading used for kalman/gyro calculations
     // clamp to 0-360
  /*   
     if ( abs(gpsAvg[0] - localData[2]) > 360 ) {
         while (localData[2] < 0) localData[2] += 360.0;
         while (localData[2] >= 360.0) localData[2] -= 360.0;
         
     }
   */  
     return;
}
