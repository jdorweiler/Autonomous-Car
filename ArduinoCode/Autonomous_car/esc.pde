// Steering servo.  Output from the PID is restricted to -500 to 500 then mapped to 1950 to 1050 for the servo PPM signal
void driveSteering(int steering) {
    steering = map(steering,-500,500,1950,1050);
    ser_steer.write(steering);
    localData[12] = steering; 
    return;    
}
