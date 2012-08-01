int driveSteering(int steering) {
    steering = map(steering,-500,500,1950,1050);
    ser_steer.write(steering);
    localData[11] = steering;     
}
