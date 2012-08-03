void logger(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
      for ( int i = 0; i < 17; i = i + 1) {
          dataFile.print(localData[i]); dataFile.print(" ");
           } 
      dataFile.println(" ");
      dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  } 

}

