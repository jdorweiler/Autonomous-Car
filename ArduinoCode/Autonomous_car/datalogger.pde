void logger(){
  // open the file
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {  
      for ( int i = 0; i < 12; i = i + 1) {
          dataFile.print(localData[i]); dataFile.print(" ");
           } 
      dataFile.println(" ");
      dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  } 
return;
}

