import serial

port = "/dev/ttyUSB1"

ser = serial.Serial(port, 57600, timeout = 300)

while True:
        content = ser.readline()
        t = [int(i) for i in content.split(',')]
        fromArduino = t[0]
        sentBack = 5
        send = fromArduino*2
        ser.write(bytes(sentBack))

