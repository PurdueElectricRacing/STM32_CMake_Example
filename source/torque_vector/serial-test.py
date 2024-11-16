import serial

port = 'COM10'  
baudrate = 115200 

ser = serial.Serial(port, baudrate, timeout=1)

try:
    while True:
        if ser.in_waiting: 
            data = ser.read(ser.in_waiting)  
            print(data)
except KeyboardInterrupt:
    print("Interrupted")
finally:
    ser.close()
