import serial
import time
import csv

X_seconds = 2
port = '/dev/cu.usbserial-DM02IZXW'
chunk_size = 1024

ser = serial.Serial(port, timeout=X_seconds)

starting_time = time.time()

while True:
    try:
        ser_bytes = ser.read(chunk_size)
        if len(ser_bytes)>0:
            end_time = time.time()
            time_elapsed = end_time - starting_time
            
            if time_elapsed>=X_seconds:
                #save data
            else:
                #continue 






        decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        print(decoded_bytes)
        with open("test_data.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([time.time(),decoded_bytes])
    except:
        print("Keyboard Interrupt")
        break



  