import serial
import time

# Open serial port
ser = serial.Serial('COM5', 115200)  # replace COM3 with your Arduino's COM port
ser.flushInput()

# Open or create a file for data logging
with open("sensor_data.txt", "w") as f:
    while True:
        try:
            ser_bytes = ser.readline()
            decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")  # decode bytes to string
            print(decoded_bytes)
            
            # Log to file
            f.write(decoded_bytes + "\n")
            f.flush()
            
            # Optionally, sleep for a short period of time
            time.sleep(1)
        except:
            print("Error")
            break
