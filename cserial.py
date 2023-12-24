import serial
from datetime import datetime

# Define serial port and baud rate
ser = serial.Serial('COM3', 9600)  # Change 'COM3' to your Arduino's serial port

# Create a filename with current date and time
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"ACC_HR_data_{current_time}.csv"

with open(filename, 'w') as file:
    file.write("Timestamp,ACC,HR,RAW1,RAW2\n")  # Writing CSV header

    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip().split(',')
            if len(data) == 4:  # Ensure all three values (X, Y, Z) are received
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                file.write(f"{timestamp},{','.join(data)}\n")  # Writing data in CSV format

ser.close()