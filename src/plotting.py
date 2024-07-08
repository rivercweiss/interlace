import numpy as np
import time
# from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial.tools.list_ports

array_size = 1000

# Get a list of all available COM ports
ports = list(serial.tools.list_ports.comports())

# Iterate through the list of ports and check if Arduino is connected
arduino_port = None
for port in ports:
    print(port)
    #if "USB-SERIAL" in port.description:
    if "Arduino" in port.description:
        arduino_port = port.device
        break

# If Arduino is found, establish a serial connection
if arduino_port:
    try:
        ser = serial.Serial(arduino_port, 9600)  # Replace 9600 with your desired baud rate
        print("Serial connection established with Arduino on port:", arduino_port)
        
        # Your code to communicate with Arduino goes here
        
       # ser.close()  # Close the serial connection when done
    except serial.SerialException as e:
        print("Failed to establish serial connection:", str(e))
else:
    print("Arduino not found on any COM port.")
time.sleep(0.1)
#line = ser.readline().decode("utf-8").strip()
# Send a character to the serial connection
dat = np.zeros((3, array_size))

for z in range(array_size):
    # print("Sending data to Arduino...")
    # ser.write(b'/n')
    # time.sleep(0.1)
    # Read a line of data from the serial connection
    line = ser.readline().decode("utf-8").strip()
    # print(line)
    # Parse line into a numpy array
    data_array = np.fromstring(line, dtype=float, sep=',')
    # print(data_array)
    # Pause for 100 milliseconds
    # time.sleep(0.1)
    dat[0, z] = data_array[0]
    dat[1, z] = data_array[1]
    dat[2, z] = data_array[2]


ser.close()  # Close the serial connection when done
# Create a scatter plot of dat
# Fit dat to an ellipse

print("Standard Deviation")
print(dat[0,:].std(),dat[1,:].std(),dat[2,:].std())
print("Max")
print(dat[0,:].max(),dat[1,:].max(),dat[2,:].max())
print("Min")
print(dat[0,:].min(),dat[1,:].min(),dat[2,:].min())

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(dat[0], dat[1], dat[2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Scatter Plot of data')

plt.show()