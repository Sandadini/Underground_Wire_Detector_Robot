import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import subprocess
import time

# Set up Bluetooth device information
bluetooth_address = '00:22:12:02:5B:40'  # Your Bluetooth device address
rfcomm_port = '/dev/rfcomm0'
channel = '1'  # The Bluetooth channel

# Connect to the Bluetooth device using rfcomm
def connect_bluetooth():
    try:
        print(f"Connecting to {bluetooth_address} on {rfcomm_port}...")
        subprocess.run(['sudo', 'rfcomm', 'connect', rfcomm_port, bluetooth_address, channel], check=True)
        print(f"Connected to {bluetooth_address}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to connect to Bluetooth: {e}")
        return False
    return True

# Set up Bluetooth connection
bluetooth_port = rfcomm_port
baud_rate = 9600

# Pre-recorded baseline magnetic field magnitudes (adjust as needed)
baseline_magnitudes = [16000, 15995, 15990]  # Adjust your baseline values

# Threshold for detecting significant changes in the magnetic field
threshold = 1000  # Adjust based on tests

# Initialize the plot
fig, ax = plt.subplots()
magnitudes = []  # List to store magnetic flux (field magnitude)
steps = []       # List to store step count

if connect_bluetooth():
    time.sleep(2)  # Wait for connection to stabilize
    try:
        ser = serial.Serial(bluetooth_port, baud_rate)
        print(f"Connected to Bluetooth on {bluetooth_port}")

        def calculate_magnetic_field_magnitude(x, y, z):
            return np.sqrt(x**2 + y**2 + z**2)

        def update_plot(frame):
            if ser.in_waiting > 0:
                try:
                    # Read and decode the serial data
                    data = ser.readline().decode('utf-8').strip()
                    print(f"Received: {data}")
                    if data.startswith('Step:'):
                        # Parse step and magnetic field data
                        parts = data.split(',')
                        step = int(parts[0].split()[1])
                        x = int(parts[1].split()[1])
                        y = int(parts[2].split()[1])
                        z = int(parts[3].split()[1])

                        # Calculate the magnitude of the magnetic field
                        magnitude = calculate_magnetic_field_magnitude(x, y, z)

                        # Append the step count and magnetic flux magnitude
                        steps.append(step)
                        magnitudes.append(magnitude)

                        # Limit the size of the lists for smooth plotting
                        steps[:] = steps[-50:]
                        magnitudes[:] = magnitudes[-50:]

                        # Clear and redraw the plot
                        ax.clear()
                        ax.plot(steps, magnitudes, label='Magnetic Flux vs Steps', color='m')

                        # Add labels and legend
                        ax.set_xlabel('Number of Steps')
                        ax.set_ylabel('Magnetic Flux (Field Magnitude)')
                        ax.legend()

                except Exception as e:
                    print(f"Error: {e}")

        # Set up animation for live plotting
        ani = animation.FuncAnimation(fig, update_plot, interval=300)
        plt.show()

    except serial.SerialException as e:
        print(f"Could not open port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")
else:
    print("Failed to establish Bluetooth connection.")
