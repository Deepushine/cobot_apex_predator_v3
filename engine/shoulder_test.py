import serial
import time

print("Connecting to APEX hardware...")
# Change 'COM15' if your borrowed Arduino is on a different port
try:
    ser = serial.Serial('COM15', 115200, timeout=1)
    time.sleep(2)  # Give Arduino time to reboot
except Exception as e:
    print(f"Failed to connect: {e}")
    exit()

try:
    print("Locking motors (Enable)...")
    ser.write(b"E\n")
    time.sleep(2) # Stabilize under weight

    print("Lifting Shoulder to 45 degrees UP...")
    ser.write(b"M,0.0,45.0,0.0,0.0\n")
    print(f"Arduino replied: {ser.readline().decode().strip()}")
    time.sleep(3) # Hold and stabilize

    print("Moving Shoulder to -45 degrees (OPPOSITE)...")
    ser.write(b"M,0.0,-45.0,0.0,0.0\n")
    print(f"Arduino replied: {ser.readline().decode().strip()}")
    time.sleep(5) 

    print("Returning to zero...")
    ser.write(b"M,0.0,0.0,0.0,0.0\n")
    print(f"Arduino replied: {ser.readline().decode().strip()}")
    time.sleep(2)

finally:
    print("Unlocking motors (Disable)...")
    ser.write(b"D\n")
    ser.close()
    print("Test complete.")