"""
APEX_PREDATOR_V3 - Serial Link Manager
Handles thread-safe, ACK-based communication with the Arduino firmware.
"""

import serial
import time
import logging
import threading

logger = logging.getLogger('APEX_SERIAL')

class SerialLink:
    def __init__(self, port='COM3', baudrate=115200, timeout=2.0):
        """
        Initialize the Serial connection.
        :param port: The COM port (Windows) or /dev/tty... (Linux/Mac)
        :param baudrate: Must match the Arduino firmware (115200)
        :param timeout: How long to wait for an ACK before throwing an error
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self.ser = None
        self.is_connected = False
        
        # Lock ensures that if the GUI and the AI engine both try to send 
        # a command at the exact same millisecond, they queue up safely.
        self.lock = threading.Lock()

    def connect(self):
        """Establish connection to the Arduino."""
        try:
            logger.info(f"Connecting to APEX hardware on {self.port} at {self.baudrate} baud...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            
            # When you open a serial port, Arduino automatically resets. 
            # We must wait for it to boot up.
            time.sleep(2) 
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.is_connected = True
            logger.info("✓ Serial connection established.")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Safely close the connection."""
        if self.ser and self.ser.is_open:
            # Disable motors for safety before disconnecting
            self.send_disable() 
            self.ser.close()
        self.is_connected = False
        logger.info("Serial connection closed.")

    def _send_and_wait(self, command, expected_ack):
        """
        Internal method to send a raw string and wait for a specific acknowledgment.
        """
        if not self.is_connected:
            logger.warning(f"Cannot send command '{command}'. Not connected.")
            return False

        with self.lock:
            try:
                # Format with newline and encode to bytes
                full_cmd = f"{command}\n".encode('utf-8')
                self.ser.reset_input_buffer() # Clear old junk
                self.ser.write(full_cmd)
                
                # Block and wait for the ACK
                start_time = time.time()
                while (time.time() - start_time) < self.timeout:
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode('utf-8').strip()
                        if response == expected_ack:
                            return True
                        elif response != "":
                            logger.debug(f"Unexpected response from Arduino: {response}")
                            
                    time.sleep(0.005) # Prevent CPU hogging
                    
                logger.error(f"Timeout waiting for ACK: '{expected_ack}' after sending '{command}'")
                return False
                
            except Exception as e:
                logger.error(f"Serial write error: {e}")
                self.is_connected = False
                return False

    # ═══════════════════════════════════════════════════════════════
    # HIGH-LEVEL COMMAND API
    # ═══════════════════════════════════════════════════════════════

    def send_move(self, base_angle, shoulder_angle, elbow_angle, roll_angle):
        """
        Send absolute joint angles to the Arduino.
        """
        # Format: M,45.00,90.00,-30.00,180.00
        cmd = f"M,{base_angle:.2f},{shoulder_angle:.2f},{elbow_angle:.2f},{roll_angle:.2f}"
        return self._send_and_wait(cmd, "ACK:MOVE")

    def send_gripper(self, angle):
        """
        Set the gripper servos (0 to 180 degrees).
        """
        # Format: G,90
        angle = max(0, min(180, int(angle))) # Clamp between 0 and 180
        cmd = f"G,{angle}"
        return self._send_and_wait(cmd, "ACK:GRIPPER")

    def send_enable(self):
        """Enable power to the stepper drivers."""
        return self._send_and_wait("E", "ACK:ENABLED")

    def send_disable(self):
        """Disable power to steppers (Emergency Stop / free-spin)."""
        return self._send_and_wait("D", "ACK:DISABLED")


# --- STANDALONE TEST BLOCK ---
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    # Change 'COM3' to whatever port your Arduino is on (e.g., '/dev/ttyACM0' on Linux/Mac)
    link = SerialLink(port='COM3')
    
    if link.connect():
        print("Testing Motor Enable...")
        link.send_enable()
        time.sleep(1)
        
        print("Testing Move Command (All joints to 0)...")
        link.send_move(0.0, 0.0, 0.0, 0.0)
        time.sleep(2)
        
        print("Testing Move Command (Base to 45 degrees)...")
        link.send_move(45.0, 0.0, 0.0, 0.0)
        time.sleep(2)
        
        print("Testing Gripper...")
        link.send_gripper(90)
        time.sleep(1)
        
        print("Disabling Motors...")
        link.send_disable()
        
        link.disconnect()