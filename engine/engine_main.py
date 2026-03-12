"""
APEX_PREDATOR_V3 - Main Execution Engine
Ties together Vision, Kinematics, and Serial Communication into a cohesive state machine.
"""

import time
import cv2
import logging
from vision_manager import VisionManager
from serial_link import SerialLink
from kinematics import Kinematics

# Configure global logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger('APEX_MAIN')

class ApexEngine:
    def __init__(self):
        logger.info("Initializing APEX PREDATOR V3 Engine...")
        
        # --- CONFIGURATION ---
        self.camera_source = 0  # Change to "http://192.168.1.XX:81/stream" for ESP32-CAM
        self.com_port = 'COM15'  # Change to your Arduino's COM port
        self.target_class = '' # The YOLO class you want to track
        
        # --- MODULE INITIALIZATION ---
        self.vision = VisionManager(model_path='best.pt', camera_source=self.camera_source)
        self.serial = SerialLink(port=self.com_port, baudrate=115200)
        self.kinematics = Kinematics()
        
        # --- WORKSPACE CALIBRATION ---
        # How high the robot hovers above the table (in mm) while tracking
        self.hover_height_z = 30.0 
        
        self.is_running = False

    def pixel_to_world(self, px, py, frame_width, frame_height):
        """
        Converts a 2D camera pixel coordinate into a 3D robot workspace coordinate (in mm).
        *Assumes the camera is mounted looking down at the workspace.*
        """
        # TODO: You will need to calibrate these scaling factors!
        # Measure how many millimeters of the table the camera can see.
        # Example: If the camera sees 300mm of width across 640 pixels, scale = 300/640.
        x_mm_per_pixel = 0.5 
        y_mm_per_pixel = 0.5 
        
        # Origin Offset: The distance from the Robot's base (0,0) to the Camera's top-left pixel (0,0)
        offset_x = 100.0  # mm forward from robot base
        offset_y = -150.0 # mm left/right from robot base
        
        # Calculate world coordinates
        world_x = offset_x + (py * y_mm_per_pixel)  # Y pixel maps to X forward depth
        world_y = offset_y + (px * x_mm_per_pixel)  # X pixel maps to Y left/right width
        
        return world_x, world_y

    def run(self):
        """Main control loop."""
        if not self.serial.connect():
            logger.error("Could not connect to hardware. Running in Vision-Only SIMULATION mode.")
            # We don't return here so you can still test the AI tracking without hardware connected
        else:
            self.serial.send_enable()

        self.is_running = True
        logger.info("Engine running. Press 'q' in the video window to quit.")

        try:
            while self.is_running:
                loop_start = time.time()
                
                # 1. Get latest vision data
                frame, detections = self.vision.get_detections()
                
                if frame is None:
                    continue

                # 2. Logic: Find our primary target
                target = None
                for det in detections:
                    if det['class'] == self.target_class:
                        target = det
                        break # Just track the first one found for now
                
                if target:
                    cx, cy = target['center']
                    h, w = frame.shape[:2]
                    
                    # 3. Transform Vision to Real-World Coordinates
                    world_x, world_y = self.pixel_to_world(cx, cy, w, h)
                    
                    # 4. Calculate Inverse Kinematics
                    angles = self.kinematics.inverse_kinematics(
                        target_x=world_x, 
                        target_y=world_y, 
                        target_z=self.hover_height_z
                    )
                    
                    if angles:
                        base_deg, shoulder_deg, elbow_deg, roll_deg = angles
                        
                        # 5. Send to Hardware
                        if self.serial.is_connected:
                            self.serial.send_move(base_deg, shoulder_deg, elbow_deg, roll_deg)
                        
                        # Draw the calculated target on the screen for debugging
                        cv2.putText(frame, f"IK: X{world_x:.0f} Y{world_y:.0f}", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(frame, "TARGET OUT OF REACH", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # 6. Display the GUI feed
                cv2.imshow("APEX_PREDATOR_V3 Control Center", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.is_running = False
                
                # Regulate loop speed to ~10Hz (Plenty fast for physical mechanics)
                elapsed = time.time() - loop_start
                if elapsed < 0.1:
                    time.sleep(0.1 - elapsed)

        except KeyboardInterrupt:
            logger.info("User interrupted.")
        finally:
            self.shutdown()

    def shutdown(self):
        """Safe shutdown sequence."""
        logger.info("Shutting down APEX Engine...")
        self.is_running = False
        self.vision.stop()
        self.serial.disconnect()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete.")

if __name__ == "__main__":
    engine = ApexEngine()
    engine.run()
