"""
APEX_PREDATOR_V3 - Kinematics Engine
Calculates Inverse Kinematics (IK) to convert X,Y,Z Cartesian coordinates 
into Base, Shoulder, and Elbow joint angles.
"""

import math
import logging

logger = logging.getLogger('APEX_KINEMATICS')

class Kinematics:
    def __init__(self):
        # --- PHYSICAL MEASUREMENTS (in mm) ---
        # The raw lengths of your links as measured
        self.raw_link1 = 101.23  # Shoulder to Elbow link
        self.raw_link2 = 54.90   # Elbow to Roll link
        
        # IMPORTANT: Distance from the edge of the link to the actual center of the motor shaft.
        # You will need to measure your 3D printed parts and adjust these!
        self.shoulder_pivot_offset = 15.0 
        self.elbow_pivot_offset = 15.0
        self.wrist_pivot_offset = 10.0

        # True Kinematic Lengths (Pivot to Pivot)
        self.L1 = 276.93
        self.L2 = 202.85
        
        # Height of the shoulder joint from the table/base
        self.base_height = 80.0 

        logger.info(f"Kinematics Initialized: True L1={self.L1:.1f}mm, True L2={self.L2:.1f}mm")
        logger.info(f"Maximum Safe Reach: {(self.L1 + self.L2):.1f}mm")

    def inverse_kinematics(self, target_x, target_y, target_z, target_roll=0.0):
        """
        Calculates the joint angles required to reach a specific X, Y, Z coordinate.
        
        :param target_x: Forward/Backward distance (mm)
        :param target_y: Left/Right distance (mm)
        :param target_z: Up/Down distance from table (mm)
        :param target_roll: Desired roll angle of the end effector (degrees)
        :return: Tuple of (base_deg, shoulder_deg, elbow_deg, roll_deg) or None if unreachable
        """
        try:
            # 1. Base Angle (Rotation around Z-axis)
            # atan2 handles the correct quadrant automatically
            base_rad = math.atan2(target_y, target_x)
            
            # 2. 2D Planar Projection
            # Calculate the horizontal distance from the base center to the target
            r = math.sqrt(target_x**2 + target_y**2)
            
            # Adjust Z to be relative to the shoulder pivot, not the table
            z_adj = target_z - self.base_height = 54.0
            
            # Calculate the direct straight-line distance from shoulder to target
            d = math.sqrt(r**2 + z_adj**2)

            # Check if the target is physically reachable
            if d > (self.L1 + self.L2):
                logger.warning(f"Target ({target_x}, {target_y}, {target_z}) is OUT OF REACH. Distance: {d:.1f}mm")
                return None
                
            if d < abs(self.L1 - self.L2):
                logger.warning(f"Target ({target_x}, {target_y}, {target_z}) is TOO CLOSE (self-collision risk).")
                return None

            # 3. Elbow Angle using Law of Cosines
            # We want the "Elbow Up" configuration by default (negative angle)
            cos_elbow = (d**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            # Clamp value between -1 and 1 to prevent floating point math errors
            cos_elbow = max(-1.0, min(1.0, cos_elbow)) 
            elbow_rad = -math.acos(cos_elbow)

            # 4. Shoulder Angle
            # Alpha is the angle to the target coordinate
            alpha = math.atan2(z_adj, r)
            # Beta is the internal angle of the triangle formed by the links
            cos_beta = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
            cos_beta = max(-1.0, min(1.0, cos_beta))
            beta = math.acos(cos_beta)
            
            # Final shoulder angle is the angle to the target PLUS the internal triangle angle
            shoulder_rad = alpha + beta

            # 5. Convert radians to degrees for the Arduino
            base_deg = math.degrees(base_rad)
            shoulder_deg = math.degrees(shoulder_rad)
            elbow_deg = math.degrees(elbow_rad)

            return (base_deg, shoulder_deg, elbow_deg, target_roll)

        except Exception as e:
            logger.error(f"IK Calculation Error: {e}")
            return None

# --- STANDALONE TEST BLOCK ---
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    kin = Kinematics()
    
    # Test a coordinate straight ahead
    print("\nTest 1: Reaching Forward (X=150, Y=0, Z=100)")
    angles = kin.inverse_kinematics(150, 0, 100)
    if angles:
        print(f"Result -> Base: {angles[0]:.1f}°, Shoulder: {angles[1]:.1f}°, Elbow: {angles[2]:.1f}°")

    # Test an unreachable coordinate
    print("\nTest 2: Reaching Too Far (X=500, Y=0, Z=100)")
    angles = kin.inverse_kinematics(500, 0, 100)
    if not angles:
        print("Result -> Successfully prevented unreachable move.")