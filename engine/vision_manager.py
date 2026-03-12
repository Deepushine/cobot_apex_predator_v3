"""
APEX_PREDATOR_V3 - Vision Manager
Handles video streaming, threaded frame capture, and YOLOv8/v11 inference.
"""

import cv2
import threading
import time
import numpy as np
from ultralytics import YOLO
import logging

logger = logging.getLogger('APEX_VISION')

class VisionManager:
    def __init__(self, model_path='best.pt', camera_source=0, conf_threshold=0.60):
        """
        Initialize the Vision Manager.
        :param model_path: Path to the trained YOLO weights (e.g., 'best.pt')
        :param camera_source: Integer for webcam (0) OR string for ESP32-CAM stream URL
        :param conf_threshold: Minimum confidence to register a detection
        """
        self.camera_source = camera_source
        self.conf_threshold = conf_threshold
        
        logger.info(f"Loading YOLO model from {model_path}...")
        # Load the PyTorch model weights you trained
        self.model = YOLO(model_path) 
        
        # Threading variables for continuous frame capture
        self.cap = None
        self.current_frame = None
        self.running = False
        self.lock = threading.Lock()
        
        self.start_stream()

    def start_stream(self):
        """Start the video capture in a background thread."""
        self.cap = cv2.VideoCapture(self.camera_source)
        
        # Optimize buffer size for IP cameras to reduce latency
        if isinstance(self.camera_source, str):
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
        if not self.cap.isOpened():
            logger.error(f"Failed to open camera source: {self.camera_source}")
            return False
            
        self.running = True
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()
        logger.info(f"Vision stream started on source: {self.camera_source}")
        return True

    def _update_frame(self):
        """Continuously grab the latest frame (runs in background)."""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.current_frame = frame
            else:
                logger.warning("Dropped frame or stream disconnected. Reconnecting...")
                time.sleep(1)
                self.cap.open(self.camera_source) # Try to auto-reconnect

    def get_detections(self):
        """
        Run inference on the newest frame and return the results.
        :return: Tuple (annotated_frame, list_of_detections)
                 list_of_detections format: [{'class': name, 'center': (cx, cy), 'bbox': (x1,y1,x2,y2), 'conf': c}]
        """
        with self.lock:
            if self.current_frame is None:
                return None, []
            frame = self.current_frame.copy()

        # Run YOLO inference
        results = self.model(frame, conf=self.conf_threshold, verbose=False)[0]
        
        detections = []
        
        # Parse the YOLO bounding boxes
        for box in results.boxes:
            # Extract coordinates and convert to integers
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            
            # Calculate the geometric center of the bounding box
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            detections.append({
                'class': cls_name,
                'center': (cx, cy),
                'bbox': (x1, y1, x2, y2),
                'conf': conf
            })
            
            # Draw targeting crosshairs on the center (for visual feedback)
            cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 15, 2)

        # Let Ultralytics plot its default bounding boxes on the frame
        annotated_frame = results.plot()

        return annotated_frame, detections

    def stop(self):
        """Cleanly shut down the camera thread."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.cap:
            self.cap.release()
        logger.info("Vision stream stopped.")


# --- STANDALONE TEST BLOCK ---
# If you run this file directly, it will test your camera and model.
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    # Example: For PC webcam use 0. 
    # For ESP32-CAM use "http://192.168.1.XX:81/stream"
    vision = VisionManager(model_path='best.pt', camera_source=0)
    
    try:
        while True:
            frame, detections = vision.get_detections()
            
            if frame is not None:
                # Print out what the AI sees
                for d in detections:
                    print(f"Detected: {d['class']} at {d['center']} with {d['conf']:.2f} certainty")
                
                cv2.imshow("APEX_PREDATOR Vision Test", frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            time.sleep(0.03) # Limit print spam to ~30 FPS
            
    except KeyboardInterrupt:
        pass
    finally:
        vision.stop()
        cv2.destroyAllWindows()