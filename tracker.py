# import cv2
# import mediapipe as mp
# import numpy as np
# import math
# import socket
# import struct
# import argparse

# # --- Parse command-line arguments ---
# parser = argparse.ArgumentParser(description="Face Tracker UDP Sender")
# parser.add_argument("--ip", default="192.168.30.1", help="Robot controller IP (default: 127.0.0.1)")
# parser.add_argument("--port", type=int, default=5173, help="UDP port (default: 5173)")
# args = parser.parse_args()

# # --- Set up UDP socket ---
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# # --- MediaPipe setup ---
# mp_face_mesh = mp.solutions.face_mesh
# face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True)

# LEFT_EYE_CENTER = 468
# RIGHT_EYE_CENTER = 473
# UPPER_LIP_CENTER = 13

# cap = cv2.VideoCapture(0)

# while cap.isOpened():
#     success, frame = cap.read()
#     if not success:
#         break

#     h, w, _ = frame.shape
#     rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     result = face_mesh.process(rgb)

#     if result.multi_face_landmarks:
#         for face_landmarks in result.multi_face_landmarks:
#             def get_coords(idx):
#                 lm = face_landmarks.landmark[idx]
#                 return np.array([lm.x * w, lm.y * h, lm.z * w])

#             left_eye = get_coords(LEFT_EYE_CENTER)
#             right_eye = get_coords(RIGHT_EYE_CENTER)
#             lip = get_coords(UPPER_LIP_CENTER)
#             eye_center = (left_eye + right_eye) / 2.0

#             v1 = right_eye - left_eye
#             v2 = lip - eye_center
#             normal = np.cross(v1, v2)
#             normal /= np.linalg.norm(normal) + 1e-6

#             # Draw for visualization
#             end = (eye_center[:2] + normal[:2] * 50).astype(int)
#             cv2.circle(frame, tuple(eye_center[:2].astype(int)), 4, (255, 0, 255), -1)
#             cv2.circle(frame, tuple(lip[:2].astype(int)), 4, (0, 255, 0), -1)
#             cv2.line(frame, tuple(eye_center[:2].astype(int)), tuple(end), (0, 0, 255), 2)

#             nx, ny, nz = normal
#             if abs(nz) < 1e-6:
#                 nz = 1e-6

#             pan_angle = math.degrees(math.atan2(nx, nz))
#             tilt_angle = math.degrees(math.atan2(-ny, nz))

#             print(f"Pan: {pan_angle}, Tilt: {tilt_angle}")

#             # Send via UDP
#             message = struct.pack('ff', pan_angle, tilt_angle)
#             sock.sendto(message, (args.ip, args.port))

#     cv2.imshow("Face Tracker", frame)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()

import cv2
import mediapipe as mp
import numpy as np
import math
<<<<<<< HEAD
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.get_logger().info('Initializing Face Tracker node...')
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/head_forward_position_controller/commands',
            10
        )
        self.get_logger().info('Created publisher for /head_forward_position_controller/commands')
        
        # Joint limits from URDF
        self.pan_limits = (-0.523, 0.523)  # head_0 joint limits in radians
        self.tilt_limits = (-0.35, 1.57)   # head_1 joint limits in radians
        self.get_logger().info(f'Joint limits set - Pan: {self.pan_limits}, Tilt: {self.tilt_limits}')
        
        # MediaPipe setup
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(static_image_mode=False, refine_landmarks=True)
        self.get_logger().info('MediaPipe face mesh initialized')
        
        # Landmark indices for the eyes
        self.LEFT_EYE = 468  # iris center left
        self.RIGHT_EYE = 473  # iris center right
        
        # Focal length for pinhole model
        self.focal_length = 500.0
        
        # Start video capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture device')
            return
        self.get_logger().info('Video capture device opened successfully')
        
        self.timer = self.create_timer(0.1, self.track_face)  # 10Hz update rate
        self.get_logger().info('Face tracking timer started at 10Hz')

    def track_face(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warning('Failed to read frame from video capture')
            return

        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.face_mesh.process(rgb)

        if result.multi_face_landmarks:
            for face_landmarks in result.multi_face_landmarks:
                def get_coords(idx):
                    lm = face_landmarks.landmark[idx]
                    return np.array([lm.x * w, lm.y * h])
                
                left_eye_pt = get_coords(self.LEFT_EYE)
                right_eye_pt = get_coords(self.RIGHT_EYE)
                
                # Compute green point (midpoint between eyes)
                green_point = (left_eye_pt + right_eye_pt) / 2.0
                
                # Draw a green circle at the green point for visualization
                cv2.circle(frame, tuple(green_point.astype(int)), 5, (0, 255, 0), -1)
                
                # Compute the offset from the image center
                center = np.array([w / 2, h / 2])
                offset = green_point - center

                # Compute raw angles
                pan_angle = -math.atan2(offset[0], self.focal_length)  # in radians
                tilt_angle = math.atan2(offset[1], self.focal_length)  # in radians
                
                # Scale angles to joint limits
                pan_angle = np.clip(pan_angle, self.pan_limits[0], self.pan_limits[1])
                tilt_angle = np.clip(tilt_angle, self.tilt_limits[0], self.tilt_limits[1])
                
                # Create and publish ROS message
                msg = Float64MultiArray()
                msg.data = [pan_angle, tilt_angle]
                self.publisher.publish(msg)
                
                # Log detailed information
                self.get_logger().info(f'Face detected - Raw angles: Pan={math.degrees(pan_angle):.2f}°, Tilt={math.degrees(tilt_angle):.2f}°')
                self.get_logger().info(f'Published message: {msg.data}')
                self.get_logger().info(f'Green point position: x={green_point[0]:.2f}, y={green_point[1]:.2f}')
                self.get_logger().info(f'Image center: x={center[0]:.2f}, y={center[1]:.2f}')
                self.get_logger().info(f'Offset: x={offset[0]:.2f}, y={offset[1]:.2f}')
        else:
            self.get_logger().info('No face detected in frame')
        
        cv2.imshow("Face Tracker (Green Point)", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info('ESC pressed, shutting down...')
            self.destroy_node()
            rclpy.shutdown()
=======
import socket
import struct
import argparse
from collections import deque

# --- Parse command-line arguments ---
parser = argparse.ArgumentParser(description="Face Tracker UDP Sender (Green Point Estimation)")
parser.add_argument("--ip", default="127.0.0.1", help="Robot controller IP (default: 127.0.0.1)")
parser.add_argument("--port", type=int, default=65432, help="UDP port (default: 65432)")
parser.add_argument("--looking-ip", default="127.0.0.1", help="IP for looking status (default: 127.0.0.1)")
parser.add_argument("--looking-port", type=int, default=65433, help="Port for looking status (default: 65433)")
parser.add_argument("--threshold", type=float, default=0.25, help="Gaze threshold (default: 0.25)")
args = parser.parse_args()

# --- Set up UDP sockets ---
tracker_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
looking_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- MediaPipe setup ---
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, 
                                  max_num_faces=1,
                                  refine_landmarks=True,
                                  min_detection_confidence=0.5,
                                  min_tracking_confidence=0.5)

# Landmark indices for the eyes
LEFT_IRIS = 468  # iris center left
RIGHT_IRIS = 473  # iris center right

# Eye contour landmarks for better detection
LEFT_EYE_CONTOUR = [33, 246, 161, 160, 159, 158, 157, 173, 133, 155, 154, 153, 145, 144, 163, 7]
RIGHT_EYE_CONTOUR = [263, 466, 388, 387, 386, 385, 384, 398, 362, 382, 381, 380, 374, 373, 390, 249]

# Buffer for smoothing
BUFFER_SIZE = 10
looking_buffer = deque([False] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

def get_eye_center(landmarks, eye_contour_indices):
    """Calculate the center of an eye using multiple contour points"""
    points = np.array([(landmarks[idx].x, landmarks[idx].y) for idx in eye_contour_indices])
    return np.mean(points, axis=0)

def is_looking_at_camera_improved(landmarks):
    """Use iris positions relative to eye centers to determine gaze direction"""
    # Get iris centers
    left_iris = np.array([landmarks[LEFT_IRIS].x, landmarks[LEFT_IRIS].y])
    right_iris = np.array([landmarks[RIGHT_IRIS].x, landmarks[RIGHT_IRIS].y])
    
    # Get eye centers using contours (more accurate than just corners)
    left_eye_center = get_eye_center(landmarks, LEFT_EYE_CONTOUR)
    right_eye_center = get_eye_center(landmarks, RIGHT_EYE_CONTOUR)
    
    # Calculate eye sizes
    left_eye_points = np.array([(landmarks[idx].x, landmarks[idx].y) for idx in LEFT_EYE_CONTOUR])
    right_eye_points = np.array([(landmarks[idx].x, landmarks[idx].y) for idx in RIGHT_EYE_CONTOUR])
    
    left_eye_width = np.max(left_eye_points[:,0]) - np.min(left_eye_points[:,0])
    right_eye_width = np.max(right_eye_points[:,0]) - np.min(right_eye_points[:,0])
    
    # Calculate gaze offsets from center
    left_gaze_offset = (left_iris - left_eye_center) / left_eye_width
    right_gaze_offset = (right_iris - right_eye_center) / right_eye_width
    
    # Average the gaze offsets
    avg_gaze_offset = (left_gaze_offset + right_gaze_offset) / 2
    
    # Get horizontal and vertical components
    gaze_x, gaze_y = avg_gaze_offset
    
    # Debug output
    print(f"Gaze offset: X={gaze_x:.3f}, Y={gaze_y:.3f}, Threshold={args.threshold}")
    
    # Check if gaze is within threshold (both horizontally and vertically)
    return abs(gaze_x) < args.threshold and abs(gaze_y) < args.threshold

def smooth_looking_state(current_state):
    """Apply smoothing to reduce flicker"""
    looking_buffer.append(current_state)
    # Count number of True values in buffer
    true_count = sum(looking_buffer)
    # Consider looking at camera if more than 60% of recent frames were looking
    return true_count >= BUFFER_SIZE * 0.6

# Choose a focal length (in pixels) for our pinhole model.
focal_length = 500.0  # Adjust as needed
>>>>>>> 99b25095e1b2332fb4582e762fdec8aa75cfd887

def main():
    rclpy.init()
    face_tracker = FaceTracker()
    try:
        rclpy.spin(face_tracker)
    except KeyboardInterrupt:
        face_tracker.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        face_tracker.cap.release()
        cv2.destroyAllWindows()
        face_tracker.destroy_node()
        rclpy.shutdown()

<<<<<<< HEAD
if __name__ == '__main__':
    main()
=======
print(f"Sending tracking data to {args.ip}:{args.port}")
print(f"Sending looking status to {args.looking_ip}:{args.looking_port}")

# Initialize variables for face detection
last_face_detected = False
face_detected_timestamp = 0

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = face_mesh.process(rgb)
    
    # Get current timestamp
    timestamp = int(cv2.getTickCount() / cv2.getTickFrequency() * 1000)  # milliseconds
    
    # Check if a face is detected
    face_detected = bool(result.multi_face_landmarks)
    
    # If face detection status changed, print a message
    if face_detected != last_face_detected:
        print(f"Face {'detected' if face_detected else 'lost'}")
        last_face_detected = face_detected
        face_detected_timestamp = timestamp
    
    # Always send face detection status
    face_message = struct.pack('??q', face_detected, False, timestamp)  # Default not looking
    looking_sock.sendto(face_message, (args.looking_ip, args.looking_port))

    if face_detected:
        face_landmarks = result.multi_face_landmarks[0]
        
        # Helper: convert normalized landmark to pixel coordinates
        def get_coords(idx):
            lm = face_landmarks.landmark[idx]
            return np.array([lm.x * w, lm.y * h])
        
        left_eye_pt = get_coords(LEFT_IRIS)
        right_eye_pt = get_coords(RIGHT_IRIS)
        
        # Compute green point (midpoint between eyes)
        green_point = (left_eye_pt + right_eye_pt) / 2.0
        
        # Get eye centers for visualization
        left_eye_center = get_eye_center(face_landmarks.landmark, LEFT_EYE_CONTOUR)
        right_eye_center = get_eye_center(face_landmarks.landmark, RIGHT_EYE_CONTOUR)
        left_eye_center_px = (int(left_eye_center[0] * w), int(left_eye_center[1] * h))
        right_eye_center_px = (int(right_eye_center[0] * w), int(right_eye_center[1] * h))
        
        # Use improved approach for gaze detection
        raw_looking = is_looking_at_camera_improved(face_landmarks.landmark)
        
        # Apply smoothing to avoid flicker
        looking_at_camera = smooth_looking_state(raw_looking)
        
        # Determine dot color based on looking status
        dot_color = (0, 255, 0) if looking_at_camera else (0, 0, 255)  # Green if looking, red if not
        
        # Draw visualization
        # Draw irises
        cv2.circle(frame, tuple(left_eye_pt.astype(int)), 5, (255, 0, 255), -1)
        cv2.circle(frame, tuple(right_eye_pt.astype(int)), 5, (255, 0, 255), -1)
        
        # Draw eye centers
        cv2.circle(frame, left_eye_center_px, 3, (0, 255, 255), -1)
        cv2.circle(frame, right_eye_center_px, 3, (0, 255, 255), -1)
        
        # Draw eye contours for visualization
        for idx in LEFT_EYE_CONTOUR:
            pt = get_coords(idx).astype(int)
            cv2.circle(frame, tuple(pt), 1, (0, 165, 255), -1)
        for idx in RIGHT_EYE_CONTOUR:
            pt = get_coords(idx).astype(int)
            cv2.circle(frame, tuple(pt), 1, (0, 165, 255), -1)
        
        # Draw the looking indicator dot
        cv2.circle(frame, tuple(green_point.astype(int)), 15, dot_color, -1)
        
        # Add text to indicate status
        status_text = "Looking at camera" if looking_at_camera else "Not looking at camera"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dot_color, 2)
        
        # Compute the offset from the image center
        center = np.array([w / 2, h / 2])
        offset = green_point - center

        # Compute angles based on a simple pinhole camera model
        pan_angle = -math.degrees(math.atan2(offset[0], focal_length))
        tilt_angle = math.degrees(math.atan2(offset[1], focal_length))
        
        print(f"Pan: {pan_angle:.2f}°, Tilt: {tilt_angle:.2f}°, Looking: {looking_at_camera}")
        
        # Send pan/tilt angles via UDP (as two floats)
        tracker_message = struct.pack('ff', pan_angle, tilt_angle)
        tracker_sock.sendto(tracker_message, (args.ip, args.port))
        
        # Send face detected and looking status via UDP 
        # Format: face_detected(bool), looking_at_camera(bool), timestamp(long)
        looking_message = struct.pack('??q', face_detected, looking_at_camera, timestamp)
        looking_sock.sendto(looking_message, (args.looking_ip, args.looking_port))
    else:
        # Display message when no face is detected
        cv2.putText(frame, "No face detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    cv2.imshow("Face Tracker", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

import socket
import struct
import time

# Set up receiver socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 65433))  # Listen on all interfaces, port 65433
print(f"Listening for looking status on port 65433...")

while True:
    try:
        data, addr = sock.recvfrom(1024)
        is_looking, timestamp = struct.unpack('?q', data)
        current_time = time.strftime('%H:%M:%S')
        print(f"[{current_time}] Looking at camera: {is_looking}, Timestamp: {timestamp}ms")
    except Exception as e:
        print(f"Error receiving data: {e}")
>>>>>>> 99b25095e1b2332fb4582e762fdec8aa75cfd887
