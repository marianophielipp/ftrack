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
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/head_forward_position_controller/commands',
            10
        )
        
        # Joint limits from URDF
        self.pan_limits = (-0.523, 0.523)  # head_0 joint limits in radians
        self.tilt_limits = (-0.35, 1.57)   # head_1 joint limits in radians
        
        # MediaPipe setup
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(static_image_mode=False, refine_landmarks=True)
        
        # Landmark indices for the eyes
        self.LEFT_EYE = 468  # iris center left
        self.RIGHT_EYE = 473  # iris center right
        
        # Focal length for pinhole model
        self.focal_length = 500.0
        
        # Start video capture
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.track_face)  # 10Hz update rate

    def track_face(self):
        success, frame = self.cap.read()
        if not success:
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
                
                # Print angles for debugging
                self.get_logger().info(f'Pan: {math.degrees(pan_angle):.2f}°, Tilt: {math.degrees(tilt_angle):.2f}°')
        
        cv2.imshow("Face Tracker (Green Point)", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    face_tracker = FaceTracker()
    rclpy.spin(face_tracker)
    face_tracker.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()