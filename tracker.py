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
import socket
import struct
import argparse

# --- Parse command-line arguments ---
parser = argparse.ArgumentParser(description="Face Tracker UDP Sender (Green Point Estimation)")
parser.add_argument("--ip", default="127.0.0.1", help="Robot controller IP (default: 127.0.0.1)")
parser.add_argument("--port", type=int, default=65432, help="UDP port (default: 65432)")
args = parser.parse_args()

# --- Set up UDP socket ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- MediaPipe setup ---
mp_face_mesh = mp.solutions.face_mesh
# For live video, set static_image_mode=False
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, refine_landmarks=True)

# Landmark indices for the eyes
LEFT_EYE = 468  # iris center left
RIGHT_EYE = 473  # iris center right

# Choose a focal length (in pixels) for our pinhole model.
# This value will affect the sensitivity of the computed angles.
focal_length = 500.0  # Adjust as needed

cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = face_mesh.process(rgb)

    if result.multi_face_landmarks:
        for face_landmarks in result.multi_face_landmarks:
            # Helper: convert normalized landmark to pixel coordinates (only x and y here)
            def get_coords(idx):
                lm = face_landmarks.landmark[idx]
                return np.array([lm.x * w, lm.y * h])
            
            left_eye_pt = get_coords(LEFT_EYE)
            right_eye_pt = get_coords(RIGHT_EYE)
            
            # Compute green point (midpoint between eyes)
            green_point = (left_eye_pt + right_eye_pt) / 2.0
            
            # Draw a green circle at the green point for visualization.
            cv2.circle(frame, tuple(green_point.astype(int)), 5, (0, 255, 0), -1)
            
            # Compute the offset from the image center.
            center = np.array([w / 2, h / 2])
            offset = green_point - center

            # Compute angles based on a simple pinhole camera model.
            # Pan: horizontal angle; Tilt: vertical angle.
            pan_angle = -math.degrees(math.atan2(offset[0], focal_length))
            tilt_angle = math.degrees(math.atan2(offset[1], focal_length))
            
            print(f"Pan: {pan_angle:.2f}°, Tilt: {tilt_angle:.2f}°")
            
            # Send via UDP (as two floats)
            message = struct.pack('ff', pan_angle, tilt_angle)
            sock.sendto(message, (args.ip, args.port))
    
    cv2.imshow("Face Tracker (Green Point)", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()