import sys
import cv2
import mediapipe as mp
from networktables import NetworkTables

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

NetworkTables.initialize(server="roborio-1540-frc.local")
nt = NetworkTables.getTable("mediapipe")

camera_index = int(sys.argv[1])
print(f"Opening camera {camera_index}")

cap = cv2.VideoCapture(camera_index)
with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # To improve performance, optionally mark the image as not writeable to pass by reference
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)

        if not results.pose_landmarks:
            continue

        left_ear = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_EAR]
        right_ear = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_EAR]
        head_width = abs(left_ear.x - right_ear.x)

        nt.putNumber("headWidth", head_width)
        print(f"Head width: {head_width}")

        nose_x = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x - 0.5) * 2
        nose_y = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].y - 0.5) * 2

        nt.putNumber("noseX", nose_x)
        nt.putNumber("noseY", nose_y)

        # Draw the pose annotation on the image
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

        cv2.imshow("pose", image)
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
