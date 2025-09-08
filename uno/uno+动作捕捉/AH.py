import cv2
import mediapipe as mp
import numpy as np
import serial
import time
from math import acos, degrees

# ===== 串口设置 =====
SERIAL_PORT = 'COM3'
BAUDRATE = 9600

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)  # 等待串口初始化

# ===== MediaPipe 初始化 =====
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

def vector(a, b):
    return np.array([b.x - a.x, b.y - a.y, b.z - a.z])

def angle_between(v1, v2):
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
    return degrees(acos(cos_angle))

def hand_local_coords(hand_landmarks):
    """
    构建手部局部坐标系
    返回旋转矩阵 R (世界坐标 -> 手部坐标)
    """
    origin = np.array([hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,
                       hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y,
                       hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].z])
    mid_mcp = np.array([hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x,
                        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y,
                        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].z])
    pinky_mcp = np.array([hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x,
                          hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y,
                          hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].z])
    
    # Z轴沿中指方向
    z_axis = mid_mcp - origin
    z_axis /= np.linalg.norm(z_axis)
    # X轴为掌心法向
    x_axis = np.cross(pinky_mcp - origin, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    # Y轴
    y_axis = np.cross(z_axis, x_axis)
    
    R = np.array([x_axis, y_axis, z_axis]).T  # 世界 -> 手部局部
    return origin, R

def transform_to_local(vec, R):
    return R.T @ vec

def get_finger_mcp_angle_local(hand_landmarks, R, origin, finger):
    if finger == 'thumb':
        joints = [mp_hands.HandLandmark.THUMB_CMC,
                  mp_hands.HandLandmark.THUMB_MCP,
                  mp_hands.HandLandmark.THUMB_IP]
    elif finger == 'index':
        joints = [mp_hands.HandLandmark.INDEX_FINGER_MCP,
                  mp_hands.HandLandmark.INDEX_FINGER_PIP,
                  mp_hands.HandLandmark.INDEX_FINGER_DIP]
    elif finger == 'middle':
        joints = [mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
                  mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
                  mp_hands.HandLandmark.MIDDLE_FINGER_DIP]
    elif finger == 'ring':
        joints = [mp_hands.HandLandmark.RING_FINGER_MCP,
                  mp_hands.HandLandmark.RING_FINGER_PIP,
                  mp_hands.HandLandmark.RING_FINGER_DIP]
    elif finger == 'pinky':
        joints = [mp_hands.HandLandmark.PINKY_MCP,
                  mp_hands.HandLandmark.PINKY_PIP,
                  mp_hands.HandLandmark.PINKY_DIP]
    else:
        return 0

    p0 = np.array([hand_landmarks.landmark[joints[0]].x,
                   hand_landmarks.landmark[joints[0]].y,
                   hand_landmarks.landmark[joints[0]].z]) - origin
    p1 = np.array([hand_landmarks.landmark[joints[1]].x,
                   hand_landmarks.landmark[joints[1]].y,
                   hand_landmarks.landmark[joints[1]].z]) - origin
    p2 = np.array([hand_landmarks.landmark[joints[2]].x,
                   hand_landmarks.landmark[joints[2]].y,
                   hand_landmarks.landmark[joints[2]].z]) - origin

    v1 = transform_to_local(p1 - p0, R)
    v2 = transform_to_local(p2 - p1, R)
    return angle_between(v1, v2)

def map_angle(angle, finger):
    if finger == 'thumb':
        min_angle, max_angle = 10, 30
    else:
        min_angle, max_angle = 10, 115
    
    # 限幅，防止越界
    angle = max(min(angle, max_angle), min_angle)
    mapped = (angle - min_angle) / (max_angle - min_angle) * 180
    return int(mapped)


def main():

    cap = cv2.VideoCapture(0)
    with mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:

        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    if handedness.classification[0].label != 'Right':
                        continue  # 只处理右手

                    origin, R = hand_local_coords(hand_landmarks)

                    try:
                        thumb_angle = map_angle(get_finger_mcp_angle_local(hand_landmarks, R, origin, 'thumb'), 'thumb')
                        index_angle = map_angle(get_finger_mcp_angle_local(hand_landmarks, R, origin, 'index'), 'other')
                        middle_angle = map_angle(get_finger_mcp_angle_local(hand_landmarks, R, origin, 'middle'), 'other')
                        pinky_angle = map_angle(get_finger_mcp_angle_local(hand_landmarks, R, origin, 'pinky'), 'other')
                    except Exception as e:
                        print("Angle calculation error:", e)
                        continue


                    msg = f"{thumb_angle},{index_angle},{middle_angle},{pinky_angle}\n"
                    print(thumb_angle, index_angle, middle_angle, pinky_angle)

                    ser.write(msg.encode('utf-8'))

                    print(msg)

                    time.sleep(0.05)  # 20Hz更新


                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
