#!/usr/bin/env python
import pybullet as p
from numpy import pi
import time
import pybullet_data
import os
import serial
import tkinter as tk
import threading

# ================= 串口初始化 =================
ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)

# ================= PyBullet 初始化 =================
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, 'support', 'hand.urdf')
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(filename, startPos, startOrientation)

# 关节分配（请确认和 URDF 对应）
thumb_joints = [2, 3, 5]
index_joints = [11, 13, 15]
middle_joints = [19, 21, 23]
little_joints = [27, 29, 31]

# 当前角度
angles = {"thumb": 0, "index": 0, "middle": 0, "little": 0}
maxForce = 500

# ================= 控制函数 =================
def send_to_robot():
    """根据 Tkinter 的角度值控制 PyBullet + 串口"""
    global angles
    thumbAngle = angles["thumb"]
    indexAngle = angles["index"]
    middleAngle = angles["middle"]
    littleAngle = angles["little"]

    # 角度转弧度，限制在 0–45°
    thumbTarget  = [thumbAngle / 180.0 * 45 * pi / 180 for _ in thumb_joints]
    indexTarget  = [indexAngle / 180.0 * 45 * pi / 180 for _ in index_joints]
    middleTarget = [middleAngle / 180.0 * 45 * pi / 180 for _ in middle_joints]
    littleTarget = [littleAngle / 180.0 * 45 * pi / 180 for _ in little_joints]

    p.setJointMotorControlArray(robotId, thumb_joints, p.POSITION_CONTROL, targetPositions=thumbTarget, forces=[maxForce]*len(thumb_joints))
    p.setJointMotorControlArray(robotId, index_joints, p.POSITION_CONTROL, targetPositions=indexTarget, forces=[maxForce]*len(index_joints))
    p.setJointMotorControlArray(robotId, middle_joints, p.POSITION_CONTROL, targetPositions=middleTarget, forces=[maxForce]*len(middle_joints))
    p.setJointMotorControlArray(robotId, little_joints, p.POSITION_CONTROL, targetPositions=littleTarget, forces=[maxForce]*len(little_joints))

    msg = f"{thumbAngle},{indexAngle},{middleAngle},{littleAngle}\n"
    ser.write(msg.encode('utf-8'))
    print(msg, end="")

def sim_loop():
    """后台循环，保持 PyBullet 运行"""
    while True:
        send_to_robot()
        p.stepSimulation()
        time.sleep(1./240.)

# ================= Tkinter 界面 =================
def start_gui():
    global angles
    root = tk.Tk()
    root.title("Hand Control")

    # 四根手指滑条
    sliders = {}
    for finger in ["thumb", "index", "middle", "little"]:
        tk.Label(root, text=finger.capitalize()).pack()
        sliders[finger] = tk.Scale(root, from_=0, to=180, orient="horizontal",
                                   command=lambda val, f=finger: angles.update({f: int(val)}))
        sliders[finger].pack()

    # 开关按钮
    def open_hand():
        for f in sliders: sliders[f].set(0)
    def close_hand():
        for f in sliders: sliders[f].set(180)
    def ok_hand():
        sliders["thumb"].set(180)
        sliders["index"].set(180)
        sliders["middle"].set(0)
        sliders["little"].set(0)
    def point_hand():
        sliders["thumb"].set(180)
        sliders["index"].set(0)
        sliders["middle"].set(180)
        sliders["little"].set(180)
    def yeah_hand():
        sliders["thumb"].set(180)
        sliders["index"].set(0)
        sliders["middle"].set(0)
        sliders["little"].set(180)


    tk.Button(root, text="Open Hand", command=open_hand).pack()
    tk.Button(root, text="Close Hand", command=close_hand).pack()
    tk.Button(root, text="OK", command=ok_hand).pack()
    tk.Button(root, text="Point", command=point_hand).pack()
    tk.Button(root, text="Yeah", command=yeah_hand).pack()

    root.mainloop()

# ================= 启动线程 =================
threading.Thread(target=sim_loop, daemon=True).start()
start_gui()
