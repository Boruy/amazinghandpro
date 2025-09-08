# Amazing Hand 项目

## 📖 简介
**Amazing Hand** 是一个开源的仿生机械手项目。  
它结合了 **3D 打印结构、Arduino 控制、电机驱动、动作捕捉** 等技术，能够实现：

- 手指独立弯曲/伸展  
- 手势识别驱动机械手  
- 在仿真环境中测试动作  
- 支持 **PC 控制** 和 **OrangePi 独立运行**  

---

## 📂 文件目录说明

- `3d打印切片/`
  - `step/` → CAD 文件（STEP 格式）
  - `stl/` → 3D 打印文件（STL 格式）
  - `README.md` → 打印与组装说明
- `材料表/`  
  - `Amazing Hand.xlsx` → 材料清单（舵机、螺丝、电子元件等）
- `拼装说明.pdf` → 机械手装配教程
- `uno/`
  - `AH.ino` → Arduino 控制程序
  - `uno+pc/` → Python 串口控制程序
  - `uno+动作捕捉/` → MediaPipe 驱动机械手程序
- `orangepi/说明.txt` → 在 OrangePi 上运行说明

---

## 🛠️ 硬件需求

- **控制器**  
  - Arduino UNO（主控）  
  - OrangePi（可选，独立运行）  
- **执行器**  
  - SG90/MG90S 舵机 × 多个（驱动手指关节）  
  - PCA9685 舵机驱动板（推荐，支持多舵机控制）  
- **电源**  
  - 5V/2A 以上电源模块  
- **外设**  
  - 摄像头（用于动作捕捉控制）  
  - 3D 打印机（PLA/ABS 材料）

👉 详细配件清单请查看：`材料表/Amazing Hand.xlsx`

---

## 🔧 软件需求

- Arduino IDE  
  - 上传 `uno/AH.ino` 至 Arduino UNO  
- Python 3.8+  
- 依赖库安装：
  ```bash
  pip install -r requirements.txt
````

主要依赖：

* `pyserial` → 串口通信
* `opencv-python` → 图像处理
* `mediapipe` → 手势识别
* `numpy`
* `pybullet` → 仿真（可选）

---

## 🚀 使用方法

### 1. 机械手打印与装配

1. 打印 `3d打印切片/stl` 文件夹中的零件
2. 按照 `拼装说明.pdf` 步骤完成装配
3. 材料参考 `材料表/Amazing Hand.xlsx`

---

### 2. Arduino 控制

1. 使用 Arduino IDE 打开 `uno/AH.ino`
2. 上传至 Arduino UNO
3. 将舵机接入 UNO / PCA9685，连接电源

---

### 3. PC 串口控制

1. 进入目录：

   ```bash
   cd uno/uno+pc
   ```
2. 运行：

   ```bash
   python AHand.py
   ```
3. 输入舵机角度 → 机械手执行相应动作

---

### 4. 动作捕捉控制

1. 确保摄像头已连接
2. 进入目录：

   ```bash
   cd uno/uno+动作捕捉
   ```
3. 运行：

   ```bash
   python AH.py
   ```
4. 机械手会实时跟随你的手势动作

---

### 5. 仿真测试

1. 在 PyBullet / ROS 中加载模型：

   ```
   uno/uno+pc/support/hand.urdf
   ```
2. 在虚拟环境中测试关节动作
3. 验证后再映射到实体机械手

---

## 📐 功能模块

* 🖨️ **3D 打印机械结构** → STEP & STL 文件
* 🎮 **Arduino 控制** → 舵机驱动手指
* ✋ **动作捕捉** → MediaPipe + OpenCV
* 🖥️ **仿真模型** → URDF + STL

---

## 👥 项目贡献

* **硬件设计**：机械手 3D 打印零件
* **控制程序**：Arduino UNO + Python 串口通信
* **动作捕捉**：MediaPipe + OpenCV
* **仿真支持**：URDF 模型（支持 ROS / PyBullet）

---

