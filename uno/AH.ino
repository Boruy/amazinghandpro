#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 舵机 PWM 范围（按实际舵机调节）
#define SERVOMIN 150
#define SERVOMAX 600

// 平滑移动参数
const int stepSize = 5; // 每次 loop 移动的步长
int MiddlePos[8] = {
  SERVOMIN, SERVOMAX,  // 小拇指
  SERVOMIN, SERVOMAX,  // 食指
  SERVOMIN, SERVOMAX,  // 中指
  SERVOMIN, SERVOMAX   // 拇指
};
int targetPos[8]; // 目标位置

// 平滑滤波参数
const int numReadings = 5;
int thumbReadings[numReadings], indexReadings[numReadings], middleReadings[numReadings], pinkyReadings[numReadings];
int readIndex = 0;
int thumbTotal = 0, indexTotal = 0, middleTotal = 0, pinkyTotal = 0;
int thumbAverage = 0, indexAverage = 0, middleAverage = 0, pinkyAverage = 0;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  pwm.begin();
  pwm.setPWMFreq(60);

  // 初始化舵机位置
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(i, 0, MiddlePos[i]);
    targetPos[i] = MiddlePos[i];
  }

  // 初始化滤波数组
  for (int i = 0; i < numReadings; i++) {
    thumbReadings[i] = indexReadings[i] = middleReadings[i] = pinkyReadings[i] = 0;
  }

  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    handleSerialData();
  }

  // 非阻塞平滑移动
  for (int i = 0; i < 8; i++) {
    if (MiddlePos[i] < targetPos[i]) {
      MiddlePos[i] += stepSize;
      if (MiddlePos[i] > targetPos[i]) MiddlePos[i] = targetPos[i];
      pwm.setPWM(i, 0, MiddlePos[i]);
    } else if (MiddlePos[i] > targetPos[i]) {
      MiddlePos[i] -= stepSize;
      if (MiddlePos[i] < targetPos[i]) MiddlePos[i] = targetPos[i];
      pwm.setPWM(i, 0, MiddlePos[i]);
    }
  }
}

// 处理串口数据
void handleSerialData() {
  String data = Serial.readStringUntil('\n');

  int thumbAngle, indexAngle, middleAngle, pinkyAngle;

  // 严格解析顺序：拇指, 食指, 中指, 小拇指
  int parsed = sscanf(data.c_str(), "%d,%d,%d,%d", &thumbAngle, &indexAngle, &middleAngle, &pinkyAngle);
  if (parsed == 4) {
    applyFilter(thumbAngle, indexAngle, middleAngle, pinkyAngle);

    // 映射到 0~180 度
    int thumbDeg  = constrain(map(thumbAverage, 0, 180, 0, 180), 0, 180);
    int indexDeg  = constrain(map(indexAverage, 0, 180, 0, 180), 0, 180);
    int middleDeg = constrain(map(middleAverage, 0, 180, 0, 180), 0, 180);
    int pinkyDeg  = constrain(map(pinkyAverage, 0, 180, 0, 180), 0, 180);


    // 再映射到舵机 PWM
    targetPos[6] = thumbEvenPos(map(thumbDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[7] = thumbOddPos(map(thumbDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[2] = indexEvenPos(map(indexDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[3] = indexOddPos(map(indexDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[4] = middleEvenPos(map(middleDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[5] = middleOddPos(map(middleDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[0] = pinkyEvenPos(map(pinkyDeg, 0, 180, SERVOMIN, SERVOMAX));
    targetPos[1] = pinkyOddPos(map(pinkyDeg, 0, 180, SERVOMIN, SERVOMAX));
  }
}

// ======== 舵机映射函数 ========
int pinkyEvenPos(int base) { return base; }
int pinkyOddPos(int base)  { return SERVOMAX - (base - SERVOMIN); }

int indexEvenPos(int base) { return base; }
int indexOddPos(int base)  { return SERVOMAX - (base - SERVOMIN); }

int middleEvenPos(int base) { return base; }
int middleOddPos(int base)  { return SERVOMAX - (base - SERVOMIN); }

int thumbEvenPos(int base) { return base; }
int thumbOddPos(int base)  { return SERVOMAX - (base - SERVOMIN); }

// 平滑滤波
void applyFilter(int thumb, int index, int middle, int pinky) {
  thumbTotal  -= thumbReadings[readIndex];
  indexTotal  -= indexReadings[readIndex];
  middleTotal -= middleReadings[readIndex];
  pinkyTotal  -= pinkyReadings[readIndex];

  thumbReadings[readIndex]  = thumb;
  indexReadings[readIndex]  = index;
  middleReadings[readIndex] = middle;
  pinkyReadings[readIndex]  = pinky;

  thumbTotal  += thumb;
  indexTotal  += index;
  middleTotal += middle;
  pinkyTotal  += pinky;

  thumbAverage  = thumbTotal  / numReadings;
  indexAverage  = indexTotal  / numReadings;
  middleAverage = middleTotal / numReadings;
  pinkyAverage  = pinkyTotal  / numReadings;

  readIndex = (readIndex + 1) % numReadings;
}
