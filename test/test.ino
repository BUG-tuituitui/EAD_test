#include <AlfredoCRSF.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
// ELRS无线电接收机，占用UART2（Arduino中的Serial2）
#define ELRS_TX 9
#define ELRS_RX 8
// 舵机引脚定义
#define LEFT1_SERVO 11
#define LEFT2_SERVO 10
#define RIGHT1_SERVO 1
#define RIGHT2_SERVO 5
//#define RUDDER_PIN 9
//#define ELEVATOR_PIN 10

// 板载灯的GPIO定义
// #define LED_BUILTIN 8 // 解锁灯，启动则亮
// #define LED_BUILTIN_B 11 // 调试用灯
#ifdef RGB_BRIGHTNESS
#undef RGB_BRIGHTNESS
#endif
#define RGB_BRIGHTNESS 10  // Change white brightness (max 255)

#ifdef RGB_BUILTIN
#undef RGB_BUILTIN
#endif
#define RGB_BUILTIN 21

// 推进器推力控制GPIO
// #define THRUSTER_PIN 13
#define THRUSTER_L 7
#define THRUSTER_R 6

// 姿态传感器I2C接口
#define GYRO_SCL 13
#define GYRO_SDA 12

// 遥控器绑定，这里使用美国手（左手油）
#define MIN_JOYSTICK 990   // 摇杆最小值
#define MAX_JOYSTICK 2100  // 摇杆最大值
#define DELTA_RADIO 100    // 接收机允许误差量
#define RADIO_HIGH 2000    // 按钮高位
#define RADIO_MIDDLE 1500  // 按钮中位
#define RADIO_LOW 1000     // 按钮低位
#define AIL 1              // 滚转，右手横向
#define ELE 2              // 俯仰，右手纵向
#define THR 3              // 油门，左手纵向
#define RUD 4              // 偏航，左手横向
#define SA 5               // 两状态开关，取值为1000和2000，这里用作解锁开关，按下时才允许进一步操作
// 下面几个是遥控器的其他按钮，保留备用
#define SB 6  // 三状态开关，取值为1000，1500，2000
#define SC 7  // 三状态开关，取值为1000，1500，2000
#define SD 8  // 两状态开关，取值为1000和2000
#define SE 9  // 单触发按钮，按下后从1000跳变到2000，否则为1000，适合当快门用，不过暂时没摄像头
// 舵机脉宽定义
#define MAX_WIDTH 2500
#define MIN_WIDTH 500
// 油门输出值
#define ANALOG_MAX 255
#define ANALOG_MIN 150
// 注释掉下面这行关闭调试输出
#define DEBUG

// 接收机变量
HardwareSerial crsfSerial(1);  // 接收机串口
AlfredoCRSF crsf;              // 接收机收到的信息0
// 舵机变量
Servo left;
Servo right;
// 飞机工作状态，目前为0锁定，为1解锁
bool status = false;
// 调用该函数之后关闭所有输出
void lock();

// 偏航差动系数
// 0 < yaw_delta < 1
// 单推进器的推力 = yaw_delta * 由油门给出的推力
double yaw_delta_left = 1.0;
double yaw_delta_right = 1.0;

void setup() {
  // put your setup code here, to run once:
  // 推进器初始化
  //analogWriteFrequency(500);
  analogWriteResolution(8);
  analogWrite(THRUSTER_L, 0);
  analogWrite(THRUSTER_R, 0);
  //Serial.begin(115200);                 // 调试串口
  // 配置状态灯
  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Off / red
  // 配置接收机
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, ELRS_RX, ELRS_TX);
  if (!crsfSerial)
    while (1)
      delay(10);
  crsf.begin(crsfSerial);
  // 配置舵机
  // 分配硬件定时器
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  // 设置频率
  left.setPeriodHertz(50);
  right.setPeriodHertz(50);
  // 关联舵机对象与GPIO引脚，设置脉宽范围
  left.attach(LEFT1_SERVO, MIN_WIDTH, MAX_WIDTH);
  right.attach(RIGHT1_SERVO, MIN_WIDTH, MAX_WIDTH);
  // #ifdef DEBUG
  //     // 备用调试灯
  //     pinMode(LED_BUILTIN_B, OUTPUT);
  // #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  // 每轮循环与接收机通信一次
  crsf.update();
  int value;                // 接受接收机读到的通道值
  double normalized_value;  // 归一化通道值
  // 没解锁的话盯着SA通道直到解锁
  while (!status) {
    value = crsf.getChannel(SA);
    if (value > RADIO_HIGH - DELTA_RADIO && value < RADIO_HIGH + DELTA_RADIO) {    // 高位
      neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue   
      status = true;
    }
    crsf.update();
  }
  // 解锁后循环扫描9个遥控器通道
  for (unsigned int channel = 1; channel <= 9; ++channel) {
    // 取得当前通道数值
    value = crsf.getChannel(channel);
    normalized_value = 1.0 * (value - MIN_JOYSTICK) / (MAX_JOYSTICK - MIN_JOYSTICK);
    switch (channel) {
      case AIL:
        // 滚转代码
        continue;
        break;
      case ELE:
        left.write((int)(180 * normalized_value));
        right.write((int)(180 * normalized_value));
        break;
      case THR:
        {
          int thrusterValueL = (int)(yaw_delta_left * ((255 - 150) * normalized_value + 159));
          int thrusterValueR = (int)(yaw_delta_right * ((255 - 150) * normalized_value + 159));
          analogWrite(THRUSTER_L, thrusterValueL);
          analogWrite(THRUSTER_R, thrusterValueR);
#ifdef DEBUG
          // 测试：串口向上位机输出当前油门输出值
          //Serial.println(thrusterValue);
          // 测试：当油门过半时板载灯B亮起
          // if (value > (MAX_JOYSTICK + MIN_JOYSTICK) / 2) {
          //     digitalWrite(THRUSTER_PIN, HIGH);
          //    // digitalWrite(LED_BUILTIN_B, HIGH);
          // } else {
          //     digitalWrite(THRUSTER_PIN, LOW);
          //    // digitalWrite(LED_BUILTIN_B, LOW);
          // }
#endif
        }
        break;
      case RUD:
        // 偏航代码
        if (normalized_value <= 0.5) {
          yaw_delta_left = 2 * normalized_value;
          yaw_delta_right = 1.0;
        } else if (normalized_value > 0.5) {
          yaw_delta_left = 1.0;
          yaw_delta_right = 2 * (1 - normalized_value);
        }
        break;
      case SA:
        // 按下后解锁，再按锁定
        if (value > RADIO_HIGH - DELTA_RADIO && value < RADIO_HIGH + DELTA_RADIO) {    // 高位
          neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
          status = true;
        } else {                                             // 只要不在高位就锁，安全起见
          neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Off / red
          lock();
          status = false;
        }
        break;
      default:
        break;
    }
  }
#ifdef DEBUG
  // 循环输出各通道数值
  // for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++) {
  //   Serial.print(crsf.getChannel(ChannelNum));
  //   Serial.print(", ");
  // }
  // Serial.println(" ");
#endif
}

void lock() {
  // #ifdef DEBUG
  //     digitalWrite(LED_BUILTIN_B, LOW);
  // #endif
  analogWrite(THRUSTER_L, 0);
  analogWrite(THRUSTER_R, 0);
  return;
}