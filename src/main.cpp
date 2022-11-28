#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <servo.h>
#include <controller.h>
#include <pointnode.h>

//#define SYS_DEBUG 1  /*调试是用，控制舵机运动与读取按键值*/

// PS2手柄触发值MAX 562
// PS2手柄触发值MIN 462
#define PS2_MAX 562
#define PS2_MIN 462

// 信号知识等引脚 自动模式灯为 11 示教器模式灯为 12
#define AUTO_LED 11
#define TEACH_LED 12

//保存巡查点按键 这个没必要使用中断
#define SAVE_KEY 13

///@todo 模式切换感觉没有必要使用中断
//中断引脚模式切换和暂停/继续
#define MODE_KEY 8
#define PAUSE_KEY 9

int patrolpointNum = 0; // counter 巡查点的个数 用于计算巡查点的编号
//初始化 巡查点链表
PatrolpointNode *patrolpointList = initPatrolpointList();
//记录起始的巡查点
PatrolpointNode *startPatrolpoint = patrolpointList;

/************示教器舵机相关定义*************/
#define SERVOF 2
#define SERVOE 1
#define SERVOB 5
#define SERVOA 6
int servo_G_max = 180;
int servo_G_min = 0;
int servo_G_init = 90;
int servo_A_max = 900;
int servo_A_min = 160;
int servo_A_init = 530;
int servo_B_max = 430;
int servo_B_min = 0;
int servo_B_init = 400;
int servo_H_max = 180;
int servo_H_min = 0;
int servo_H_init = 90;
int servo_E_max = 1000;
int servo_E_min = 0;
int servo_E_init = 700;
int servo_F_max = 1000;
int servo_F_min = 250;
int servo_F_init = 600;

// LCD对象
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup()
{

  //按键初始化
  pinMode(SAVE_KEY, INPUT_PULLUP);
  //中断初始化
  pinMode(MODE_KEY, INPUT_PULLUP);
  pinMode(PAUSE_KEY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_KEY), modeChange, FALLING); //模式切换 按键中断 低电平触发 中断函数modeKey
  // attachInterrupt(digitalPinToInterrupt(PAUSE_KEY), pauseContinue, FALLING); //暂停/继续 按键中断 低电平触发 中断函数pauseKey
  //初始化icc通信 amega2560的icc通信引脚为 20 21 scl引脚为22 sda引脚为23
  Wire.begin();
  /************串口初始化*****************/
  Serial.begin(115200);  //初始化串口 用于电脑调试
  Serial1.begin(115200); //初始化串口 用于模拟舵机控制 串口1 pin 18 19 18为TX 19为RX
  Serial2.begin(115200); //初始化串口 用于和机械臂通信 串口2 pin 16 17 16为TX 17为RX
  // Serial3.begin(115200); //初始化串口 用于和机械臂通信 串口3 pin 14 15 这个用不到

  //如果定义了调试宏 SYS_DEBUG 直接跳过初始化
#ifdef SYS_DEBUG
  Serial.println("SYS_DEBUG is defined");
  Serial.println("开始调试");
#else
  //信号灯 默认为示教器模式
  pinMode(AUTO_LED, OUTPUT);
  pinMode(TEACH_LED, OUTPUT);

  /************屏幕初始化*****************/
  lcd.init();      //初始化LCD
  lcd.backlight(); //打开背光
  lcd.clear();     //清屏
  lcd.setCursor(0, 0);
  lcd.print("Start init...");

  //舵机归回到初始位置并解锁舵机
  LobotSerialServoMove(Serial1, SERVOA, servo_A_init, 2000);
  LobotSerialServoMove(Serial1, SERVOB, servo_B_init, 2000);
  LobotSerialServoMove(Serial1, SERVOE, servo_E_init, 2000);
  LobotSerialServoMove(Serial1, SERVOF, servo_F_init, 2000);
  delay(2000);
  LobotSerialServoUnload(Serial1, SERVOA);
  LobotSerialServoUnload(Serial1, SERVOB);
  LobotSerialServoUnload(Serial1, SERVOE);
  LobotSerialServoUnload(Serial1, SERVOF);

  /************WIFI初始化*****************/
  ///@todo wifi初始化
  ///@todo 等待wifi连接成功 继续执行
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTING");
  //等待wifi连接成功 等待串口收到的字符串含有“WIFICONNECTED”
  while (!Serial.available())
  {
    delay(100);
  }
  while (Serial.available())
  {
    String str = Serial.readString();
    if (str.indexOf("WIFICONNECTED") != -1)
    {
      delay(1000);
      break;
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTED");
  delay(2000);
#endif
}

/// @brief 根据接收的序号和指定的舵机ID执行相应的动作
/// @param pointNum  接收到的序号
/// @return  1 为执行成功 0 为执行失败
int doActionByPonintNum(int pointNum)
{
  //根据序号找到对应的动作 pintNum从1开始 0为无效动作
  if (pointNum == 0)
  {
    return 0;
  }
  //在链表中找到第pointNum个动作
  PatrolpointNode *p = patrolpointList;
  for (int i = 1; i < pointNum; i++)
  {
    p = p->next;
    //如果找到最后一个动作还没有找到 则返回0
    if (p == NULL)
    {
      return 0;
    }
  }
  //执行动作 串口1为舵机控制 串口2为机械臂控制
  Serial2.print("M2A" + p->servo_A + 'B' + p->servo_B + 'E' + p->servo_E + 'F' + p->servo_F + 'G' + p->servo_G + 'H' + p->servo_H + 'X');
  //舵机上锁
  LobotSerialServoLoad(Serial1, SERVOA);
  LobotSerialServoLoad(Serial1, SERVOB);
  LobotSerialServoLoad(Serial1, SERVOE);
  LobotSerialServoLoad(Serial1, SERVOF);
  LobotSerialServoMove(Serial1, SERVOA, p->servo_A, 2000);
  LobotSerialServoMove(Serial1, SERVOB, p->servo_B, 2000);
  LobotSerialServoMove(Serial1, SERVOE, p->servo_E, 2000);
  LobotSerialServoMove(Serial1, SERVOF, p->servo_F, 2000);
  delay(2000);
  //舵机解锁
  LobotSerialServoUnload(Serial1, SERVOA);
  LobotSerialServoUnload(Serial1, SERVOB);
  LobotSerialServoUnload(Serial1, SERVOE);
  LobotSerialServoUnload(Serial1, SERVOF);
  return 1;
}

/***
 * E轴 A0
 * F轴 A1
 * H轴 A2
 * A轴 A3
 * B轴 A4
 * G轴 A5
 */

//检测操作者是否发生了操作 对舵机和摇杆 1为发生了操作 0为没有发生操作
int checkOperator()
{
  int operatorFlag = 0;
  //检测摇杆是否发生了操作A0 A1 A2 A5
  if (analogRead(A0) > PS2_MAX || analogRead(A0) < PS2_MIN || analogRead(A1) > PS2_MAX || analogRead(A1) < PS2_MIN)
  {
    servo_E_init = Gearspeedregulation(analogRead(A0), servo_E_init, servo_E_max, servo_E_min, 2);
    servo_F_init = Gearspeedregulation(analogRead(A1), servo_F_init, servo_F_max, servo_F_min, 2);
    //开始移动舵机 首先上锁舵机 这是对示教器的操作
    LobotSerialServoLoad(Serial1, SERVOE);
    LobotSerialServoLoad(Serial1, SERVOF);
    //移动舵机
    LobotSerialServoMove(Serial1, SERVOE, servo_E_init, 100);
    LobotSerialServoMove(Serial1, SERVOF, servo_F_init, 100);
    delay(100);
    //解锁舵机
    LobotSerialServoUnload(Serial1, SERVOE);
    LobotSerialServoUnload(Serial1, SERVOF);
    operatorFlag = 1;
  }
  if (analogRead(A2) > PS2_MAX || analogRead(A2) < PS2_MIN || analogRead(A3) > PS2_MAX || analogRead(A3) < PS2_MIN)
  {
    servo_H_init = Gearspeedregulation(analogRead(A2), servo_H_init, servo_H_max, servo_H_min, 2);
    servo_A_init = Gearspeedregulation(analogRead(A3), servo_A_init, servo_A_max, servo_A_min, 2);
    //开始移动舵机 首先上锁舵机 这是对示教器的操作
    LobotSerialServoLoad(Serial1, SERVOA);
    //移动舵机
    LobotSerialServoMove(Serial1, SERVOA, servo_A_init, 100);
    delay(100);
    //解锁舵机
    LobotSerialServoUnload(Serial1, SERVOA);
    operatorFlag = 1;
  }
  if (analogRead(A4) > PS2_MAX || analogRead(A4) < PS2_MIN || analogRead(A5) > PS2_MAX || analogRead(A5) < PS2_MIN)
  {
    servo_B_init = Gearspeedregulation(analogRead(A4), servo_B_init, servo_B_max, servo_B_min, 2);
    servo_G_init = Gearspeedregulation(analogRead(A5), servo_G_init, servo_G_max, servo_G_min, 2);
    //开始移动舵机 首先上锁舵机 这是对示教器的操作
    LobotSerialServoLoad(Serial1, SERVOB);
    //移动舵机
    LobotSerialServoMove(Serial1, SERVOB, servo_B_init, 100);
    delay(100);
    //解锁舵机
    LobotSerialServoUnload(Serial1, SERVOB);
    operatorFlag = 1;
  }
  //读取舵机的角度判断是否发生了操作
  if (LobotSerialServoReadPosition(Serial1, SERVOA) != servo_A_init)
  {
    servo_A_init = LobotSerialServoReadPosition(Serial1, SERVOA);
    operatorFlag = 1;
  }
  if (LobotSerialServoReadPosition(Serial1, SERVOB) != servo_B_init)
  {
    servo_B_init = LobotSerialServoReadPosition(Serial1, SERVOB);
    operatorFlag = 1;
  }
  if (LobotSerialServoReadPosition(Serial1, SERVOE) != servo_E_init)
  {
    servo_E_init = LobotSerialServoReadPosition(Serial1, SERVOE);
    operatorFlag = 1;
  }
  if (LobotSerialServoReadPosition(Serial1, SERVOF) != servo_F_init)
  {
    servo_F_init = LobotSerialServoReadPosition(Serial1, SERVOF);
    operatorFlag = 1;
  }
  return operatorFlag;
}

//示教器模式
void TeachMode()
{
  //初始化示教器舵机
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TeachMode");
  digitalWrite(AUTO_LED, LOW);
  digitalWrite(TEACH_LED, HIGH);
  while (1)
  {
    if (checkOperator() == 1)
    {
      //发送数据给机械臂 发送的数据格式为M1A500B500E500F500G500H1000X 机械臂为串口2
      ///@todo 这个你到时候再测试一下
      Serial2.print("M1A" + servo_A_init + 'B' + servo_B_init + 'E' + servo_E_init + 'F' + servo_F_init + 'G' + servo_G_init + 'H' + servo_H_init + 'X');
    }
    //检查是否按下保存键
    if (SAVE_KEY == LOW)
    {
      delay(100);
      //将数据插入到链表中
      insertPatrolpointList(patrolpointList, ++patrolpointNum, servo_G_init, servo_A_init, servo_B_init, servo_H_init, servo_E_init, servo_F_init);
    }
  }
}

//巡查模式
void AutoPatrolMode()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AutoPatrolMode");
  digitalWrite(AUTO_LED, HIGH);
  digitalWrite(TEACH_LED, LOW);
  // 执行巡逻
  //找到最后一个点
  PatrolpointNode *p = patrolpointList;
  while (p->next != NULL)
  {
    p = p->next;
  }
  while (p->prev != NULL)
  {
    //中间有人操作就停止巡逻
    if (checkOperator() == 1)
    {
      break;
    }
    //如果点击了暂停按钮 则暂停巡逻
    if (digitalRead(PAUSE_KEY) == LOW)
    {
      delay(100);
      // lcd 清屏
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Pause");
      while (1)
      {
        //给机械臂发送停止指令
        Serial2.print("M3X");
        //如果点击了继续按钮 则继续巡逻
        if (digitalRead(PAUSE_KEY) == LOW)
        {
          //消除按键抖动
          delay(100);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("AutoPatrolMode");
          Serial2.print("M3X");
          lcd.setCursor(2, 0);
          lcd.print("Point" + p->step);
          break;
        }
      }
    }
    //发送数据给机械臂 发送的数据格式为M1A500B500E500F500G500H1000X 机械臂为串口2
    //控制机械臂移动到指定位置
    Serial2.print("M2A" + p->servo_A + 'B' + p->servo_B + 'E' + p->servo_E + 'F' + p->servo_F + 'G' + p->servo_G + 'H' + p->servo_H + 'X');
    //控制示教器机械臂运动
    LobotSerialServoLoad(Serial1, SERVOA);
    LobotSerialServoLoad(Serial1, SERVOB);
    LobotSerialServoLoad(Serial1, SERVOE);
    LobotSerialServoLoad(Serial1, SERVOF);
    LobotSerialServoMove(Serial1, SERVOE, p->servo_E, 2000);
    LobotSerialServoMove(Serial1, SERVOF, p->servo_F, 2000);
    LobotSerialServoMove(Serial1, SERVOA, p->servo_A, 2000);
    LobotSerialServoMove(Serial1, SERVOB, p->servo_B, 2000);
    delay(2000);
    LobotSerialServoUnload(Serial1, SERVOA);
    LobotSerialServoUnload(Serial1, SERVOB);
    LobotSerialServoUnload(Serial1, SERVOE);
    LobotSerialServoUnload(Serial1, SERVOF);
    //清空lcd屏幕 显示巡查点信息
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("AutoPatrolMode");
    lcd.setCursor(2, 0);
    lcd.print("Point" + p->step);
    Serial.println("执行巡逻" + String(p->step));
    //等待机械臂到达指定位置 等待串口里面有数据 且数据为OK 因为中间也有可能有人操作 所以要检查是否有人操作
    while (Serial2.available() == 0)
    {
      //中间有人操作就停止巡逻
      if (checkOperator() == 1)
      {
        break;
      }
      //如果点击了暂停按钮 则暂停巡逻
      if (digitalRead(PAUSE_KEY) == LOW)
      {
        delay(100);
        // lcd 清屏
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Pause");
        while (1)
        {
          //给机械臂发送停止指令
          Serial2.print("M3X");
          //如果点击了继续按钮 则继续巡逻
          if (digitalRead(PAUSE_KEY) == LOW)
          {
            //消除按键抖动
            delay(100);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("AutoPatrolMode");
            Serial2.print("M3X");
            lcd.setCursor(2, 0);
            lcd.print("Point" + p->step);
            break;
          }
        }
      }
      delay(10);
    }
    while (Serial2.available() > 0)
    {
      if (Serial2.read() == 'O')
      {
        if (Serial2.read() == 'K')
        {
          break;
        }
      }
    }
    delay(1000);
    p = p->prev;
    ///@todo 这个好像出现俩溢出 哥我这没开发板了 你把serial的注释取消试试
  }
}

void modeChange()
{
  //这个函数用于切换模式 对于当前状态的判断 直接读取AUTO_LED和TEACH_LED的状态
  //如果AUTO_LED为高电平，TEACH_LED为低电平，说明当前为自动模式
  //如果AUTO_LED为低电平，TEACH_LED为高电平，说明当前为示教器模式
  if (AUTO_LED == HIGH)
  {
    //如果当前为自动模式，切换到示教器模式
    TeachMode();
  }
  else if (TEACH_LED == HIGH)
  {
    //如果当前为示教器模式，切换到自动模式
    AutoPatrolMode();
  }
}

void pauseContinue()
{
}

void loop()
{
#ifdef SYS_DEBUG
  //读取串口的数据
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 'C')
    {
      String str = Serial.readStringUntil('E');
      //将字符串转换为整数
      int temp = str.toInt();
      LobotSerialServoMove(Serial1, SERVOB, temp, 100);
    }
  }

  // //读取A0 A1口的模拟值
  int analogValueA0 = analogRead(A0);
  int analogValueA1 = analogRead(A1);
  int analogValueA2 = analogRead(A2);
  int analogValueA3 = analogRead(A3);
  int analogValueA4 = analogRead(A4);
  int analogValueA5 = analogRead(A5);
  Serial.print("A0:");
  Serial.print(analogValueA0);
  Serial.print("A1:");
  Serial.print(analogValueA1);
  Serial.print("A2:");
  Serial.print(analogValueA2);
  Serial.print("A3:");
  Serial.print(analogValueA3);
  Serial.print("A4:");
  Serial.print(analogValueA4);
  Serial.print("A5:");
  Serial.println(analogValueA5);
#else
  /**
   * 模式选择
   * 1.巡查模式 可打断快速进入示教模式 AutoPatrolMode
   * 2.示教模式  TeachMode
   * 默认为示教器模式 通过按键切换 按键按下后切换模式 同时lcd显示当前模式
   * 屏幕显示
   * 1.显示当前模式
   * 2.显示当前巡查点
   */
  TeachMode();
  AutoPatrolMode();
#endif
}