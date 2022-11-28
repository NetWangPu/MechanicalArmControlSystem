#include<EEPROM.h>
/***
 * 
 * 存储数据
 * mega2560有4KB（4096bytes）的EEPROM，可以用来存储数据，但是只能存储字节，不能存储结构体，
 * 所以需要自己定义结构体，然后将结构体转换成字节，存储到EEPROM中，读取的时候再将字节转换成结构体。
 * 
 * 这个从EEPROM中读取是在开始的时候读取，写入到EEPROM中是在链表发生变化的时候写入（保存巡查点）
 * 
 * 
*/

//
#define EEPROM_SIZE 4096
#define EEPROM_START 0
#define EEPROM_END EEPROM_SIZE - 1
//计算出一个节点的大小
#define PatrolpointNodeSize sizeof(PatrolpointNode)

//写一个双向链表节点 记录每个巡查点的信息
typedef struct PatrolpointNode {
  int step;  //第几个巡查点
  int servo_G;   //距离起始点的距离 
  int servo_A;	 //A轴的角度
  int servo_B;	//B轴的角度
  int servo_H;	//H轴伸缩的长度
  int servo_E;	//E轴的角度
  int servo_F;  //F轴的角度
  struct PatrolpointNode *next;   //节点指针 next
  struct PatrolpointNode *prev;   //节点指针 prev 用于返向巡查
} PatrolpointNode;

void StructToByte(PatrolpointNode *node, byte *byteArray, int size)
{
  byte *p = (byte *)node;
  for (int i = 0; i < size; i++)
  {
    byteArray[i] = p[i];
  }
}

void ByteToStruct(byte *byteArray, PatrolpointNode *node, int size)
{
  byte *p = (byte *)node;
  for (int i = 0; i < size; i++)
  {
    p[i] = byteArray[i];
  }
}

void SaveStructToEEPROM(PatrolpointNode *node, int size, int address)
{
  byte *byteArray = new byte[size];
  StructToByte(node, byteArray, size);
  for (int i = 0; i < size; i++)
  {
    EEPROM.write(address + i, byteArray[i]);
  }
  delete[] byteArray;
}

void ReadStructFromEEPROM(PatrolpointNode *node, int size, int address)
{
  byte *byteArray = new byte[size];
  for (int i = 0; i < size; i++)
  {
    byteArray[i] = EEPROM.read(address + i);
  }
  ByteToStruct(byteArray, node, size);
  delete[] byteArray;
}

/// @brief 将链表中的数据存储到EEPROM中
/// @param head 链表的头指针
void SavePatrolpointNodeToEEPROM(PatrolpointNode *head)
{
  PatrolpointNode *p = head;
  int i = 0;
  while (p != NULL)
  {
    SaveStructToEEPROM(p, PatrolpointNodeSize, i * PatrolpointNodeSize);
    p = p->next;
    i++;
  }
}
 
/// @brief 将EEPROM中的数据读取出来加载到链表中
/// @param head 链表的头指针
void ReadPatrolpointNodeFromEEPROM(PatrolpointNode *head)
{
  PatrolpointNode *p = head;
  int i = 0;
  while (p != NULL)
  {
    ReadStructFromEEPROM(p, PatrolpointNodeSize, i * PatrolpointNodeSize);
    p = p->next;
    i++;
  }
}

/// @brief 初始化链表返回头节点
/// @return 头节点
PatrolpointNode *initPatrolpointList() {
  PatrolpointNode *head = (PatrolpointNode *)malloc(sizeof(PatrolpointNode));
  head->next = NULL;
  head->prev = NULL;
  return head;
}

/// @brief 将节点插入到链表中
/// @param head 头节点
/// @param step 第几个巡查点
/// @param servo_G 距离起始点的距离
/// @param servo_A A轴的角度
/// @param servo_B B轴的角度
/// @param servo_H H轴伸缩的长度
/// @param servo_E E轴的角度
/// @param servo_F F轴的角度
void insertPatrolpointList(PatrolpointNode *head, int step, int servo_G, int servo_A, int servo_B, int servo_H, int servo_E, int servo_F) {
  PatrolpointNode *p = head;
  while (p->next != NULL) {
    p = p->next;
  }
  PatrolpointNode *newNode = (PatrolpointNode *)malloc(sizeof(PatrolpointNode));
  newNode->step = step;
  newNode->servo_G = servo_G;
  newNode->servo_A = servo_A;
  newNode->servo_B = servo_B;
  newNode->servo_H = servo_H;
  newNode->servo_E = servo_E;
  newNode->servo_F = servo_F;
  newNode->next = NULL;
  newNode->prev = p;
  p->next = newNode;
}

/// @brief 打印链表 这个函数是用来测试的
/// @param head 头节点
void printPatrolpointList(PatrolpointNode *head) {
  PatrolpointNode *p = head->next;
  while (p != NULL) {
    Serial.print("step:");
    Serial.print(p->step);
    Serial.print(" servo_G:");
    Serial.print(p->servo_G);
    Serial.print(" servo_A:");
    Serial.print(p->servo_A);
    Serial.print(" servo_B:");
    Serial.print(p->servo_B);
    Serial.print(" servo_H:");
    Serial.print(p->servo_H);
    Serial.print(" servo_E:");
    Serial.print(p->servo_E);
    Serial.print(" servo_F:");
    Serial.println(p->servo_F);
    p = p->next;
  }
}