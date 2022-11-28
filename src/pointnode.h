/***
 * 
 * 存储数据
 * 
 * 
*/

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

/// @brief 打印链表
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