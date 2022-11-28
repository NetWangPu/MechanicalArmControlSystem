/***
 * 
 * 
 * 手柄力度值
 * 
 * 
*/



/// @brief 读取手柄力度值并控制舵机角度 2档位 并且能设置角度阈值
/// @param sensorValue 手柄力度值
/// @param angle      舵机角度
/// @param servo_max  舵机最大角度值
/// @param gear    单次增加的最小角度值
/// @return      舵机角度
int Gearspeedregulation(int sensorValue,float angle, float servo_max ,float servo_min, float gear){
  if(sensorValue > 562 ){
    if(sensorValue > 812){
      angle = angle + 2 * gear;
    }else{
      angle = angle + gear;
    }
  }else if(sensorValue < 462){
    if(sensorValue < 212){
      angle = angle - 2 * gear;
    }else{
      angle = angle - gear;
    }
  }else{
    angle = angle;
  }
  if(angle > servo_max){
    angle = servo_max;
  }else if(angle < servo_min){
    angle = servo_min;
  }
  return angle;
}