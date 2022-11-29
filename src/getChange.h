/***
 * 
 * 
 * 用来获取机械臂的信息的
 * 1.获取电量
 * 
 * 
*/

///@使用串口通讯给机械臂发送获取电量的指令给机械臂 解析返回的电量信息 Serial2
int getPower()
{
    //发送获取电量的指令
    Serial2.print("GETPOWER")
    //等待机械臂返回电量信息
    while(Serial2.available() == 0)
    {
        delay(10);
    }
    String power = Serial2.readStringUntil('E'); //读取到E之前的字符串
    power = power.substring(1); //去掉P
    int powerInt = power.toInt();
    return powerValue;
}