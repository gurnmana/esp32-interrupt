#ifndef _LED_H_
#define _LED_H_

//定义LED灯的IO口
#define LED_RED_IO 		32  //对应红灯的LED，绿灯为15，蓝灯为16
#define LED_GREEN_IO 	15  //对应红灯的LED，绿灯为15，蓝灯为16
#define LED_BLUE_IO 	16  //对应红灯的LED，绿灯为15，蓝灯为16

//定义LED状态
#define LED_ON          0   //LED灯亮电平为低电平
#define LED_OFF         1   //LED灯灭电平为高电平

//控制红灯
void led_red(int on);
//控制绿灯
void led_green(int on);
//控制蓝灯
void led_blue(int on);
//LED初始化
void initLed();

#endif