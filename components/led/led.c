#include "driver/gpio.h"
#include "led.h"

//控制红灯
void led_red(int on)
{
    if(on==LED_ON)
    {
       gpio_set_level(LED_RED_IO, LED_ON);//开灯
    }
    else
    {
        gpio_set_level(LED_RED_IO, LED_OFF);//关灯
    }
}

//控制绿灯
void led_green(int on)
{
    if(on==LED_ON)
    {
       gpio_set_level(LED_GREEN_IO, LED_ON);//开灯
    }
    else
    {
        gpio_set_level(LED_GREEN_IO, LED_OFF);//关灯
    }
}

//控制蓝灯
void led_blue(int on)
{
    if(on==LED_ON)
    {
       gpio_set_level(LED_BLUE_IO, LED_ON);//开灯
    }
    else
    {
        gpio_set_level(LED_BLUE_IO, LED_OFF);//关灯
    }
}


//LED初始化
void initLed()
{
    //初始化彩灯的IO口，
    //红灯：GPIO32，绿灯：GPIO15，蓝灯：GPIO16
    gpio_pad_select_gpio(LED_RED_IO);
    gpio_pad_select_gpio(LED_GREEN_IO);
    gpio_pad_select_gpio(LED_BLUE_IO);
    gpio_set_direction(LED_RED_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE_IO, GPIO_MODE_OUTPUT);    
    
    //关红绿蓝三色灯
    led_red(LED_OFF);
    led_green(LED_OFF);
    led_blue(LED_OFF);
}

