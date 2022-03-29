#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "led.h"

typedef enum {
	KEY_SHORT_PRESS = 1, KEY_LONG_PRESS,
} alink_key_t;

//宏定义一个GPIO
#define KEY_GPIO    0

static xQueueHandle gpio_evt_queue = NULL;

int led_index=0;

//每调用一次，亮下一种颜色
//红灯--->绿灯--->蓝灯--->全亮--->全灭，周期循环
void led_flash()
{
    if(led_index==0)
    {
        //第一次按下红灯亮
        led_red(LED_ON);
        led_green(LED_OFF);
        led_blue(LED_OFF);
        led_index=1;
    }
    else if(led_index==1)
    {
        //第二次按下绿灯亮
        led_red(LED_OFF);
        led_green(LED_ON);
        led_blue(LED_OFF);
        led_index=2;
    }
    else if(led_index==2)
    {
        //第三次按下蓝灯亮
        led_red(LED_OFF);
        led_green(LED_OFF);
        led_blue(LED_ON);
        led_index=3;
    }
    else if(led_index==3)
    {
        //第四次按下红绿蓝灯全亮
        led_red(LED_ON);
        led_green(LED_ON);
        led_blue(LED_ON);
        led_index=4;
    }
    else// if(led_index==3)
    {
        //第四次按下红绿蓝灯全灭
        led_red(LED_OFF);
        led_green(LED_OFF);
        led_blue(LED_OFF);
        led_index=0;
    }

}

//IO口中断回调函数
void IRAM_ATTR gpio_isr_handler(void *arg) {
	uint32_t gpio_num = (uint32_t) arg;
    //插入一个中断到队列中
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//按键中断初始化
void KeyInit() {
	//配置GPIO，下降沿和上升沿触发中断
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;//配置为下降沿和上升沿触发中断
	io_conf.pin_bit_mask = 1 << KEY_GPIO;//设置中断的IO口
	io_conf.mode = GPIO_MODE_INPUT;//设置IO为输入功能
	io_conf.pull_up_en = 1;//上位使能
	gpio_config(&io_conf);//配置IO口

	gpio_set_intr_type(KEY_GPIO, GPIO_INTR_ANYEDGE);//配置为下降沿和上升沿触发中断
	gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));//创建一个2个大小的队列

	gpio_install_isr_service(0);
	gpio_isr_handler_add(KEY_GPIO, gpio_isr_handler, (void *) KEY_GPIO);//注册中断回调函数
}

//按键扫描函数
//返回长按键和短按键
esp_err_t key_scan() {
	uint32_t io_num;
	BaseType_t press_key = pdFALSE;
	BaseType_t lift_key = pdFALSE;
	int backup_time = 0;

	while (1) {

		//接收从消息队列发来的消息
		xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);

		//记录下用户按下按键的时间点
		if (gpio_get_level(io_num) == 0) {
			press_key = pdTRUE;
			backup_time = system_get_time();
			//如果当前GPIO口的电平已经记录为按下，则开始减去上次按下按键的时间点
		} else if (press_key) {
			//记录抬升时间点
			lift_key = pdTRUE;
			backup_time = system_get_time() - backup_time;
		}

		//近当按下标志位和按键弹起标志位都为1时候，才执行回调
		if (press_key & lift_key) {
			press_key = pdFALSE;
			lift_key = pdFALSE;

			//如果大于1s则回调长按，否则就短按回调
			if (backup_time > 1000000) {
				return KEY_LONG_PRESS;
			} else {
				return KEY_SHORT_PRESS;
			}
		}
	}
}

void key_trigger(void *arg) {

	esp_err_t ret = 0;
    initLed();//初始化LED
	KeyInit();//初始化按键中断

	while (1) {
		ret = key_scan();//等待按键按下，
		if (ret == -1)
			vTaskDelete(NULL);

		switch (ret) {
		case KEY_SHORT_PRESS:
			printf("短按触发回调 ... \r\n");
            led_flash();//改变LED
			break;

		case KEY_LONG_PRESS:
			printf("长按触发回调 ... \r\n");
			break;

		default:
			break;
		}
	}

	vTaskDelete(NULL);
}


void app_main() {
    //创建一个任务，用于检测按键
	xTaskCreate(key_trigger, "key_trigger", 1024 * 2, NULL, 10,	NULL);
}

