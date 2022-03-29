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

//�궨��һ��GPIO
#define KEY_GPIO    0

static xQueueHandle gpio_evt_queue = NULL;

int led_index=0;

//ÿ����һ�Σ�����һ����ɫ
//���--->�̵�--->����--->ȫ��--->ȫ������ѭ��
void led_flash()
{
    if(led_index==0)
    {
        //��һ�ΰ��º����
        led_red(LED_ON);
        led_green(LED_OFF);
        led_blue(LED_OFF);
        led_index=1;
    }
    else if(led_index==1)
    {
        //�ڶ��ΰ����̵���
        led_red(LED_OFF);
        led_green(LED_ON);
        led_blue(LED_OFF);
        led_index=2;
    }
    else if(led_index==2)
    {
        //�����ΰ���������
        led_red(LED_OFF);
        led_green(LED_OFF);
        led_blue(LED_ON);
        led_index=3;
    }
    else if(led_index==3)
    {
        //���Ĵΰ��º�������ȫ��
        led_red(LED_ON);
        led_green(LED_ON);
        led_blue(LED_ON);
        led_index=4;
    }
    else// if(led_index==3)
    {
        //���Ĵΰ��º�������ȫ��
        led_red(LED_OFF);
        led_green(LED_OFF);
        led_blue(LED_OFF);
        led_index=0;
    }

}

//IO���жϻص�����
void IRAM_ATTR gpio_isr_handler(void *arg) {
	uint32_t gpio_num = (uint32_t) arg;
    //����һ���жϵ�������
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//�����жϳ�ʼ��
void KeyInit() {
	//����GPIO���½��غ������ش����ж�
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;//����Ϊ�½��غ������ش����ж�
	io_conf.pin_bit_mask = 1 << KEY_GPIO;//�����жϵ�IO��
	io_conf.mode = GPIO_MODE_INPUT;//����IOΪ���빦��
	io_conf.pull_up_en = 1;//��λʹ��
	gpio_config(&io_conf);//����IO��

	gpio_set_intr_type(KEY_GPIO, GPIO_INTR_ANYEDGE);//����Ϊ�½��غ������ش����ж�
	gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));//����һ��2����С�Ķ���

	gpio_install_isr_service(0);
	gpio_isr_handler_add(KEY_GPIO, gpio_isr_handler, (void *) KEY_GPIO);//ע���жϻص�����
}

//����ɨ�躯��
//���س������Ͷ̰���
esp_err_t key_scan() {
	uint32_t io_num;
	BaseType_t press_key = pdFALSE;
	BaseType_t lift_key = pdFALSE;
	int backup_time = 0;

	while (1) {

		//���մ���Ϣ���з�������Ϣ
		xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);

		//��¼���û����°�����ʱ���
		if (gpio_get_level(io_num) == 0) {
			press_key = pdTRUE;
			backup_time = system_get_time();
			//�����ǰGPIO�ڵĵ�ƽ�Ѿ���¼Ϊ���£���ʼ��ȥ�ϴΰ��°�����ʱ���
		} else if (press_key) {
			//��¼̧��ʱ���
			lift_key = pdTRUE;
			backup_time = system_get_time() - backup_time;
		}

		//�������±�־λ�Ͱ��������־λ��Ϊ1ʱ�򣬲�ִ�лص�
		if (press_key & lift_key) {
			press_key = pdFALSE;
			lift_key = pdFALSE;

			//�������1s��ص�����������Ͷ̰��ص�
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
    initLed();//��ʼ��LED
	KeyInit();//��ʼ�������ж�

	while (1) {
		ret = key_scan();//�ȴ��������£�
		if (ret == -1)
			vTaskDelete(NULL);

		switch (ret) {
		case KEY_SHORT_PRESS:
			printf("�̰������ص� ... \r\n");
            led_flash();//�ı�LED
			break;

		case KEY_LONG_PRESS:
			printf("���������ص� ... \r\n");
			break;

		default:
			break;
		}
	}

	vTaskDelete(NULL);
}


void app_main() {
    //����һ���������ڼ�ⰴ��
	xTaskCreate(key_trigger, "key_trigger", 1024 * 2, NULL, 10,	NULL);
}

