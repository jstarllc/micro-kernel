
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <linux/version.h>   
#include <linux/proc_fs.h>   
#include <linux/fb.h>
#include <linux/rk_fb.h>
#include <linux/display-sys.h>
#include <linux/of.h>

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/fcntl.h>

#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/irq.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include "leds-tnt.h"


// *****************************************************************************
// Define
// *****************************************************************************

// *****************************************************************************
// Variable
// *****************************************************************************


/******************************************************************************
 * Function	: led_i2c_delay
 * Note		:
 ******************************************************************************/
void led_i2c_delay(uint32_t delay)
{
	volatile uint32_t count = delay;

	while(count--)
	{
	}
}

// *****************************************************************************
// Function 	: led_i2c_start
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_start(void)
{
	LED_DAT_HIGH();
	LED_SCL_HIGH();
	
	LED_DAT_LOW();
	LED_SCL_LOW();
}

// *****************************************************************************
// Function 	: led_i2c_stop
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_stop(void)
{
	LED_DAT_LOW();
	LED_SCL_LOW();
	
	LED_SCL_HIGH();
	LED_DAT_HIGH();
}

// *****************************************************************************
// Function 	: led_i2c_acks
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_acks(void)
{
	int wait = 10;
	
	LED_DAT_HIGH();
	LED_DAT_SET_IN();

	LED_SCL_HIGH();

	while(wait--)
	{
		if(LED_READ_DATA() == Bit_RESET)
		{
			break;
		}
	}

	if(wait == 0xFF)
	{
		pr_info("[%s:%d] I2C NAK\n", _FN_, _LN_);
	}

	LED_SCL_LOW();

	LED_GPIO_OUT_MODE(g_gpio_led_sda, 1);	//srcho added	
}

// *****************************************************************************
// Function 	: led_i2c_ackm
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_ackm(void)
{
	LED_DAT_LOW();

	LED_SCL_HIGH();
	LED_SCL_LOW();
}

// *****************************************************************************
// Function 	: led_i2c_nacks
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_nacks(void)
{
	int wait = 10;
	
	LED_DAT_HIGH();
	LED_DAT_SET_IN();

	LED_SCL_HIGH();

	while(wait--)
	{
		if(LED_READ_DATA() == Bit_SET)
		{
			break;
		}
	}

	if(wait == 0xFF)
	{
		pr_info("[%s:%d] I2C NAK\n", _FN_, _LN_);
	}

	LED_SCL_LOW();
	
	LED_GPIO_OUT_MODE(g_gpio_led_sda, 1);	//srcho added
}

// *****************************************************************************
// Function 	: led_i2c_nackm
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_nackm(void)
{
	LED_DAT_HIGH();

	LED_SCL_HIGH();
	LED_SCL_LOW();
}

// *****************************************************************************
// Function 	: led_i2c_send_data
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_send_data(int Data)
{
	int index = 8;
		
	LED_SCL_LOW();
		
	while(index--)
	{
		if((Data & 0x80) == 0)
		{
			LED_DAT_LOW();
		}
		else
		{
			LED_DAT_HIGH();
		}
		
		LED_SCL_HIGH();
		LED_SCL_LOW();

		Data <<= 1;
	}
}

// *****************************************************************************
// Function 	: led_i2c_read_data
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
int led_i2c_read_data(void)
{
	int index = 8;
	int data = 0;

	LED_DAT_SET_IN();
	LED_SCL_LOW();	  	
	
	while(index--)
	{
		data <<= 1;
		
		if(LED_READ_DATA() == Bit_SET)
		{
			data |= Bit_SET;
		}
		
		LED_SCL_HIGH();
		LED_SCL_LOW();
	}


	LED_GPIO_OUT_MODE(g_gpio_led_sda, 1);	//srcho added
	
	return data;
}

// *****************************************************************************
// Function 	: led_i2c_byte_read
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
int led_i2c_byte_read(int devAddr, int regAddr)
{
	int data;

	led_i2c_start();
	
	led_i2c_send_data(devAddr);
	led_i2c_acks();
	
	led_i2c_send_data(regAddr);
	led_i2c_acks();
	
	led_i2c_stop();
	
	led_i2c_start();
	
	led_i2c_send_data(devAddr + I2C_READ);
	led_i2c_acks();
	
	data = led_i2c_read_data();
	led_i2c_acks();
	
	led_i2c_stop();

	return data;
}

// *****************************************************************************
// Function 	: led_i2c_byte_write
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_byte_write(int devAddr, int regAddr, int data)
{
	led_i2c_start();
	
	led_i2c_send_data(devAddr);
	led_i2c_acks();
	
	led_i2c_send_data(regAddr);
	led_i2c_acks();
	
	led_i2c_send_data(data);
	led_i2c_acks();
	
	led_i2c_stop();
}

// *****************************************************************************
// Function 	: led_i2c_word_write
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_word_write(int devAddr, int regAddr, uint16_t data)
{
	led_i2c_start();
	
	led_i2c_send_data(devAddr);
	led_i2c_acks();
	
	led_i2c_send_data(regAddr);
	led_i2c_acks();
	
	led_i2c_send_data((data >> 8) & 0xFF);
	led_i2c_acks();
	
	led_i2c_send_data((data >> 0) & 0xFF);
	led_i2c_acks();
	
	led_i2c_stop();
}

// *****************************************************************************
// Function 	: led_i2c_word_read
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
uint16_t led_i2c_word_read(int devAddr, int regAddr)
{
	uint16_t data;

	led_i2c_start();
	
	led_i2c_send_data(devAddr);
	led_i2c_acks();
	
	led_i2c_send_data(regAddr);
	led_i2c_acks();

	led_i2c_send_data(devAddr + I2C_READ);
	led_i2c_acks();

	data = (led_i2c_read_data() << 8);
	led_i2c_ackm();
	
	data += (led_i2c_read_data() << 0);
	led_i2c_nackm();
	
	led_i2c_stop();

	return data;
}

// *****************************************************************************
// Function 	: led_i2c_buff_read
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_buff_read(int devAddr, int regAddr, int count, int *buffer)
{
	led_i2c_start();
	
	led_i2c_send_data(devAddr);
	led_i2c_acks();
	
	led_i2c_send_data(regAddr);
	led_i2c_acks();
	
	led_i2c_stop();
	
	led_i2c_start();
	
	led_i2c_send_data(devAddr + I2C_READ);
	led_i2c_acks();

	while(count)
	{
		*buffer++ = led_i2c_read_data();
		
		if(count == 1)
		{
			led_i2c_nackm();
		}
		else
		{
			led_i2c_ackm();
		}

		count--;
	}
	
	led_i2c_stop();
}

// *****************************************************************************
// Function 	: led_i2c_buff_write
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void led_i2c_buff_write(int devAddr, int regAddr, int count, int *buffer)
{
	led_i2c_start();
	
	led_i2c_send_data(devAddr);
	led_i2c_acks();
	
	led_i2c_send_data(regAddr);
	led_i2c_acks();
	
	while(count)
	{
		led_i2c_send_data(*buffer++);

		if(count == 1)
		{
			led_i2c_nackm();
		}
		else
		{
			led_i2c_ackm();
		}

		count--;
	}
	
	led_i2c_stop();
}

/******************************************************************************
 * Function		: led_i2c_init
 * Parameters	: none
 * Return		: none
 * Description	:
 ******************************************************************************/
void led_i2c_init(void)
{
#if 0	//org

	//LED_DAT_HIGH();
	//LED_SCL_HIGH();

#else	//srcho edited

	LED_GPIO_OUT_MODE(g_gpio_led_sda, 1);
	LED_GPIO_OUT_MODE(g_gpio_led_scl, 1);

#endif
	
	//pr_info("[%s:%d] Complete\n", _FN_, _LN_);
}

