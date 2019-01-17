
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

#define MYDEV_NAME "tnt_leds"

// *****************************************************************************
// Include
// *****************************************************************************


// *****************************************************************************
// Define
// *****************************************************************************


// *****************************************************************************
// Function
// *****************************************************************************
static void led_write_data(void);

// *****************************************************************************
// Macro
// *****************************************************************************


// *****************************************************************************
// Variable
// *****************************************************************************
//uint8_t pwm_r[MAX_CHANNEL];
//uint8_t pwm_g[MAX_CHANNEL];
//uint8_t pwm_b[MAX_CHANNEL];
int pwm_r[MAX_CHANNEL];
int pwm_g[MAX_CHANNEL];
int pwm_b[MAX_CHANNEL];

uint16_t led_color = LED_COLOR_R;

/******************************************************************************
 * Function		: 
 * Parameters	: none
 * Return		: none
 * Description	:
 ******************************************************************************/
void led_write_color_full(uint16_t r, uint16_t g, uint16_t b)
{
	uint16_t c_index;

	// Blue
	for(c_index = 0; c_index < MAX_CHANNEL; c_index++)
	{
		led_i2c_byte_write(LED_B_ADDR, (c_index + 2), b);
	}

	// Green
	for(c_index = 0; c_index < MAX_CHANNEL; c_index++)
	{
		led_i2c_byte_write(LED_G_ADDR, (c_index + 2), g);
	}

	// Red
	for(c_index = 0; c_index < MAX_CHANNEL; c_index++)
	{
		led_i2c_byte_write(LED_R_ADDR, (c_index + 2), r);
	}

	for(c_index = 0x14; c_index <= 0x17; c_index++)
	{
		led_i2c_byte_write(LED_B_ADDR, c_index, 0xAA);
		led_i2c_byte_write(LED_G_ADDR, c_index, 0xAA);
		led_i2c_byte_write(LED_R_ADDR, c_index, 0xAA);
	}
}

// *****************************************************************************
// Function 	: led_set_one
// Input		: None
// Output		: None
// Note			: 
// *****************************************************************************
void led_set_one(uint16_t number)
{
	if(number >= (MAX_CHANNEL - 1))
	{
		number = (MAX_CHANNEL - 1);
	}
	
	memset(pwm_b, 0x00, sizeof(pwm_b));
	memset(pwm_g, 0x00, sizeof(pwm_g));
	memset(pwm_r, 0x00, sizeof(pwm_r));
	
	switch(led_color)
	{
		case LED_COLOR_B:
			pwm_b[number] = PWM_MAX;
			break;

		case LED_COLOR_G:
			pwm_g[number] = PWM_MAX;
			break;

		case LED_COLOR_R:
			pwm_r[number] = PWM_MAX;
			break;
	}

	led_write_data();
}

// *****************************************************************************
// Function 	: led_set_select
// Input		: None
// Output		: None
// Note			: 
// *****************************************************************************
void led_set_select(uint16_t color, uint16_t number, uint16_t on)
{
	if(number >= (MAX_CHANNEL - 1))
	{
		number = (MAX_CHANNEL - 1);
	}
	
	switch(color)
	{
		case LED_COLOR_B:
			pwm_b[number] = on;
			break;

		case LED_COLOR_G:
			pwm_g[number] = on;
			break;

		case LED_COLOR_R:
			pwm_r[number] = on;
			break;
	}

	led_write_data();
}

// *****************************************************************************
// Function 	: led_write_data
// Input		: None
// Output		: None
// Note			: 
// *****************************************************************************
static void led_write_data(void)
{
//	uint16_t c_index;

#if 0
	// Blue
	for(c_index = 0; c_index < MAX_CHANNEL; c_index++)
	{
		led_i2c_byte_write(LED_B_ADDR, (c_index + 2), pwm_b[c_index]);
	}

	// Green
	for(c_index = 0; c_index < MAX_CHANNEL; c_index++)
	{
		led_i2c_byte_write(LED_G_ADDR, (c_index + 2), pwm_g[c_index]);
	}

	// Red
	for(c_index = 0; c_index < MAX_CHANNEL; c_index++)
	{
		led_i2c_byte_write(LED_R_ADDR, (c_index + 2), pwm_r[c_index]);
	}
#else	//srcho
	led_i2c_buff_write(LED_B_ADDR, 0xA2, MAX_CHANNEL, pwm_b);
	led_i2c_buff_write(LED_G_ADDR, 0xA2, MAX_CHANNEL, pwm_g);
	led_i2c_buff_write(LED_R_ADDR, 0xA2, MAX_CHANNEL, pwm_r);
#endif

#if 0	//srcho del
	for(c_index = 0x14; c_index <= 0x17; c_index++)
	{
		led_i2c_byte_write(LED_B_ADDR, c_index, 0xAA);
		led_i2c_byte_write(LED_G_ADDR, c_index, 0xAA);
		led_i2c_byte_write(LED_R_ADDR, c_index, 0xAA);
	}
#endif	
}

