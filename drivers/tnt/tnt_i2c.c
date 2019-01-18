
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
#include <linux/jiffies.h>


#include "tnt_io.h"


// *****************************************************************************
// Define
// *****************************************************************************

// *****************************************************************************
// Variable
// *****************************************************************************


/******************************************************************************
 * Function	: i2c_delay
 * Note		:
 ******************************************************************************/
void i2c_delay(unsigned long delay)
{
	volatile unsigned long count = delay;

	while(count--)
	{
	}
}


// *****************************************************************************
// Function 	: i2c_start
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_start(void)
{
	DAT_HIGH();
	SCL_HIGH();
	
	DAT_LOW();
	SCL_LOW();
}

// *****************************************************************************
// Function 	: i2c_stop
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_stop(void)
{
	DAT_LOW();
	SCL_LOW();
	
	SCL_HIGH();
	DAT_HIGH();
}

// *****************************************************************************
// Function 	: i2c_acks
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_acks(void)
{
	int wait = 10;
	
	DAT_HIGH();
	DAT_SET_IN();

	SCL_HIGH();

	while(wait--)
	{
		if(IO_READ_DATA() == Bit_RESET)
		{
			break;
		}
	}

	if(wait == 0xFF)
	{
		pr_info("[%s:%d] I2C NAK\n", _FN_, _LN_);
	}

	SCL_LOW();
}

// *****************************************************************************
// Function 	: i2c_ackm
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_ackm(void)
{
	DAT_LOW();

	SCL_HIGH();
	SCL_LOW();
}

// *****************************************************************************
// Function 	: i2c_nacks
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_nacks(void)
{
	int wait = 10;
	
	DAT_HIGH();
	DAT_SET_IN();

	SCL_HIGH();

	while(wait--)
	{
		if(IO_READ_DATA() == Bit_SET)
		{
			break;
		}
	}

	if(wait == 0xFF)
	{
		pr_info("[%s:%d] I2C NAK\n", _FN_, _LN_);
	}

	SCL_LOW();
}

// *****************************************************************************
// Function 	: i2c_nackm
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_nackm(void)
{
	DAT_HIGH();

	SCL_HIGH();
	SCL_LOW();
}

// *****************************************************************************
// Function 	: i2c_send_data
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_send_data(int Data)
{
	int index = 8;
		
	SCL_LOW();
		
	while(index--)
	{
		if((Data & 0x80) == 0)
		{
			DAT_LOW();
		}
		else
		{
			DAT_HIGH();
		}
		
		SCL_HIGH();
		SCL_LOW();

		Data <<= 1;
	}
}

// *****************************************************************************
// Function 	: i2c_read_data
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
int i2c_read_data(void)
{
	int index = 8;
	int data = 0;

	DAT_SET_IN();
	SCL_LOW();	  	
	
	while(index--)
	{
		SCL_HIGH();

		i2c_delay(10);	
	
		data <<= 1;
		
		if(IO_READ_DATA() == Bit_SET)
		{
			data |= Bit_SET;
		}

		SCL_LOW();
	}
	return data;
}

// *****************************************************************************
// Function 	: i2c_byte_read
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
int i2c_byte_read(int devAddr, int regAddr)
{
	int data;

	i2c_start();
	
	i2c_send_data(devAddr);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();
	
	i2c_stop();
	
	i2c_start();
	
	i2c_send_data(devAddr + I2C_READ);
	i2c_acks();
	
	data = i2c_read_data();
	i2c_acks();
	
	i2c_stop();

	return data;
}

// *****************************************************************************
// Function 	: i2c_byte_write
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_byte_write(int devAddr, int regAddr, int data)
{
	i2c_start();
	
	i2c_send_data(devAddr);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();
	
	i2c_send_data(data);
	i2c_acks();
	
	i2c_stop();
}

// *****************************************************************************
// Function 	: i2c_word_write
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_word_write(int devAddr, int regAddr, uint16_t data)
{
	i2c_start();
	
	i2c_send_data(devAddr);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();
	
	i2c_send_data((data >> 8) & 0xFF);
	i2c_acks();
	
	i2c_send_data((data >> 0) & 0xFF);
	i2c_acks();
	
	i2c_stop();
}

// *****************************************************************************
// Function 	: i2c_word_read
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
uint16_t i2c_word_read(int devAddr, int regAddr)
{
	uint16_t data;

	i2c_start();
	
	i2c_send_data(devAddr);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();

	i2c_send_data(devAddr + I2C_READ);
	i2c_acks();

	data = (i2c_read_data() << 8);
	i2c_ackm();
	
	data += (i2c_read_data() << 0);
	i2c_nackm();
	
	i2c_stop();

	return data;
}

// *****************************************************************************
// Function 	: i2c_buff_read
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_buff_read(int devAddr, int regAddr, int count, int *buffer)
{
	i2c_start();
	
	i2c_send_data(devAddr);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();
	
	i2c_stop();
	
	i2c_start();
	
	i2c_send_data(devAddr + I2C_READ);
	i2c_acks();

	while(count)
	{
		*buffer++ = i2c_read_data();
		
		if(count == 1)
		{
			i2c_nackm();
		}
		else
		{
			i2c_ackm();
		}

		count--;
	}
	
	i2c_stop();
}

// *****************************************************************************
// Function 	: i2c_buff_write
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_buff_write(int devAddr, int regAddr, int count, int *buffer)
{
	i2c_start();
	
	i2c_send_data(devAddr);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();
	
	while(count)
	{
		i2c_send_data(*buffer++);

		if(count == 1)
		{
			i2c_nackm();
		}
		else
		{
			i2c_ackm();
		}

		count--;
	}
	
	i2c_stop();
}


// *****************************************************************************
// Function 	: i2c_write_tmp
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_write_tmp(uint8_t regAddr, uint16_t cmd)
{

	g_delayVal = 2;
	
	i2c_word_write(HDC1080_I2C_ADDR, regAddr, cmd);

}

// *****************************************************************************
// Function 	: i2c_read_temp
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
uint16_t i2c_read_temp(int regAddr)
{
	uint16_t data;

	g_delayVal = 200;
	
	i2c_start();
	
	i2c_send_data(HDC1080_I2C_ADDR);
	i2c_acks();
	
	i2c_send_data(regAddr);
	i2c_acks();

	msleep_interruptible(50);
//	i2c_delay(1000);


	i2c_send_data(HDC1080_I2C_ADDR + I2C_READ);
	i2c_acks();

	data = (i2c_read_data() << 8);
	i2c_ackm();
	
	data += (i2c_read_data() << 0);
	i2c_nackm();
	
	i2c_stop();

	return data;
}

// *****************************************************************************
// Function 	: i2c_read_touch
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
uint16_t i2c_read_touch(void)
{
	uint16_t r_h = 0, r_l = 0;
	uint16_t data = 0;

//	printk("[%s:%d] =================================\r\n", _FN_, _LN_);
	

	g_delayVal = 400;
	
	i2c_start();
	
	i2c_send_data(TUOCH_I2C_ADDR | I2C_READ);
	i2c_acks();

	i2c_delay(1000);
	
	r_h = i2c_read_data();
	i2c_ackm();
	
	r_l = i2c_read_data();
	i2c_nackm();
	
	i2c_stop();


	data = (r_l << 8) | (r_h & 0xFF);



//	printk("[%s:%d] Read : 0x%04X (0x%x, 0x%x)\r\n", _FN_, _LN_, data, r_h, r_l);
//	printk("[%s:%d] =================================\r\n", _FN_, _LN_);


	 
//	i2c_delay(10);

	return data;
}

// *****************************************************************************
// Function 	: i2c_write_light
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
void i2c_write_light(uint8_t regAddr, uint8_t cmd)
{
	g_delayVal = 200;
	
	i2c_start();
	
	i2c_send_data(LIGHT_I2C_ADDR);
	i2c_ackm();

	
	i2c_send_data(regAddr);
	i2c_ackm();


	i2c_send_data(cmd);
	i2c_ackm();
	
	i2c_stop();
}

// *****************************************************************************
// Function 	: i2c_read_light
// Input		: 
// Output		: 
// Note			: 
// *****************************************************************************
uint16_t i2c_read_light(uint8_t regAddr)
{
	uint16_t data;

	g_delayVal = 200;

	i2c_start();
	
	i2c_send_data(LIGHT_I2C_ADDR);
	SCL_HIGH();
	SCL_LOW();

	i2c_delay(500);

	i2c_send_data(regAddr);
	SCL_HIGH();
	SCL_LOW();

	DAT_HIGH();
	SCL_HIGH();
	
	DAT_LOW();
	SCL_LOW();

	i2c_send_data(LIGHT_I2C_ADDR | I2C_READ);
	SCL_HIGH();
	SCL_LOW();

	i2c_delay(1000);

	data = i2c_read_data();

	DAT_HIGH();
	SCL_HIGH();
	SCL_LOW();

	
	i2c_stop();
	i2c_delay(10);

	return data;
}


/******************************************************************************
 * Function		: i2c_init
 * Parameters	: none
 * Return		: none
 * Description	:
 ******************************************************************************/
void i2c_init(void)
{
	uint16_t data[6];

	g_delayVal = 0;

	DAT_HIGH();
	SCL_HIGH();


	i2c_read_touch();
	i2c_delay(10);	
	
	i2c_write_light(0x80, 0x01);
	i2c_write_light(0x85, 0x12);
	i2c_delay(10);	
	
	i2c_write_tmp(HDC_CONFIGURATION, 0x8000);
	i2c_delay(10);	
	i2c_write_tmp(HDC_CONFIGURATION, 0x1000);
	i2c_delay(10);	

	data[0] = i2c_read_temp(HDC_MANUFACTURER_ID);
	data[1] = i2c_read_temp(HDC_DEVICE_ID);

	data[2] = i2c_read_temp(HDC_SERIAL_ID1);
	data[3] = i2c_read_temp(HDC_SERIAL_ID2);
	data[4] = i2c_read_temp(HDC_SERIAL_ID3);

	i2c_delay(100);	

	data[5] = i2c_read_temp(HDC_CONFIGURATION);
	
	/* Read Chip ID */
	printk("[%s] RID : 0x%04X, 0x%04X, 0x%04X, 0x%04X, 0x%04X\r\n", _FN_, data[0], data[1], data[2], data[3], data[4]);
	printk("[%s] CFG : 0x%04X\r\n", _FN_, data[5]);
	
	pr_info("[%s:%d] Complete\n", _FN_, _LN_);

}




