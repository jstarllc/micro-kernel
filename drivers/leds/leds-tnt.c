
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
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

#define USER_PATH "driver/tnt-leds"

static struct proc_dir_entry *g_tnt_led_entry;

static struct platform_device *g_pdev;

int g_gpio_led_sda = 0;
int g_gpio_led_scl = 0;
int g_gpio_led_rst = 0;

static int g_userCounter = 0;

/******************************************************************************
 * Function	: LED_SCL_HIGH
 * Note		:
 ******************************************************************************/
void LED_SCL_HIGH(void)
{
	LED_GPIO_HIGH(g_gpio_led_scl);
	led_i2c_delay(DELAY_CNT);
}
/******************************************************************************
 * Function	: LED_SCL_LOW
 * Note		:
 ******************************************************************************/
void LED_SCL_LOW(void)
{
	LED_GPIO_LOW(g_gpio_led_scl);
	led_i2c_delay(DELAY_CNT);
}
/******************************************************************************
 * Function	: LED_DAT_HIGH
 * Note		:
 ******************************************************************************/
void LED_DAT_HIGH(void)
{
	LED_GPIO_HIGH(g_gpio_led_sda);
	led_i2c_delay(DELAY_CNT);
}

/******************************************************************************
 * Function	: DAT_LOW
 * Note		:
 ******************************************************************************/
void LED_DAT_LOW(void)
{
	LED_GPIO_LOW(g_gpio_led_sda);
	led_i2c_delay(DELAY_CNT);
}

/******************************************************************************
 * Function	: LED_DAT_SET_IN
 * Note		:
 ******************************************************************************/
void LED_DAT_SET_IN(void)
{
	LED_GPIO_IN_MODE(g_gpio_led_sda);
	led_i2c_delay(DELAY_CNT);
}

/******************************************************************************
 * Function	: DAT_SET_IN
 * Note		:
 ******************************************************************************/
int LED_READ_DATA(void)
{
	int rData = gpio_get_value(g_gpio_led_sda) & 0x01;
	return rData;
}

// *****************************************************************************
// Function 	: tnt_led_driver_init
// Note			: 
// *****************************************************************************
void tnt_led_proc_init(void)
{
	static int first_one = 0;
	Led_pwm_t pwm;
	int i;
		
	led_i2c_init();

	/* REST High */
	gpio_direction_output(g_gpio_led_rst, 1);

	/* Mode Set */
	led_i2c_byte_write(LED_R_ADDR, 0x00, 0x00);
	led_i2c_byte_write(LED_G_ADDR, 0x00, 0x00);
	led_i2c_byte_write(LED_B_ADDR, 0x00, 0x00);

	/* LED Driver Output State Registers */
#if 0
	/* Default LED driver x is individual brightness can be controlled */
	led_i2c_byte_write(LED_R_ADDR, LED_OUT0, 0xAA);	
	led_i2c_byte_write(LED_R_ADDR, LED_OUT1, 0xAA);
	led_i2c_byte_write(LED_R_ADDR, LED_OUT2, 0xAA);
	led_i2c_byte_write(LED_R_ADDR, LED_OUT3, 0x2A);		/* led15 not used */

	led_i2c_byte_write(LED_G_ADDR, LED_OUT0, 0xAA);	
	led_i2c_byte_write(LED_G_ADDR, LED_OUT1, 0xAA);
	led_i2c_byte_write(LED_G_ADDR, LED_OUT2, 0xAA);
	led_i2c_byte_write(LED_G_ADDR, LED_OUT3, 0x2A);		/* led15 not used */

	led_i2c_byte_write(LED_B_ADDR, LED_OUT0, 0xAA);	
	led_i2c_byte_write(LED_B_ADDR, LED_OUT1, 0xAA);
	led_i2c_byte_write(LED_B_ADDR, LED_OUT2, 0xAA);
	led_i2c_byte_write(LED_B_ADDR, LED_OUT3, 0x2A);		/* led15 not used */
#else
	/* Default LED driver x is individual brightness and group dimming can be controlled */
	led_i2c_byte_write(LED_R_ADDR, LED_OUT0, 0xFF);	
	led_i2c_byte_write(LED_R_ADDR, LED_OUT1, 0xFF);
	led_i2c_byte_write(LED_R_ADDR, LED_OUT2, 0xFF);
	led_i2c_byte_write(LED_R_ADDR, LED_OUT3, 0x3F);		/* led15 not used */

	led_i2c_byte_write(LED_G_ADDR, LED_OUT0, 0xFF);	
	led_i2c_byte_write(LED_G_ADDR, LED_OUT1, 0xFF);
	led_i2c_byte_write(LED_G_ADDR, LED_OUT2, 0xFF);
	led_i2c_byte_write(LED_G_ADDR, LED_OUT3, 0x3F);		/* led15 not used */

	led_i2c_byte_write(LED_B_ADDR, LED_OUT0, 0xFF);	
	led_i2c_byte_write(LED_B_ADDR, LED_OUT1, 0xFF);
	led_i2c_byte_write(LED_B_ADDR, LED_OUT2, 0xFF);
	led_i2c_byte_write(LED_B_ADDR, LED_OUT3, 0x3F);		/* led15 not used */
#endif
	/* GRPPWM */
	led_i2c_byte_write(LED_R_ADDR, 0x12, 125);
	led_i2c_byte_write(LED_G_ADDR, 0x12, 125);
	led_i2c_byte_write(LED_B_ADDR, 0x12, 125);	
	
	/* GRPFREQ */
	led_i2c_byte_write(LED_R_ADDR, 0x13, 5);
	led_i2c_byte_write(LED_G_ADDR, 0x13, 5);
	led_i2c_byte_write(LED_B_ADDR, 0x13, 5);

#if 0
	/* BLINK FUNCTION ENABLE */
	led_i2c_byte_write(LED_R_ADDR, 0x01, 0x20);
	led_i2c_byte_write(LED_G_ADDR, 0x01, 0x20);
	led_i2c_byte_write(LED_B_ADDR, 0x01, 0x20);	
#else
	/* DIMMING FUNCTION ENABLE */
	led_i2c_byte_write(LED_R_ADDR, 0x01, 0x00);
	led_i2c_byte_write(LED_G_ADDR, 0x01, 0x00);
	led_i2c_byte_write(LED_B_ADDR, 0x01, 0x00);	
#endif

	if(first_one == 0)
	{
		for(i = 0; i < MAX_CHANNEL; i++)
		{
			//EJM this is probs where to turn off the LEDs on boot
			
			pwm.pwm_r[i] = PWM_MAX/4;
			pwm.pwm_g[i] = PWM_MAX/4;
			pwm.pwm_b[i] = PWM_MAX/4;						
		}
		
		led_i2c_buff_write(LED_R_ADDR, 0xA2, MAX_CHANNEL, pwm.pwm_r);
		led_i2c_buff_write(LED_G_ADDR, 0xA2, MAX_CHANNEL, pwm.pwm_g);
		led_i2c_buff_write(LED_B_ADDR, 0xA2, MAX_CHANNEL, pwm.pwm_b);
		
		first_one = 1;
	}
}

// *****************************************************************************
// Function 	: tnt_led_proc_open
// Note			: 
// *****************************************************************************
static int tnt_led_proc_open(struct inode *inode, struct file *file)
{
	//pr_info("[%s:%d]\n\n", _FN_, _LN_);
	g_userCounter++;
	
	tnt_led_proc_init();

	return 0;
}

// *****************************************************************************
// Function 	: tnt_led_proc_open
// Note			: 
// *****************************************************************************
static int tnt_led_proc_release(struct inode *inode, struct file *file)
{
    //pr_info("[%s:%d]\n\n", _FN_, _LN_);
	g_userCounter--;

    return 0;
}

// *****************************************************************************
// Function 	: tnt_led_proc_open
// Note			: 
// *****************************************************************************
static ssize_t tnt_led_proc_read(struct file *file, char __user * buf, size_t lbuf, loff_t * ppos)
{
    int nbytes = 0;
    //pr_info("[%s:%d]\n\n", _FN_, _LN_);
	//nbytes = bytes_to_do - copy_to_user(buf, ramdisk + *ppos, bytes_to_do);

	return nbytes;
}

// *****************************************************************************
// Function 	: tnt_led_proc_open
// Note			: 
// *****************************************************************************
static ssize_t tnt_led_proc_write(struct file *file, const char __user * buf, size_t lbuf, loff_t * ppos)
{
    int nbytes = 0;
    //pr_info("[%s:%d]\n\n", _FN_, _LN_);
	//nbytes = bytes_to_do - copy_from_user(ramdisk + *ppos, buf, bytes_to_do);

    return nbytes;
}

static ssize_t tnt_led_proc__ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{    	
	Led_reg_t reg;
	Led_blink_t blink;
	Led_pwm_t pwm;
	Led_set_one_t set_one;
	Led_select_one_t select_one;

    //printk( "ioctl\n" );

	switch(cmd)
	{
		case CMD_SET_FULL:
		{
			//pr_info("[%s:%d] CMD_SET_FULL\n\n", _FN_, _LN_);

			if (copy_from_user(&pwm, (void *)arg, sizeof(pwm)))
			{
				return -EFAULT;
			}

			/* Auto-increment Mode */			
			/* Auto-increment Flag : 1 */
			/* Auto-increment Options AI1 : 0, AI2 : 1 */
			/* Start reg addr : 0x02 */

			led_i2c_buff_write(LED_R_ADDR, 0xA2, MAX_CHANNEL, pwm.pwm_r);
			led_i2c_buff_write(LED_G_ADDR, 0xA2, MAX_CHANNEL, pwm.pwm_g);
			led_i2c_buff_write(LED_B_ADDR, 0xA2, MAX_CHANNEL, pwm.pwm_b);
		}
		break;

		case CMD_SET_ONE:
		{
			//pr_info("[%s:%d] CMD_SET_ONE\n\n", _FN_, _LN_);

			if (copy_from_user(&set_one, (void *)arg, sizeof(set_one)))
			{
				return -EFAULT;
			}

			led_color = set_one.led_color;
			led_set_one(set_one.num);
		}
		break;

		case CMD_SET_SELECT:
		{
			//pr_info("[%s:%d] CMD_SET_SELECT\n\n", _FN_, _LN_);

			if (copy_from_user(&select_one, (void *)arg, sizeof(select_one)))
			{
				return -EFAULT;
			}

			led_set_select(select_one.led_color, select_one.num, select_one.pwm);
		}
		break;

		case CMD_SET_BLINK:
		{
			//pr_info("[%s:%d] CCMD_SET_BLINK)\n\n", _FN_, _LN_);

			if (copy_from_user(&blink, (void *)arg, sizeof(blink)))
			{
				return -EFAULT;
			}

			if(blink.led_color == LED_COLOR_R)
			{
				reg.addr = LED_R_ADDR;
			}
			else if(blink.led_color == LED_COLOR_G)
			{
				reg.addr = LED_G_ADDR;
			}
			else if(blink.led_color == LED_COLOR_B)
			{
				reg.addr = LED_B_ADDR;
			}

			reg.reg = 0x14 + (blink.num / 4);

			reg.data = (uint8_t)led_i2c_byte_read(reg.addr, reg.reg);

			if(blink.enable != 0)
			{
				reg.data |= (0x03 << ((blink.num % 4) * 2));
			}
			else
			{
				reg.data &= ~(0x03 << ((blink.num % 4) * 2));
				reg.data |= 0x02 << ((blink.num % 4) * 2);
			}

			led_i2c_byte_write(reg.addr, reg.reg, reg.data);

			//pr_info("[%s:%d] CMD_SET_BLINK\n\n", _FN_, _LN_);

			//printk("addr : 0x%02x, reg : 0x%02x, data : 0x%02x\n", reg.addr, reg.reg, reg.data);			
		}
		break;

		case CMD_REG_WRITE:
		{
			//pr_info("[%s:%d] CMD_REG_WRITE\n\n", _FN_, _LN_);

			if (copy_from_user(&reg, (void *)arg, sizeof(reg)))
			{
				return -EFAULT;
			}

			led_i2c_byte_write(reg.addr, reg.reg, reg.data);
		}
		break;

		case CMD_REG_READ:
		{
			//pr_info("[%s:%d] CMD_REG_READ\n\n", _FN_, _LN_);
			
			if (copy_from_user(&reg, (void *)arg, sizeof(reg)))
			{
				return -EFAULT;
			}

			reg.data = (uint8_t)led_i2c_byte_read(reg.addr, reg.reg);

			if (copy_to_user((void *)arg, &reg, sizeof(reg)))
			{
				return -EFAULT;
			}
		}
		break;		
	}
	
    return 0;
} 
// *****************************************************************************
// struct 	: tnt_led_proc_fops
// Note			: 
// *****************************************************************************
static const struct file_operations tnt_led_proc_fops = {
    .owner = THIS_MODULE,
	.read = tnt_led_proc_read,
    .write = tnt_led_proc_write,
    .open = tnt_led_proc_open,
    .release = tnt_led_proc_release,
//	.ioctl = tnt_led_proc__ioctl,
	.unlocked_ioctl = tnt_led_proc__ioctl,
	.compat_ioctl = tnt_led_proc__ioctl,

};

// *****************************************************************************
// Function 	: tnt_led_proc_open
// Note			: 
// *****************************************************************************
static int tnt_led_probe(struct platform_device *pdev)
{
    int ret = -1;
	enum of_gpio_flags flag;
		
	struct device_node *led_node = pdev->dev.of_node;

    pr_info("[%s:%d]\n\n", _FN_, _LN_);
	
    g_pdev = pdev;

	g_gpio_led_sda = of_get_named_gpio_flags(led_node,"led-sda", 0, &flag);

	printk(KERN_INFO "[%s:%d] g_gpio_led_sda, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_led_sda, flag);
	
	if (!gpio_is_valid(g_gpio_led_sda))
	{
		printk(KERN_INFO "[%s:%d] of_get_named_gpio_flags, gpio : %d\n", __FUNCTION__, __LINE__, g_gpio_led_sda);
		return -1;
	} 
	
    ret = gpio_request(g_gpio_led_sda, "led-sda");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] gpio_request, gpio : %d\n", __FUNCTION__, __LINE__, g_gpio_led_sda);
		gpio_free(g_gpio_led_sda);
		ret = -EIO;
		goto EXIT_FAIL;
	}
	gpio_direction_output(g_gpio_led_sda, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));

	g_gpio_led_scl = of_get_named_gpio_flags(led_node,"led-scl", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_led_scl, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_led_scl, flag);

	if (!gpio_is_valid(g_gpio_led_scl)){
		printk(KERN_INFO "[%s:%d] of_get_named_gpio_flags, g_gpio_led_scl : %d\n", __FUNCTION__, __LINE__, g_gpio_led_scl);
		return -1;
	} 

    ret = gpio_request(g_gpio_led_scl, "led-scl");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] gpio_request, g_gpio_led_scl : %d\n", __FUNCTION__, __LINE__, g_gpio_led_scl);
		gpio_free(g_gpio_led_scl);
		ret = -EIO;
		goto EXIT_FAIL;
	}
	gpio_direction_output(g_gpio_led_scl, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));

	g_gpio_led_rst = of_get_named_gpio_flags(led_node,"led-rst", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_led_rst, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_led_rst, flag);

	if (!gpio_is_valid(g_gpio_led_rst)){
		printk(KERN_INFO "[%s:%d] of_get_named_gpio_flags, g_gpio_led_rst : %d\n", __FUNCTION__, __LINE__, g_gpio_led_rst);
		return -1;
	} 

    ret = gpio_request(g_gpio_led_rst, "led-rst");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] gpio_request, g_gpio_led_rst : %d\n", __FUNCTION__, __LINE__, g_gpio_led_rst);
		gpio_free(g_gpio_led_rst);
		ret = -EIO;
		goto EXIT_FAIL;
	}

	gpio_direction_output(g_gpio_led_rst, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));

	// Create a test entry under USER_ROOT_DIR
	g_tnt_led_entry = proc_create(USER_PATH, 0666, NULL, &tnt_led_proc_fops);
    if (NULL == g_tnt_led_entry)   
    {   
        goto EXIT_FAIL;   
    }	
	
    pr_info("[%s:%d] Success...!!!!!\n\n", _FN_, _LN_);

	tnt_led_proc_init();

	return 0;  //return Ok

EXIT_FAIL:
	return ret;
}

// *****************************************************************************
// Function 	: tnt_led_remove
// Note			: 
// *****************************************************************************
static int tnt_led_remove(struct platform_device *pdev)
{ 
    pr_info("[%s:%d]\n\n", _FN_, _LN_);
    return 0;
}

// *****************************************************************************
// struct 	: 
// Note		: 
// *****************************************************************************
#ifdef CONFIG_OF
static const struct of_device_id of_rk_tnt_led_match[] = {
	{ .compatible = "tnt,led" },
	{ /* Sentinel */ }
};
#endif
static struct platform_driver tnt_led_driver = {
	.probe		= tnt_led_probe,
	.remove		= tnt_led_remove,
	.driver		= {
	.name	= "tnt-led",
	.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_rk_tnt_led_match,
#endif
	},
};

// *****************************************************************************
// Function 	: tnt_led_init
// Note			: 
// *****************************************************************************
static int __init tnt_led_init(void)
{
    pr_info("[%s:%d]\n\n", _FN_, _LN_);

    return platform_driver_register(&tnt_led_driver);
}

static void __exit tnt_led_exit(void)
{
    pr_info("[%s:%d]\n\n", _FN_, _LN_);
	platform_driver_unregister(&tnt_led_driver);
}

subsys_initcall(tnt_led_init);
module_exit(tnt_led_exit);

MODULE_AUTHOR("ygyu <ygyu@thountech.com>");
MODULE_DESCRIPTION("TNT Input driver");
MODULE_LICENSE("GPL");

