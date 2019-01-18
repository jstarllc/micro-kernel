
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
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>

#include "tnt_io.h"

#define READ_JOB_NONE				0x00
#define READ_JOB_TOUCH				0x10
#define READ_JOB_LIGHT				0x11
#define READ_JOB_TEMPERATURE		0x12
#define READ_JOB_PIR				0x13
#define CTL_JOB_MOTOR				0x14

#define IOCTL_READ_LIGHT 			0x100
#define IOCTL_READ_TEMPERRATURE 	0x101
#define IOCTL_SET_MOTOR 			0x102
#define IOCTL_GPIO_67				0x113

#define IOCTL_SET_MOTOR_ON 			0x104
#define IOCTL_SET_MOTOR_OFF			0x105


#ifndef SLEEP_MILLI_SEC
#define SLEEP_MILLI_SEC(nMilliSec)\
do { \
long timeout = (nMilliSec) * HZ / 1000; \
while(timeout > 0) \
{ \
timeout = schedule_timeout(timeout); \
} \
}while(0);
#endif

#define USER_PATH "driver/tnt-io" 

#define MSG_QUEUE_CNT				128
#define NEXT_MSG_QUEUE(index)   ((index+1) % MSG_QUEUE_CNT)

typedef struct _IO_DATA_ST_
{
	uint8_t type;
	uint16_t data0;
	uint16_t data1;
} IO_DATA_ST, *P_IO_DATA_ST;


typedef struct _MSG_QUEUE_ST_
{
	IO_DATA_ST buf[MSG_QUEUE_CNT];
    int front;
    int rear;
	int cnt;
}MSG_QUEUE_ST, *P_MSG_QUEUE_ST;

static MSG_QUEUE_ST g_msgQueue;

static struct proc_dir_entry *g_tnt_io_entry; 
static struct platform_device *g_pdev;


static struct task_struct *kthread_id = NULL;
struct mutex tnt_io_kthread_readData_mutex;

int g_delayVal = 0;

int g_gpio_io_sda = 0;
int g_gpio_io_scl = 0;

int g_gpio_io_int = 0;
int g_gpio_io_rst = 0;
int g_gpio_io_mot = 0;
int g_gpio_io_pir = 0;

int g_irq_touch = 0;
int g_irq_pir = 0;

static int g_userCounter = 0;
//static int g_get_touch_counter = 0;
//static int g_get_light_counter = 0;
//static int g_get_temperature_counter = 0;

static IO_DATA_ST g_ioData;

//int g_initDriver_flag = 0;
	
static DECLARE_WAIT_QUEUE_HEAD(io_watiQueue);


static int g_io_eventFlag = true;

rwlock_t g_open_user_lock;

rwlock_t g_readSensor_lock;

rwlock_t g_get_touch_lock;
rwlock_t g_initFlag_lock;
rwlock_t g_get_temperature_lock;
rwlock_t g_get_light_lock;



/******************************************************************************
 * Function	: SCL_HIGH
 * Note		:
 ******************************************************************************/
void SCL_HIGH(void)
{
	IO_GPIO_HIGH(g_gpio_io_scl);
	i2c_delay(g_delayVal);
}
/******************************************************************************
 * Function	: SCL_LOW
 * Note		:
 ******************************************************************************/
void SCL_LOW(void)
{
	IO_GPIO_LOW(g_gpio_io_scl);
	i2c_delay(g_delayVal);
}
/******************************************************************************
 * Function	: DAT_HIGH
 * Note		:
 ******************************************************************************/
void DAT_HIGH(void)
{
	IO_GPIO_HIGH(g_gpio_io_sda);
	i2c_delay(g_delayVal);

}

/******************************************************************************
 * Function	: DAT_LOW
 * Note		:
 ******************************************************************************/
void DAT_LOW(void)
{
	IO_GPIO_LOW(g_gpio_io_sda);
	i2c_delay(g_delayVal);
}

/******************************************************************************
 * Function	: DAT_SET_IN
 * Note		:
 ******************************************************************************/
void DAT_SET_IN(void)
{
	IO_GPIO_IN_MODE(g_gpio_io_sda);
	i2c_delay(g_delayVal);
}

/******************************************************************************
 * Function	: DAT_SET_IN
 * Note		:
 ******************************************************************************/
int IO_READ_DATA(void)
{
	int rData = gpio_get_value(g_gpio_io_sda) & 0x01;
	return rData;
}

/******************************************************************************
 * Function	: init_msgQueue
 * Note		:
 ******************************************************************************/
void init_msgQueue(void)
{
	if(mutex_lock_interruptible(&tnt_io_kthread_readData_mutex) == 0)
	{
		memset(&g_msgQueue, 0, sizeof(MSG_QUEUE_ST));
		mutex_unlock(&tnt_io_kthread_readData_mutex);
	}
}
/******************************************************************************
 * Function	: isFull_msgQueue
 * Note		:
 ******************************************************************************/
int isFull_msgQueue(void)
{
	int frontVal = 0;
	int rearVal = 0;

	if(mutex_lock_interruptible(&tnt_io_kthread_readData_mutex) == 0)
	{
		frontVal = g_msgQueue.front;
		rearVal = g_msgQueue.rear;

		mutex_unlock(&tnt_io_kthread_readData_mutex);

		return (NEXT_MSG_QUEUE(rearVal) == frontVal);
	}

	return true;
}
/******************************************************************************
 * Function	: isEmpty_msgQueue
 * Note		:
 ******************************************************************************/
int isEmpty_msgQueue(void)
{
	int result = false;
	int frontVal = 0;
	int rearVal = 0;

	if(mutex_lock_interruptible(&tnt_io_kthread_readData_mutex) == 0)
	{
		frontVal = g_msgQueue.front;
		rearVal = g_msgQueue.rear;
		mutex_unlock(&tnt_io_kthread_readData_mutex);
	}

	if(frontVal == rearVal)
	{
		result = true;
	}
	
	return result;
}
/******************************************************************************
 * Function	: push_msgQueue
 * Note		:
 ******************************************************************************/
int push_msgQueue(uint8_t type, uint16_t val)
{
	int ret = false;
	
	if (isFull_msgQueue())
	{
		return ret;
	}

	if(mutex_lock_interruptible(&tnt_io_kthread_readData_mutex) == 0)
	{
		g_msgQueue.buf[g_msgQueue.rear].type = type;
		g_msgQueue.buf[g_msgQueue.rear].data0 = val;
		g_msgQueue.rear = NEXT_MSG_QUEUE(g_msgQueue.rear);
		g_msgQueue.cnt++;

		mutex_unlock(&tnt_io_kthread_readData_mutex);

		ret = true;
	}

	return ret;

}

/******************************************************************************
 * Function	: isEmpty_msgQueue
 * Note		:
 ******************************************************************************/
int pull_msgQueue(uint8_t *pType, uint16_t *pVal)
{
	int ret = false;
	
	if (isEmpty_msgQueue())
	{
		return ret;
	}

	if(mutex_lock_interruptible(&tnt_io_kthread_readData_mutex) == 0)
	{
		*pType = g_msgQueue.buf[g_msgQueue.front].type;
		*pVal = g_msgQueue.buf[g_msgQueue.front].data0;
		g_msgQueue.front = NEXT_MSG_QUEUE(g_msgQueue.front);
		g_msgQueue.cnt--;

		mutex_unlock(&tnt_io_kthread_readData_mutex);

		ret = true;
	}

	return ret;

}


/******************************************************************************
 * Function	: DAT_SET_IN
 * Note		:
 ******************************************************************************/
int tnt_io_kthread_readData(void *data)
{

 	uint16_t readVal = 0;
	uint16_t read_data0 = -1;
	uint16_t read_data1 = -1;
	
	
	uint8_t jobNumber = 0;
    pr_info("[%s:%d] START, kthread..!@!@!@!@!@!@\n\n", _FN_, _LN_);
	
	init_msgQueue();


	while (true)
	{
		if(pull_msgQueue(&jobNumber, &read_data0) == true)
		{
//			pr_info("[%s:%d] Get Job Number = %d\n", _FN_, _LN_, jobNumber);
			
			switch(jobNumber)
			{
				case READ_JOB_TOUCH:
					read_data0 = i2c_read_touch();
					//printk("[%s:%d] get : 0x%04X\r\n", _FN_, _LN_, read_data0);
					
					read_lock( &g_readSensor_lock );
					g_ioData.type = jobNumber;
					g_ioData.data0 = read_data0;
					g_ioData.data1 = -1;
					read_unlock( &g_readSensor_lock );

					g_io_eventFlag = 1;
					wake_up_interruptible(&io_watiQueue); 


//					write_lock( &g_get_touch_lock );
//					g_get_touch_counter--;
//					write_unlock( &g_get_touch_lock );
					break;
					
				case READ_JOB_LIGHT:
					read_data0 = i2c_read_light(0x88);
					read_data0 = (read_data0 << 8) | (i2c_read_light(0x89) & 0xFF);

					read_data1 = i2c_read_light(0x8A);
					read_data1 = (read_data1 << 8) | (i2c_read_light(0x8B) & 0xFF);

					readVal = (read_data0 + read_data1) / 2;
					//printk("[%s:%d] IRQ, Read Light 0x%X, 0x%X --> 0x%X\r\n", _FN_, _LN_, read_light_ch0, read_light_ch1, readVal);

					read_lock( &g_readSensor_lock );
					g_ioData.type = jobNumber;
					g_ioData.data0 = readVal;
					g_ioData.data1 = -1;
					read_unlock( &g_readSensor_lock );

					g_io_eventFlag = 1;
					wake_up_interruptible(&io_watiQueue); 

//					write_lock( &g_get_light_lock );
//					g_get_light_counter--;
//					write_unlock( &g_get_light_lock );

					break;

				case READ_JOB_TEMPERATURE:
					read_data0 = (double)i2c_read_temp(HDC_TEMPERATURE);
					read_data1 = (double)i2c_read_temp(HDC_HUMIDITY);

//					printk("[%s:%d] Read Temp, Hum = %d, %d\r\n", _FN_, _LN_, read_data0, read_data1);
#if 1
					read_lock( &g_readSensor_lock );
					g_ioData.type = jobNumber;
					g_ioData.data0 = read_data0;
					g_ioData.data1 = read_data1;
					read_unlock( &g_readSensor_lock );

					g_io_eventFlag = 1;
					wake_up_interruptible(&io_watiQueue); 

#endif
					break;

				case READ_JOB_PIR: 
//					printk("[%s:%d] PIR = %d\r\n", _FN_, _LN_, read_data0);
#if 1
					read_lock( &g_readSensor_lock );
					g_ioData.type = jobNumber;
					g_ioData.data0 = read_data0;
					g_ioData.data1 = read_data1;
					read_unlock( &g_readSensor_lock );

					g_io_eventFlag = 1;
					wake_up_interruptible(&io_watiQueue); 

#endif
					break;

				case CTL_JOB_MOTOR:
					gpio_direction_output(g_gpio_io_mot, read_data0);					
					break;
					
				default:
					break;
			}
		}
		else
		{
			
			msleep_interruptible(1);
		}
	}

	pr_info("[%s:%d] kernel thread exits.\n", _FN_, _LN_);
	return 0;
}




// *****************************************************************************
// Function 	: tnt_io_proc_open
// Note			: 
// *****************************************************************************
static int tnt_io_proc_open(struct inode *inode, struct file *file)
{
//	int initDriver_flag = 0;
//	int currentUserCnt = 0;
    pr_info("[%s:%d]\n\n", _FN_, _LN_);


#if 0
	read_lock( &g_initFlag_lock );
	initDriver_flag = g_initDriver_flag;
	read_unlock( &g_initFlag_lock );

	if(initDriver_flag == 0)
	{
		write_lock( &g_initFlag_lock );
		g_initDriver_flag++;
		write_unlock( &g_initFlag_lock );
	}

	read_lock( &g_open_user_lock );
	currentUserCnt = g_userCounter;
	read_unlock( &g_open_user_lock );

	if(currentUserCnt == 0)
	{
		
	}
	else
	{

	}
#endif

	write_lock( &g_open_user_lock );
	g_userCounter++;
	write_unlock( &g_open_user_lock );
	
    return 0;
}


// *****************************************************************************
// Function 	: tnt_io_proc_open
// Note			: 
// *****************************************************************************
static int tnt_io_proc_release(struct inode *inode, struct file *file)
{
    pr_info("[%s:%d]\n\n", _FN_, _LN_);

	write_lock( &g_open_user_lock );
	g_userCounter--;
	write_unlock( &g_open_user_lock );
		
    return 0;
}

// *****************************************************************************
// Function 	: tnt_io_proc_ioctl
// Note			: 
// *****************************************************************************
static ssize_t tnt_io_proc_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)  
{
	int status = 0;
//	int read_data0, read_data1;	

	//printk("[%s:%d] cmd = 0x%X\r\n", _FN_, _LN_, cmd);

	switch(cmd)
	{
		case IOCTL_READ_LIGHT:
//			write_lock( &g_get_light_lock );
//			g_get_light_counter++;
//			write_unlock( &g_get_light_lock );
			push_msgQueue(READ_JOB_LIGHT, 0);
			break;
			
		case IOCTL_READ_TEMPERRATURE:
#if 0

			read_data0 = i2c_read_temp(HDC_TEMPERATURE);
			read_data1 = i2c_read_temp(HDC_HUMIDITY);

//			printk("[%s:%d] Read Temp, Hum = %d, %d\r\n", _FN_, _LN_, read_data0, read_data1);

#else
			push_msgQueue(READ_JOB_TEMPERATURE, 0);
#endif

			
			break;

		case IOCTL_SET_MOTOR:
			get_user(status, (int *)arg);

			if(status == 0)
			{
//				gpio_direction_output(g_gpio_io_mot, 1);
				push_msgQueue(CTL_JOB_MOTOR, 1);
			}
			else
			{
//				gpio_direction_output(g_gpio_io_mot, 0);
				push_msgQueue(CTL_JOB_MOTOR, 0);
			}
			break;

		case IOCTL_SET_MOTOR_ON:
			//printk("Moter On..!@!@!@\r\n");
			gpio_direction_output(g_gpio_io_mot, 0);		// on : 0
			break;

		case IOCTL_SET_MOTOR_OFF:
			//printk("Moter OFF..!@!@!@\r\n");
			gpio_direction_output(g_gpio_io_mot, 1);		// off : 1
			break;

		case IOCTL_GPIO_67:
			get_user(status, (int *)arg);
			//printk("ctl gpio 67 = %d\r\n", status);
			if(status == 0)
			{
				gpio_direction_output(67, 1);
			}
			else
			{
				gpio_direction_output(67, 0);
			}
			break;
	}

	//printk("[%s:%d] --------->>>>>>>>>>>>>>>>>>>>>>>\r\n", _FN_, _LN_);

    return 0;  
} 
// *****************************************************************************
// Function 	: tnt_io_proc_open
// Note			: 
// *****************************************************************************
static ssize_t tnt_io_proc_read(struct file *file, char __user * buf, size_t lbuf, loff_t * ppos)
{
	int retVal = 0;
    int nbytes = -1;
	IO_DATA_ST ioData;

	//printk("[%s:%d] <--------------------------\r\n", _FN_, _LN_);

//	interruptible_sleep_on(&touch_watiQueue);
	wait_event_interruptible(io_watiQueue, g_io_eventFlag != 0);
	g_io_eventFlag = 0;


	//printk("[%s:%d] --------------------------\r\n", _FN_, _LN_);
	
	read_lock( &g_readSensor_lock );
	memcpy(&ioData, &g_ioData, sizeof(IO_DATA_ST));
	read_unlock( &g_readSensor_lock );

	//printk("[%s:%d] Snd to User : type, data0, data1 = 0x%04X, 0x%04X, 0x%04X\r\n", _FN_, _LN_, ioData.type, ioData.data0, ioData.data1);
	
	retVal = copy_to_user((void *)buf, (const void *)&ioData, (unsigned long)sizeof(IO_DATA_ST));

	if(retVal == 0)
	{
		nbytes = sizeof(IO_DATA_ST);
	}
	
	return nbytes;
}

// *****************************************************************************
// Function 	: tnt_io_proc_open
// Note			: 
// *****************************************************************************
static ssize_t tnt_io_proc_write(struct file *file, const char __user * buf, size_t lbuf, loff_t * ppos)
{
    int nbytes = 0;

    pr_info("[%s:%d]\n\n", _FN_, _LN_);

//	nbytes = bytes_to_do - copy_from_user(ramdisk + *ppos, buf, bytes_to_do);
    return nbytes;
}

// *****************************************************************************
// struct 	: tnt_io_proc_fops
// Note			: 
// *****************************************************************************
static const struct file_operations tnt_io_proc_fops = {
    .owner = THIS_MODULE,
	.read = tnt_io_proc_read,
    .write = tnt_io_proc_write,
    .open = tnt_io_proc_open,
    .release = tnt_io_proc_release,
//	.ioctl = tnt_io_proc_ioctl,
	.unlocked_ioctl = tnt_io_proc_ioctl,
	.compat_ioctl =    tnt_io_proc_ioctl,
};

#if 0

// *****************************************************************************
// Function 	: tnt_pir_irq
// Note			: 
// *****************************************************************************
 static irqreturn_t tnt_pir_irq(int irq, void *dev_id)
 {
	int initFlag = 0;
	int getVal;
	getVal = gpio_get_value(g_gpio_io_pir);
	
	read_lock( &g_initFlag_lock );
	initFlag = g_initDriver_flag;
	read_unlock( &g_initFlag_lock );

	if(initFlag != 1)
	{
//		printk("[%s:%d]  Don't init driver..!@!!@\r\n", _FN_, _LN_);		
		return IRQ_HANDLED;
	}

	//printk("[%s:%d] IRQ, Input to PIR sensor ---> %d.!@!@\r\n", _FN_, _LN_, (getVal & 0x01));

	push_msgQueue(READ_JOB_PIR, getVal);


	return IRQ_HANDLED;
 }
#endif


 // *****************************************************************************
// Function 	: tnt_io_proc_open
// Note			: 
// *****************************************************************************
 static irqreturn_t tnt_touch_irq(int irq, void *dev_id)
 {
	int initFlag = 0;
//	uint16_t getVal = 0;

//	printk("[%s:%d] <--------------------------\r\n", _FN_, _LN_);

	read_lock( &g_open_user_lock );
	initFlag = g_userCounter;
	read_unlock( &g_open_user_lock );

	if(initFlag == 0)
	{
		return IRQ_HANDLED;
	}


	


#if 0

	read_lock( &g_initFlag_lock );
	initFlag = g_initDriver_flag;
	read_unlock( &g_initFlag_lock );



	if(initFlag == 0)
	{
		printk("[%s:%d]  Don't init driver..!@!!@\r\n", _FN_, _LN_);	
		return IRQ_HANDLED;
	}

	read_lock( &g_get_touch_lock );
	interruptCounter = g_get_touch_counter;
	read_unlock( &g_get_touch_lock );

	if(interruptCounter != 0)
	{
//		printk("[%s:%d]  Don't finish interrupt..!@!!@\r\n", _FN_, _LN_);		
		return IRQ_HANDLED;
	}
#endif

//	write_lock( &g_get_touch_lock );
//	interruptCounter = g_get_touch_counter++;
//	write_unlock( &g_get_touch_lock );


	//printk("[%s:%d]  read touch val = 0x%X\n", _FN_, _LN_, getVal);

	push_msgQueue(READ_JOB_TOUCH, 0);


	return IRQ_HANDLED;
 }
 
// *****************************************************************************
// Function 	: tnt_io_proc_open
// Note			: 
// *****************************************************************************
static int tnt_io_probe(struct platform_device *pdev)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	
	int ret = -1;

	enum of_gpio_flags flag;
		
	struct device_node *input_node = pdev->dev.of_node;

    pr_info("[%s:%d]\n\n", _FN_, _LN_);

	rwlock_init(&g_open_user_lock);
	rwlock_init(&g_readSensor_lock);
	rwlock_init(&g_get_touch_lock);
	rwlock_init(&g_get_light_lock);
	rwlock_init(&g_get_temperature_lock);
	rwlock_init(&g_initFlag_lock);

	mutex_init(&tnt_io_kthread_readData_mutex);

    g_pdev = pdev;


	g_gpio_io_sda = of_get_named_gpio_flags(input_node,"io-sda", 0, &flag);

	printk(KERN_INFO "[%s:%d] gpio, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_io_sda, flag);
	
	if (!gpio_is_valid(g_gpio_io_sda))
	{
		printk(KERN_INFO "[%s:%d] of_get_named_gpio_flags, gpio : %d\n", __FUNCTION__, __LINE__, g_gpio_io_sda);
		return -1;
	} 
	
    ret = gpio_request(g_gpio_io_sda, "io-sda");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] gpio_request, gpio : %d\n", __FUNCTION__, __LINE__, g_gpio_io_sda);
		gpio_free(g_gpio_io_sda);
		ret = -EIO;
		goto EXIT_FAIL;
	}
	gpio_direction_output(g_gpio_io_sda, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));

	//---------------------------------------------------------------------------------------------------
	g_gpio_io_scl = of_get_named_gpio_flags(input_node,"io-scl", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_io_scl, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_io_scl, flag);

	if (!gpio_is_valid(g_gpio_io_scl)){
		printk(KERN_INFO "[%s:%d] of_get_named_gpio_flags, g_gpio_io_scl : %d\n", __FUNCTION__, __LINE__, g_gpio_io_scl);
		return -1;
	} 

    ret = gpio_request(g_gpio_io_scl, "io-scl");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] gpio_request, g_gpio_io_scl : %d\n", __FUNCTION__, __LINE__, g_gpio_io_scl);
		gpio_free(g_gpio_io_scl);
		ret = -EIO;
		goto EXIT_FAIL;
	}
	gpio_direction_output(g_gpio_io_scl, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));


	//---------------------------------------------------------------------------------------------------
	g_gpio_io_rst = of_get_named_gpio_flags(input_node,"io-rst", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_io_rst, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_io_rst, flag);

	if (!gpio_is_valid(g_gpio_io_rst)){
		printk(KERN_INFO "[%s:%d] g_gpio_io_rst : %d\n", __FUNCTION__, __LINE__, g_gpio_io_rst);
		return -1;
	} 

    ret = gpio_request(g_gpio_io_rst, "io-rst");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] g_gpio_io_rst : %d\n", __FUNCTION__, __LINE__, g_gpio_io_rst);
		gpio_free(g_gpio_io_rst);
		ret = -EIO;
		goto EXIT_FAIL;
	}
	gpio_direction_output(g_gpio_io_rst, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));


	//---------------------------------------------------------------------------------------------------
	g_gpio_io_mot = of_get_named_gpio_flags(input_node,"io-mot", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_io_mot, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_io_mot, flag);

	if (!gpio_is_valid(g_gpio_io_mot)){
		printk(KERN_INFO "[%s:%d] g_gpio_io_rst : %d\n", __FUNCTION__, __LINE__, g_gpio_io_mot);
		return -1;
	} 

    ret = gpio_request(g_gpio_io_mot, "io-mot");
	if (ret != 0)
	{
		printk(KERN_INFO "[%s:%d] g_gpio_io_rst : %d\n", __FUNCTION__, __LINE__, g_gpio_io_mot);
		gpio_free(g_gpio_io_mot);
		ret = -EIO;
		goto EXIT_FAIL;
	}
	gpio_direction_output(g_gpio_io_mot, ((flag == OF_GPIO_ACTIVE_LOW)? 0:1));

#if 0
	//---------------------------------------------------------------------------------------------------
	g_gpio_io_pir = of_get_named_gpio_flags(input_node,"io-pir", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_io_int, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_io_pir, flag);

	if (!gpio_is_valid(g_gpio_io_pir)){
		printk(KERN_INFO "[%s:%d] g_gpio_io_pir : %d\n", __FUNCTION__, __LINE__, g_gpio_io_pir);
		return -1;
	} 

	g_irq_pir = gpio_to_irq(g_gpio_io_pir);
	if (g_irq_pir)
	{
		ret = gpio_request(g_gpio_io_pir, "io-pir");
		if (ret != 0)
		{
			printk(KERN_INFO "[%s:%d] g_gpio_io_pir : %d\n", __FUNCTION__, __LINE__, g_gpio_io_pir);
			gpio_free(g_gpio_io_pir);
			ret = -EIO;
			goto EXIT_FAIL;
		}

		ret = request_irq(g_irq_pir, tnt_pir_irq, flag, "io-pir", NULL);
		if (ret != 0)
		{
			free_irq(g_irq_pir, NULL);
			dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
     	}
	}

	IO_GPIO_IN_MODE(g_gpio_io_pir);
#endif

	//---------------------------------------------------------------------------------------------------
	g_gpio_io_int = of_get_named_gpio_flags(input_node,"io-int", 0,&flag);

	printk(KERN_INFO "[%s:%d] g_gpio_io_int, flag = %d, %d\n", __FUNCTION__, __LINE__, g_gpio_io_int, flag);

	if (!gpio_is_valid(g_gpio_io_int)){
		printk(KERN_INFO "[%s:%d] of_get_named_gpio_flags, g_gpio_io_scl : %d\n", __FUNCTION__, __LINE__, g_gpio_io_scl);
		return -1;
	} 

	g_irq_touch = gpio_to_irq(g_gpio_io_int);
	if (g_irq_touch)
	{
		ret = gpio_request(g_gpio_io_int, "io-int");
		if (ret != 0)
		{
			printk(KERN_INFO "[%s:%d] gpio_request, g_gpio_io_int : %d\n", __FUNCTION__, __LINE__, g_gpio_io_int);
			gpio_free(g_gpio_io_int);
			ret = -EIO;
			goto EXIT_FAIL;
		}

		ret = request_irq(g_irq_touch, tnt_touch_irq, flag, "io-int", NULL);
		if (ret != 0)
		{
			free_irq(g_irq_touch, NULL);
			dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
     	}
	}

// Create a test entry under USER_ROOT_DIR   
	g_tnt_io_entry = proc_create(USER_PATH, 0666, NULL, &tnt_io_proc_fops);   
    if (NULL == g_tnt_io_entry)   
    {   
        goto EXIT_FAIL;   
    }

	i2c_init();
	

	
#if 1


	



	kthread_id = kthread_run(tnt_io_kthread_readData, NULL, "TEST kthread");


	sched_setscheduler(kthread_id, SCHED_FIFO, &param);
	wake_up_process(kthread_id);
	
#endif
	pr_info("[%s:%d] Success...!!!!!\n\n", _FN_, _LN_);

	return 0;  //return Ok

EXIT_FAIL:
	return ret;
}



// *****************************************************************************
// Function 	: tnt_io_remove
// Note			: 
// *****************************************************************************
static int tnt_io_remove(struct platform_device *pdev)
{ 
    pr_info("[%s:%d]\n\n", _FN_, _LN_);
    return 0;
}




// *****************************************************************************
// struct 	: 
// Note		: 
// *****************************************************************************
#ifdef CONFIG_OF
static const struct of_device_id of_rk_tnt_io_match[] = {
	{ .compatible = "tnt,io" },
	{ /* Sentinel */ }
};
#endif
static struct platform_driver tnt_io_driver = {
	.probe		= tnt_io_probe,
	.remove		= tnt_io_remove,
	.driver		= {
	.name	= "tnt-io",
	.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_rk_tnt_io_match,
#endif
	},
};

// *****************************************************************************
// Function 	: tnt_io_init
// Note			: 
// *****************************************************************************
static int __init tnt_io_init(void)
{


    pr_info("[%s:%d]\n\n", _FN_, _LN_);
	


    return platform_driver_register(&tnt_io_driver);
}

static void __exit tnt_io_exit(void)
{
	int retVal = 0;
	
    pr_info("[%s:%d]\n\n", _FN_, _LN_);

	if(kthread_id)
	{
		retVal = kthread_stop(kthread_id);
		if(retVal != -EINTR)
		{
			printk("Main logic tread stopped.\n");
				
			kthread_id = NULL;
		}
	}

	platform_driver_unregister(&tnt_io_driver);
}

subsys_initcall(tnt_io_init);
module_exit(tnt_io_exit);

MODULE_AUTHOR("ygyu <ygyu@thountech.com>");
MODULE_DESCRIPTION("TNT IO driver");
MODULE_LICENSE("GPL");



