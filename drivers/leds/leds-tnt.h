// *****************************************************************************
// Project 	: 
// Date		: 2017.08.11
// Device	: 
// Note		:
// *****************************************************************************

// *****************************************************************************
// Define
// *****************************************************************************
#define _FN_			__FUNCTION__
#define _LN_			__LINE__

#define I2C_READ			1

#define LED_R_ADDR			0xC0
#define LED_G_ADDR			0xC2
#define LED_B_ADDR			0xC4

#define LED_OUT0			0x14
#define LED_OUT1			0x15
#define LED_OUT2			0x16
#define LED_OUT3			0x17

#define MAX_CHANNEL			15

#define PWM_MAX				255
#define PWM_PIR				32

//#define DELAY_CNT			50
#define DELAY_CNT			0	/* srcho, GPIO Toggle 시간이 느려서 delay 불필요! */

#define CMD_SET_FULL		0x100
#define CMD_SET_ONE			0x101
#define	CMD_SET_SELECT		0x102
#define CMD_SET_BLINK		0x103

#define CMD_REG_WRITE		0x201
#define CMD_REG_READ		0x202

#define REG_BUFFER_SIZE 	31

#if 0	//org

#define LED_GPIO_HIGH(a)	gpio_direction_output(a, 1)
#define LED_GPIO_LOW(a)		gpio_direction_output(a, 0)
#define LED_GPIO_IN_MODE(a)	gpio_direction_input(a)

#else	//srcho edited

#define LED_GPIO_HIGH(a)		gpio_set_value(a, 1)
#define LED_GPIO_LOW(a)			gpio_set_value(a, 0)

#define LED_GPIO_OUT_MODE(a, b)	gpio_direction_output(a, b)
#define LED_GPIO_IN_MODE(a)		gpio_direction_input(a)

#endif

typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;

typedef enum
{ 
  LED_COLOR_B = 0,
  LED_COLOR_G,
  LED_COLOR_R
}LED_COLOR;

typedef struct {
	uint8_t addr;
	uint8_t reg;
	uint8_t data;
}Led_reg_t;

typedef struct {
	int pwm_r[MAX_CHANNEL];
	int pwm_g[MAX_CHANNEL];
	int pwm_b[MAX_CHANNEL];
}Led_pwm_t;

typedef struct {
	LED_COLOR led_color;
	char num;
}Led_set_one_t;

typedef struct {
	LED_COLOR led_color;
	char num;
	char pwm;
}Led_select_one_t;

typedef struct {
	LED_COLOR led_color;
	char num;
	char enable;
}Led_blink_t;

// *****************************************************************************
// Extern
// *****************************************************************************
extern int g_gpio_led_sda;
extern int g_gpio_led_scl;
extern uint16_t led_color;

// *****************************************************************************
// Function
// *****************************************************************************
void LED_SCL_HIGH(void);
void LED_SCL_LOW(void);
void LED_DAT_HIGH(void);
void LED_DAT_LOW(void);
void LED_DAT_SET_IN(void);
int LED_READ_DATA(void);

void led_i2c_init(void);
void led_i2c_delay(uint32_t delay);
void led_i2c_byte_write(int devAddr, int regAddr, int data);
int led_i2c_byte_read(int devAddr, int regAddr);
void led_i2c_buff_write(int devAddr, int regAddr, int count, int *buffer);

void led_write_color_full(uint16_t r, uint16_t g, uint16_t b);
void led_set_one(uint16_t number);
void led_set_select(uint16_t color, uint16_t number, uint16_t on);

