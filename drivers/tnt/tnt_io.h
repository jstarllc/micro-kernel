// *****************************************************************************
// Project 	: 
// Date		: 2017.08.11
// Device	: 
// Note		:
// *****************************************************************************

// *****************************************************************************
// Define
// *****************************************************************************
#define _FN_				__FUNCTION__
#define _LN_				__LINE__
#define I2C_READ			1
#define TUOCH_I2C_ADDR		0x84
#define LIGHT_I2C_ADDR		0x52

#define HDC1080_I2C_ADDR	0x80


#define HDC_TEMPERATURE			0x00
#define HDC_HUMIDITY			0x01
#define HDC_CONFIGURATION		0x02
#define HDC_SERIAL_ID1			0xFB
#define HDC_SERIAL_ID2			0xFC
#define HDC_SERIAL_ID3			0xFD
#define HDC_MANUFACTURER_ID		0xFE
#define HDC_DEVICE_ID			0xFF


#define TOUCH_DELAY_CNT		35
#define LIGHT_DELAY_CNT		35

#define RESULT_SUCCESS		0
#define RESULT_FAIL			-1

#define IO_GPIO_HIGH(a)		gpio_direction_output(a, 1)
#define IO_GPIO_LOW(a)		gpio_direction_output(a, 0)
#define IO_GPIO_IN_MODE(a)	gpio_direction_input(a)

typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;


// *****************************************************************************
// Extern
// *****************************************************************************
extern int g_gpio_input_sda;
extern int g_gpio_input_scl;

extern int g_delayVal;
extern int g_initDriver_flag;

// *****************************************************************************
// Function
// *****************************************************************************
void SCL_HIGH(void);
void SCL_LOW(void);
void DAT_HIGH(void);
void DAT_LOW(void);
void DAT_SET_IN(void);
int IO_READ_DATA(void);

void i2c_init(void);
void i2c_delay(unsigned long delay);

uint16_t i2c_read_touch(void);
uint16_t i2c_read_light(uint8_t regAddr);
void i2c_write_light(uint8_t regAddr, uint8_t cmd);
uint16_t i2c_read_temp(int regAddr);
void i2c_write_tmp(uint8_t regAddr, uint16_t cmd);




