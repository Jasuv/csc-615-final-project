#ifndef COLOR_LIB_H
#define COLOR_LIB_H

#include <stdint.h>
#include "DEV_Config.h"

// internal sensor constants
#define TCS_ADDR		0x29
#define TCS_CMD			0x80
#define TCS_REG_ENABLE	0x00
#define TCS_REG_ATIME	0x01
#define TCS_REG_CONTROL	0x0F
#define TCS_REG_ID		0x12
#define TCS_REG_CDATA	0x14
#define TCS_REG_RDATA	0x1c
#define TCS_REG_GDATA	0x18
#define TCS_REG_BDATA	0x1A
#define TCS_PON			0x01
#define TCS_AEN			0x0b

// integration time
#define INTEGRATIONTIME_2_4MS 0xFF //   1 cycles - Max Count: 1024
#define INTEGRATIONTIME_24MS  0xF6 //  10 cycles - Max Count: 10245
#define INTEGRATIONTIME_50MS  0xEB //  20 cycles - Max Count: 20480
#define INTEGRATIONTIME_101MS 0xD5 //  42 cycles - Max Count: 43008
#define INTEGRATIONTIME_154MS 0xC0 //  64 cycles - Max Count: 65535
#define INTEGRATIONTIME_700MS 0x00 // 256 cycles - Max Count: 65535

// gain
#define GAIN_1X  0x00 // no gain
#define GAIN_4X  0x01 // 4x gain
#define GAIN_16X 0x02 // 16x gain
#define GAIN_60X 0x03 // 60x gain


typedef struct {
	char name[20];
	uint32_t hexValue;
	float confidence;
} ColorResult;

typedef struct {
	const char* name;
	uint32_t hex;
} ColorTarget;


# define COLOR_COUNT 17
static const ColorTarget PROJECT_COLORS[] = {
	{"Red", 0xFF0000}, 
	{"Burgundy", 0x800020}, 
	{"Orange", 0xFF6E00},
	{"Brown", 0x8C6036}, 
	{"Yellow", 0xFFFF00}, 
	{"Green", 0x00FF00},
	{"Lime", 0x85FF00}, 
	{"Olive", 0x636B2F}, 
	{"Blue", 0x0000FF},
	{"Cyan", 0x00FFFF},
	{"Purple", 0x800080}, 
	{"Magenta", 0xFF00FF},
	{"Lavender", 0xA76EEE}, 
	{"Pink", 0xFF6289}, 
	{"Gray", 0x808080},
	{"White", 0xFFFFFF}, 
	{"Black", 0x000000}
};

int ColorLib_Init(void);
ColorResult ColorLib_GetMatch(void);

#endif
