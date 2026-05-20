/**************************************************************
* Class::  CSC-615-01 Spring 2026
* Name:: Haibin Cao, Eric Ahsue, Kiran Khatri, John Tsiglieris
* Student ID:: 923756077, 922711514, 925750019, 923593954
* GitHub-Name:: haibinc, Jasuv, khatri5034, John-Tsiglieris
* Project:: 
*
* File:: ColorLib.c
*
* Description:: Implement an RGB detection system by reading sensor data 
* and using I2C communication between sensor and raspberry PI.
* 
* Blue = stop for 5 seconds, then continue
* Red = stop permanently
* 
**************************************************************/

#include "ColorLib.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// write to sensor
static void sensor_write(uint8_t reg, uint8_t data) { 
	I2C_Write_Byte(reg | TCS_CMD, data); 
}

// read 16 bits from sensor over I2C
static uint16_t sensor_read(uint8_t reg) { 
    DEV_I2C_Init(TCS_ADDR);
	return I2C_Read_Word(reg | TCS_CMD | 0x20); 
}

int ColorLib_Init(void) {
	// sensor register check
	DEV_I2C_Init(TCS_ADDR);
	uint8_t id = I2C_Read_Byte(TCS_REG_ID | TCS_CMD);
	if (id != 0x44 && id != 0x4D) return 1;

	// sensor init
	sensor_write(TCS_REG_ATIME, INTEGRATIONTIME_154MS);
	sensor_write(TCS_REG_CONTROL, GAIN_1X);
	sensor_write(TCS_REG_ENABLE, TCS_PON);
	DEV_Delay_ms(3);
	sensor_write(TCS_REG_ENABLE, TCS_PON | TCS_AEN);

	return 0;
}

ColorResult ColorLib_GetMatch(void) {
    ColorResult result;

    uint16_t C = sensor_read(TCS_REG_CDATA);
    uint16_t R = sensor_read(TCS_REG_RDATA);
    uint16_t G = sensor_read(TCS_REG_GDATA);
    uint16_t B = sensor_read(TCS_REG_BDATA);

    printf("RAW -> C:%u R:%u G:%u B:%u\n", C, R, G, B);

    // red match
    if ( R > B && R > G) {
        strcpy(result.name, "Red");
        result.hexValue = 0xFF0000;
        result.confidence = 90;
        return result;
    }

    // blue match
    if ( C >170 && C < 310 && B > R && B > G) {
        strcpy(result.name, "Blue");
        result.hexValue = 0x0000FF;
        result.confidence = 95;
        return result;
    }

    // unknown match
    strcpy(result.name, "Unknown");
    result.hexValue = 0x000000;
    result.confidence = 0;
    
    return result;
}