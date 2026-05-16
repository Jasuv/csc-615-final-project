#include "ColorLib.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// write to sensor
static void sensor_write(uint8_t reg, uint8_t data) { 
    I2C_Write_Byte(reg | TCS_CMD, data); 
}

// read 16 bits from sensor
static uint16_t sensor_read(uint8_t reg) { 
	return I2C_Read_Word(reg | TCS_CMD | 0x20); 
}

int ColorLib_Init(void) {

	// sensor register check
	DEV_I2C_Init(TCS_ADDR);
	uint8_t id = I2C_Read_Byte(TCS_REG_ID | TCS_CMD);
	if (id != 0x44 && id != 0x4D) return 1;

	// sensor init
	sensor_write(TCS_REG_ATIME, INTEGRATIONTIME_700MS);
	sensor_write(TCS_REG_CONTROL, GAIN_16X);
	sensor_write(TCS_REG_ENABLE, TCS_PON);
	DEV_Delay_ms(3);
	sensor_write(TCS_REG_ENABLE, TCS_PON | TCS_AEN);

	return 0;
}

ColorResult ColorLib_GetMatch(void) {
    ColorResult bestMatch;

    // We set a very wide threshold (200).
    // This captures desaturated "slate" colors but still excludes the "wrong" side of the color wheel.
    const float DISTANCE_THRESHOLD = 200.0f;

    // 1. Get Raw Data
    uint16_t R = sensor_read(TCS_REG_RDATA);
    uint16_t G = sensor_read(TCS_REG_GDATA);
    uint16_t B = sensor_read(TCS_REG_BDATA);

    printf("[ColorLib] Raw Sensor Data - R: %u, G: %u, B: %u\n", R, G, B);

    // 2. Normalize to 8-bit (Brightness scaling)
    float maxVal = (R > G && R > B) ? R : (G > B ? G : B);
    float factor = (maxVal > 255) ? (maxVal / 255.0f) : 1.0f;

    int currR = (int)(R / factor);
    int currG = (int)(G / factor);
    int currB = (int)(B / factor);

    // 3. Define our 3 specific targets
    struct {
        uint32_t hex;
        const char* name;
    } Targets[3] = {
        {0xFF0000, "Red"},
        {0x0000FF, "Blue"},
        {0x000000, "Black"}
    };

    float minDistance = 10000.0f;
    int bestIndex = -1;

    // 4. Compare current color to only Red, Blue, and Black
    for (int i = 0; i < 3; i++) {
        int tarR = (Targets[i].hex >> 16) & 0xFF;
        int tarG = (Targets[i].hex >> 8) & 0xFF;
        int tarB = Targets[i].hex & 0xFF;

        float dist = sqrt(pow(currR - tarR, 2) +
                          pow(currG - tarG, 2) +
                          pow(currB - tarB, 2));

        if (dist < minDistance) {
            minDistance = dist;
            bestIndex = i;
        }
    }

    // 5. Final Threshold Check
    if (bestIndex != -1 && minDistance <= DISTANCE_THRESHOLD) {
        strcpy(bestMatch.name, Targets[bestIndex].name);
    } else {
        // Fallback policy: treat non-red/non-blue matches as black.
        strcpy(bestMatch.name, "Black");
    }

    bestMatch.hexValue = (currR << 16) | (currG << 8) | currB;
    return bestMatch;
}
