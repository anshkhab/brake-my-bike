/* File: msa311.c
 * -----------------
 * MSA311 library for MangoPi.
 * 
 * Provides initialization, configuration, and data reading functionalities
 * for the MSA311 accelerometer using I2C communication.
 *
 * Author: Zeynep Eylül Yağcıoğlu
 */

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "gpio.h"
#include "timer.h"
#include "assert.h"
#include "ccu.h"
#include "malloc.h"
#include <stddef.h>
#include "strings.h"
#include "printf.h"

/* MSA311 I2C Address */
#define MSA311_ADDRESS 0x62

/* Register Definitions */
#define REG_SOFT_RESET  0x00
#define REG_PART_ID     0x01
#define REG_ACC_X_LSB   0x02
#define REG_ACC_Y_LSB   0x04
#define REG_ACC_Z_LSB   0x06
#define REG_FS_RANGE    0x0F
#define REG_ODR         0x10

/* Configuration Values */
#define FS_2G           0x00
#define FS_4G           0x01
#define FS_8G           0x02
#define FS_16G          0x03

#define ODR_125HZ       0x07

/* Expected Device ID for MSA311 */
#define EXPECTED_PART_ID 0x13 // Replace with the actual value if different

/* Accelerometer Device Structure */
typedef struct {
    i2c_device_t *i2c_dev;  // I2C device handle
    int range;              // Current accelerometer range in mg (2000, 4000, etc.)
} msa311_t;

/* Initialize MSA311 Accelerometer */
msa311_t *msa311_init(void) {
    gpio_set_function(GPIO_PG13, GPIO_FN_ALT3); // SDA
    gpio_set_function(GPIO_PG12, GPIO_FN_ALT3); // SCL

    i2c_init();

    msa311_t *msa = malloc(sizeof(msa311_t));
    if (!msa) return NULL;

    msa->i2c_dev = i2c_new(MSA311_ADDRESS);
    if (!msa->i2c_dev) {
        free(msa);
        return NULL;
    }

    // Verify Device ID
    uint8_t part_id = i2c_read_reg(msa->i2c_dev, REG_PART_ID);
    if (part_id != EXPECTED_PART_ID) {
        printf("MSA311 ID mismatch! Expected 0x%x, got 0x%x\n", EXPECTED_PART_ID, part_id);
        free(msa);
        return NULL;
    }

    // Soft reset
    i2c_write_reg(msa->i2c_dev, REG_SOFT_RESET, 0x01);
    timer_delay_us(1000);

    // Set range to ±2G and store range in mg
    i2c_write_reg(msa->i2c_dev, REG_FS_RANGE, FS_2G);
    msa->range = 2000; // 2G = 2000 mg

    // Set data rate to 125 Hz
    i2c_write_reg(msa->i2c_dev, REG_ODR, ODR_125HZ);

        /* Power Mode Configuration */
    #define REG_POWER_MODE  0x11 // Replace with actual register for power mode
    #define POWER_NORMAL    0x00
    i2c_write_reg(msa->i2c_dev, REG_POWER_MODE, POWER_NORMAL);

    /* Bandwidth Configuration */
    #define REG_BANDWIDTH   0x12 // Replace with actual register for bandwidth
    #define BANDWIDTH_125HZ 0x07
    i2c_write_reg(msa->i2c_dev, REG_BANDWIDTH, BANDWIDTH_125HZ);

    /* Resolution Configuration */
    #define REG_RESOLUTION  0x13 // Replace with actual register for resolution
    #define RESOLUTION_14   0x01
    i2c_write_reg(msa->i2c_dev, REG_RESOLUTION, RESOLUTION_14);


    return msa;
}

/* Set the acceleration range */
bool msa311_set_range(msa311_t *msa, int range) {
    uint8_t reg_value;
    switch (range) {
        case 2000: reg_value = FS_2G; break;
        case 4000: reg_value = FS_4G; break;
        case 8000: reg_value = FS_8G; break;
        case 16000: reg_value = FS_16G; break;
        default: return false;
    }

    if (!i2c_write_reg(msa->i2c_dev, REG_FS_RANGE, reg_value)) return false;
    msa->range = range; // Store range in mg
    return true;
}

bool msa311_read_acceleration(msa311_t *msa, int *x_mg, int *y_mg, int *z_mg) {
    uint8_t data[6];

    // Read acceleration data registers
    if (!i2c_read_reg_n(msa->i2c_dev, REG_ACC_X_LSB, data, 6)) {
        printf("I2C read failed.\n");
        return false;
    }

    // Log raw data
    printf("Raw data read: %02x %02x %02x %02x %02x %02x\n", 
           data[0], data[1], data[2], data[3], data[4], data[5]);

    // Combine MSB and LSB
    int16_t raw_x = ((data[1] << 4) | (data[0] & 0x0F));
    int16_t raw_y = ((data[3] << 4) | (data[2] & 0x0F));
    int16_t raw_z = ((data[5] << 4) | (data[4] & 0x0F));

    // Sign-extend 12-bit values
    raw_x = (raw_x & 0x0800) ? (raw_x | 0xF000) : raw_x;
    raw_y = (raw_y & 0x0800) ? (raw_y | 0xF000) : raw_y;
    raw_z = (raw_z & 0x0800) ? (raw_z | 0xF000) : raw_z;

    // Log raw 12-bit values
    printf("Raw 12-bit values: X=%d, Y=%d, Z=%d\n", raw_x, raw_y, raw_z);

    // Convert raw data to mg
    *x_mg = (raw_x * msa->range) / 1024;
    *y_mg = (raw_y * msa->range) / 1024;
    *z_mg = (raw_z * msa->range) / 1024;

    // Log converted values
    printf("Converted -> X: %d mg, Y: %d mg, Z: %d mg\n", *x_mg, *y_mg, *z_mg);

    // Delay to allow sensor update
    timer_delay_ms(10);

    return true;
}


/* Free MSA311 resources */
void msa311_free(msa311_t *msa) {
    if (!msa) return;
    i2c_free(msa->i2c_dev);
    free(msa);
}

/* Main program */
int main(void) {
    // Initialize UART for debugging
    uart_init();

    // Initialize the accelerometer
    msa311_t *msa = msa311_init();
    if (!msa) {
        printf("MSA311 initialization failed!\n");
        return -1;
    }

    // Set range to ±4G
    if (!msa311_set_range(msa, 4000)) {
        printf("Failed to set range!\n");
        msa311_free(msa);
        return -1;
    }

    int x_mg = 0, y_mg = 0, z_mg = 0; // Variables to store acceleration in mg

    printf("Starting acceleration readings...\n");

    while (1) {
        // Read acceleration data
        if (msa311_read_acceleration(msa, &x_mg, &y_mg, &z_mg)) {
            printf("Acceleration -> X: %d mg, Y: %d mg, Z: %d mg\n", x_mg, y_mg, z_mg);
        } else {
            printf("Failed to read acceleration data\n");
        }
        timer_delay_us(100000); // Delay 100ms
    }

    msa311_free(msa);
    return 0;
}



