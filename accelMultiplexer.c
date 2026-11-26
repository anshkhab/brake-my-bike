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
#include "malloc.h"
#include "printf.h"
#include "assert.h"
#include "uart.h"

/* Multiplexer control pins */
#define MUX_SELECT_PIN_1 PE17  // Selector A (Pin 14 of DM74LS153)
#define MUX_SELECT_PIN_2 PE16  // Selector B (Pin 15 of DM74LS153)

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
#define REG_POWER_MODE  0x11
#define REG_BANDWIDTH   0x12
#define REG_RESOLUTION  0x13

/* Configuration Values */
#define FS_2G           0x00
#define FS_4G           0x01
#define FS_8G           0x02
#define FS_16G          0x03

#define ODR_125HZ       0x07
#define POWER_NORMAL    0x00
#define BANDWIDTH_125HZ 0x07
#define RESOLUTION_14   0x01

/* Expected Device ID for MSA311 */
#define EXPECTED_PART_ID 0x13

/* Accelerometer Device Structure */
typedef struct {
    i2c_device_t *i2c_dev;  // I2C device handle
    int range;              // Current accelerometer range in mg (2000, 4000, etc.)
} msa311_t;

/* Function Prototypes */
static bool set_range(msa311_t *msa, uint8_t range);
static bool set_data_rate(msa311_t *msa, uint8_t data_rate);
static bool set_power_mode(msa311_t *msa, uint8_t power_mode);
static bool set_bandwidth(msa311_t *msa, uint8_t bandwidth);
static bool set_resolution(msa311_t *msa, uint8_t resolution);

bool msa311_read_raw(msa311_t *msa, int16_t *x_raw, int16_t *y_raw, int16_t *z_raw);
bool msa311_read_acceleration(msa311_t *msa, int *x_mg, int *y_mg, int *z_mg);
int msa311_calculate_magnitude(int x_mg, int y_mg, int z_mg);
void msa311_free(msa311_t *msa);

/* Multiplexer Channel Selection */
void mux_select(uint8_t channel) {
    gpio_write(MUX_SELECT_PIN_1, channel & 0x01);
    gpio_write(MUX_SELECT_PIN_2, (channel >> 1) & 0x01);
}

/* Initialize MSA311 Accelerometers via Multiplexer */
msa311_t *msa311_init(void) {
    gpio_set_function(GPIO_PG13, GPIO_FN_ALT3); // SDA
    gpio_set_function(GPIO_PG12, GPIO_FN_ALT3); // SCL

    // Initialize I2C
    i2c_init();

    // Configure multiplexer select pins as output
    gpio_set_output(MUX_SELECT_PIN_1); // Multiplexer Selector A
    gpio_set_output(MUX_SELECT_PIN_2); // Multiplexer Selector B

    msa311_t *msa = malloc(sizeof(msa311_t));
    if (!msa) return NULL;

    msa->i2c_dev = i2c_new(MSA311_ADDRESS);
    if (!msa->i2c_dev) {
        free(msa);
        return NULL;
    }

    // Initialize and configure each accelerometer
    for (int channel = 0; channel < 2; channel++) {
        mux_select(channel); // Select the current multiplexer channel
        timer_delay_us(1000); // Allow time for switching

        // Verify Device ID
        uint8_t part_id = i2c_read_reg(msa->i2c_dev, REG_PART_ID);
        if (part_id != EXPECTED_PART_ID) {
            printf("Accelerometer on channel %d ID mismatch! Expected 0x%x, got 0x%x\n",
                   channel, EXPECTED_PART_ID, part_id);
            free(msa);
            return NULL;
        }

        // Apply default configurations
        set_range(msa, FS_4G);
        set_data_rate(msa, ODR_125HZ);
        set_power_mode(msa, POWER_NORMAL);
        set_bandwidth(msa, BANDWIDTH_125HZ);
        set_resolution(msa, RESOLUTION_14);

        printf("Accelerometer %d initialized successfully.\n", channel + 1);
    }

    msa->range = 4000; // Default to ±4G range
    return msa;
}

/* Configuration Functions */
static bool set_range(msa311_t *msa, uint8_t range) {
    assert(range == FS_2G || range == FS_4G || range == FS_8G || range == FS_16G);

    // Write range value to register
    if (!i2c_write_reg(msa->i2c_dev, REG_FS_RANGE, range)) {
        assert(false && "Failed to set range: I2C write error");
        return false;
    }

    // Map range value to corresponding milli-g range
    msa->range = (range == FS_2G) ? 2000 :
                 (range == FS_4G) ? 4000 :
                 (range == FS_8G) ? 8000 : 16000;

    // Add delay for sensor configuration
    timer_delay_us(100);
    return true;
}

static bool set_data_rate(msa311_t *msa, uint8_t data_rate) {
    assert(data_rate <= 0x0F); // Valid data rate values are 0x00 to 0x0F

    if (!i2c_write_reg(msa->i2c_dev, REG_ODR, data_rate)) {
        assert(false && "Failed to set data rate: I2C write error");
        return false;
    }

    // Add delay for sensor configuration
    timer_delay_us(100);
    return true;
}

static bool set_power_mode(msa311_t *msa, uint8_t power_mode) {
    assert(power_mode == POWER_NORMAL); // Only supporting POWER_NORMAL for now

    if (!i2c_write_reg(msa->i2c_dev, REG_POWER_MODE, power_mode)) {
        assert(false && "Failed to set power mode: I2C write error");
        return false;
    }

    // Add delay for sensor configuration
    timer_delay_us(100);
    return true;
}

static bool set_bandwidth(msa311_t *msa, uint8_t bandwidth) {
    assert(bandwidth <= 0x0F); // Valid bandwidth values are 0x00 to 0x0F

    if (!i2c_write_reg(msa->i2c_dev, REG_BANDWIDTH, bandwidth)) {
        assert(false && "Failed to set bandwidth: I2C write error");
        return false;
    }

    // Add delay for sensor configuration
    timer_delay_us(100);
    return true;
}

static bool set_resolution(msa311_t *msa, uint8_t resolution) {
    assert(resolution == RESOLUTION_14); // Only supporting RESOLUTION_14 for now

    if (!i2c_write_reg(msa->i2c_dev, REG_RESOLUTION, resolution)) {
        assert(false && "Failed to set resolution: I2C write error");
        return false;
    }

    // Add delay for sensor configuration
    timer_delay_us(100);
    return true;
}

bool msa311_read_raw(msa311_t *msa, int16_t *x_raw, int16_t *y_raw, int16_t *z_raw) {
    uint8_t data[6];

    // Read 6 bytes from the accelerometer starting from the X-axis LSB register
    if (!i2c_read_reg_n(msa->i2c_dev, REG_ACC_X_LSB, data, 6)) {
        printf("Error: Failed to read raw accelerometer data.\n");
        return false;
    }

    // Combine MSB and LSB for each axis (12-bit data)
    *x_raw = ((data[1] << 4) | (data[0] & 0x0F));
    *y_raw = ((data[3] << 4) | (data[2] & 0x0F));
    *z_raw = ((data[5] << 4) | (data[4] & 0x0F));

    // Perform 12-bit sign extension for negative values
    if (*x_raw & 0x0800) *x_raw |= 0xF000; // Sign-extend bit 11
    if (*y_raw & 0x0800) *y_raw |= 0xF000;
    if (*z_raw & 0x0800) *z_raw |= 0xF000;

    return true;
}

bool msa311_read_acceleration(msa311_t *msa, int *x_mg, int *y_mg, int *z_mg) {
    int16_t x_raw, y_raw, z_raw;

    // Read raw accelerometer data
    if (!msa311_read_raw(msa, &x_raw, &y_raw, &z_raw)) {
        printf("Error: Failed to read acceleration data.\n");
        return false;
    }

    // Convert raw data to milli-g values
    *x_mg = (x_raw * msa->range) / 1024;
    *y_mg = (y_raw * msa->range) / 1024;
    *z_mg = (z_raw * msa->range) / 1024;

    return true;
}

/*
float simple_sqrtf(float number) {
    if (number < 0) {
        return -1; // Error: negative input
    }

    float x = number;
    float y = 1.0f;
    float epsilon = 0.00001f; // Convergence threshold

    while (x - y > epsilon) {
        x = (x + y) / 2;
        y = number / x;
    }
    return x;
}

int msa311_calculate_magnitude(int x_mg, int y_mg, int z_mg) {
    float magnitude = simple_sqrtf((x_mg * x_mg) + (y_mg * y_mg) + (z_mg * z_mg)) / 1000.0;
    // Convert magnitude to a scaled integer for two decimal places
    int magnitude_scaled = (int)(magnitude * 100);
    return magnitude_scaled;
}
*/

int custom_abs(int value) {
    // Returns the absolute value of an integer
    return (value < 0) ? -value : value;
}

float custom_fabsf(float value) {
    // Returns the absolute value of a float
    return (value < 0.0f) ? -value : value;
}

float custom_atan2(float y, float x) {
    // Custom atan2 implementation to handle edge cases
    if (x == 0.0f) {
        if (y == 0.0f) {
            return 0.0f; // Both x and y are zero, theta is undefined; return 0 as a fallback
        }
        return (y > 0.0f) ? 1.5708f : -1.5708f; // ±π/2 for vertical line
    }

    float atan;
    float z = y / x;

    if (custom_fabsf(z) < 1.0f) {
        atan = z / (1.0f + 0.28f * z * z);
        if (x < 0.0f) {
            return (y < 0.0f) ? atan - 3.14159f : atan + 3.14159f; // Adjust for ±π
        }
    } else {
        atan = 1.5708f - z / (z * z + 0.28f); // π/2 - approximation
        if (y < 0.0f) {
            return atan - 3.14159f; // Adjust for 3rd/4th quadrant
        }
    }
    return atan;
}

float calculate_theta(int x_mg, int z_mg) {
    // Handle cases where both x and z are zero
    if (x_mg == 0 && z_mg == 0) {
        printf("Warning: Both X and Z readings are zero. Theta is undefined.\n");
        return 0.0f; // Default to 0 degrees
    }

    // Calculate theta in radians using custom atan2
    float theta_rad = custom_atan2((float)z_mg, (float)x_mg);

    // Convert to degrees
    float theta_deg = theta_rad * (180.0f / 3.14159f);
    return theta_deg;
}

/* Free MSA311 Resources */
void msa311_free(msa311_t *msa) {
    if (!msa) return;
    i2c_free(msa->i2c_dev);
    free(msa);
}

int main(void) {
    uart_init();

    msa311_t *msa = msa311_init();
    if (!msa) {
        printf("MSA311 initialization failed!\n");
        return -1;
    }

    int x_mg, y_mg, z_mg;

    while (1) {
        for (int i = 0; i < 2; i++) {
            mux_select(i); // Switch between accelerometers
            timer_delay_us(500); // Allow for stabilization

            if (msa311_read_acceleration(msa, &x_mg, &y_mg, &z_mg)) {
                float theta = calculate_theta(x_mg, z_mg);
                printf("Accelerometer %d: Theta: %.2f degrees | X: %d mg, Y: %d mg, Z: %d mg\n", 
                       i + 1, theta, x_mg, y_mg, z_mg);
            } else {
                printf("Warning: Failed to read acceleration data from Accelerometer %d\n", i + 1);
            }
        }
        timer_delay_us(100000); // Delay 100ms
    }

    msa311_free(msa);
    return 0;
}
