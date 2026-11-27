#include "gpio.h"
#include "gpio_interrupt.h"
#include "timer.h"
#include "uart.h"
#include "msa311.h"

// GPIO pin for LED
#define LED_PIN GPIO_PC1

// GPIO pin for Button
#define BUTTON GPIO_PB4

// Threshold for turning off the LED
#define ACCEL_THRESHOLD 1000

static volatile bool button_pressed = false;
static volatile bool reading_accel = false;

void handle_button_interrupt(void *aux_data) {
    button_pressed = true;
    gpio_write(LED_PIN, 1); // Turn on LED
    gpio_interrupt_clear(BUTTON);
}

void monitor_accelerometer(msa311_t *msa) {
    int x_mg, y_mg, z_mg;

    while (reading_accel) {
        if (msa311_read_acceleration(msa, &x_mg, &y_mg, &z_mg)) {
            int magnitude = msa311_calculate_magnitude(x_mg, y_mg, z_mg);

            printf("Accel magnitude: %d mg\n", magnitude);

            if (magnitude > ACCEL_THRESHOLD) {
                gpio_write(LED_PIN, 0); // Turn off LED
                reading_accel = false;
            }
        } else {
            printf("Failed to read accelerometer data\n");
        }

        timer_delay_us(100000); // 100ms delay
    }
}

void config_button(void) {
    gpio_set_input(BUTTON);
    gpio_set_pullup(BUTTON); // Use internal pullup resistor

    gpio_interrupt_init();
    gpio_interrupt_config(BUTTON, GPIO_INTERRUPT_NEGATIVE_EDGE, false);
    gpio_interrupt_register_handler(BUTTON, handle_button_interrupt, NULL);
    gpio_interrupt_enable(BUTTON);
}

void main(void) {
    uart_init();
    gpio_init();

    // Initialize LED pin
    gpio_set_output(LED_PIN);
    gpio_write(LED_PIN, 0); // Turn off LED initially

    // Configure button with interrupt
    config_button();

    // Initialize accelerometer
    msa311_t *msa = msa311_init();
    if (!msa) {
        printf("Failed to initialize accelerometer!\n");
        return;
    }

    // Enable global interrupts
    interrupts_init();
    interrupts_global_enable();

    printf("System initialized. Waiting for button press...\n");

    while (true) {
        if (button_pressed) {
            button_pressed = false;
            reading_accel = true;
            printf("Button pressed. Monitoring accelerometer...\n");
            monitor_accelerometer(msa);
        }

        // Small delay to reduce CPU usage
        timer_delay_us(10000);
    }

    msa311_free(msa);
}
