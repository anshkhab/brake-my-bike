#include "gpio.h"
#include "gpio_interrupt.h"
#include "timer.h"
#include "uart.h"
#include "msa311.h"

// GPIO pin for LED
#define LED_PIN GPIO_PC1

// GPIO pin for Button
#define BUTTON GPIO_PB4

static volatile bool button_pressed = false;
static volatile bool reading_accel = false;

void handle_button_interrupt(void *aux_data) {
    button_pressed = true;
    gpio_interrupt_clear(BUTTON);
}

void monitor_accelerometer(msa311_t *msa) {
    int x_mg, y_mg, z_mg;
    int negative_theta_count = 0;
    int total_readings = 0;

    while (reading_accel) {
        if (msa311_read_acceleration(msa, &x_mg, &y_mg, &z_mg)) {
            float theta = calculate_theta(x_mg, z_mg);
            int theta_int = (int)(theta * 100); // Scale to two decimal places

            printf("Theta: %d.%02d degrees | Accel (mg) -> X: %d, Y: %d, Z: %d\n", 
                   theta_int / 100, custom_abs(theta_int % 100), x_mg, y_mg, z_mg);

            if (theta < 0) {
                negative_theta_count++;
            }
            total_readings++;

            if (total_readings >= 15) {
                if (negative_theta_count >= 10) {
                    gpio_write(LED_PIN, 0); // Turn off LED
                    reading_accel = false;
                }
                // Reset counters for the next batch of readings
                negative_theta_count = 0;
                total_readings = 0;
            }
        } else {
            printf("Failed to read accelerometer data\n");
        }

        timer_delay_us(100000); // 100ms delay
    }
}

void config_button(void) {
    gpio_set_input(BUTTON);
    gpio_set_function(BUTTON, GPIO_FN_INPUT);  // Use internal pullup resistor

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

            if (!reading_accel) {
                gpio_write(LED_PIN, 1); // Turn on LED
                reading_accel = true;
                printf("Button pressed. Monitoring accelerometer...\n");
                monitor_accelerometer(msa);
            }
        }

        // Small delay to reduce CPU usage
        timer_delay_us(10000);
    }

    msa311_free(msa);
}
