#include "pwm.h"
#include "gpio.h"
#include "timer.h" // For delays
#include "printf.h"

#include "uart.h"

int main(void) {

    timer_delay_ms(2000);     // Hold for 2 seconds

    // Initialize the PWM module
    pwm_init();
    uart_init();

    printf("system run|\n");

    // Configure PWM channel 4 (PB1) with a 50 Hz frequency (typical for servos)
    pwm_config_channel(PWM4, GPIO_PB1, 50, false);

    // Move servo to -90 degrees
    pwm_set_duty(PWM4, 6.5); // Duty cycle for approx. -90 degrees
    timer_delay_ms(10000);     // Hold for 2 seconds

    // Move servo to +85 degrees
    pwm_set_duty(PWM4, 9.5); // Duty cycle for approx. +85 degrees
    timer_delay_ms(2000);     // Hold for 2 seconds

    return 0;
}
