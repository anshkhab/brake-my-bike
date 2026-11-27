#include "pwm.h"
#include "gpio.h"
#include "timer.h" // For delays
#include "printf.h"

int main(void) {
    // Initialize the PWM module
    pwm_init();

    // Configure PWM channel 4 (PB1) with a 50 Hz frequency (typical for servos)
    pwm_config_channel(PWM4, GPIO_PB1, 50, false);

    // Move servo to -90 degrees
    pwm_set_duty(PWM4, 6.5); // Duty cycle for approx. -90 degrees
    timer_delay_ms(2000);     // Hold for 2 seconds

    return 0;
}
