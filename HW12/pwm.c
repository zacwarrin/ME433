
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define pwm_pin 16 // the built in LED on the Pico

int main() {
gpio_set_function(pwm_pin, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
uint slice_num = pwm_gpio_to_slice_num(pwm_pin); // Get PWM slice number
float div = 40; // must be between 1-255
pwm_set_clkdiv(slice_num, div); // divider
uint16_t wrap = 62500; // when to rollover, must be less than 65535
pwm_set_wrap(slice_num, wrap);
pwm_set_enabled(slice_num, true); // turn on the PWM



while(true){
    pwm_set_gpio_level(pwm_pin, wrap / 40); // set the duty cycle to 2.5% (0 degrres)
    sleep_ms(2000);
    pwm_set_gpio_level(pwm_pin, wrap / 8);
    sleep_ms(2000);
}

}

