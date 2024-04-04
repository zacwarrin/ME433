#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Initializing Pins\n");
    
    gpio_init(16); // set pin 21 as I/O
    gpio_set_dir(16, GPIO_OUT); // Output pin
    gpio_put(16, 1); // Turn LED on

    while (gpio_get(15) == 0){
    }

    gpio_put(16, 0); // Turn LED off

    adc_init(); // init the adc module
    adc_gpio_init(26); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(0); // select to read from ADC0
    
    while (1) {
        printf("Enter Number of samples to take: \n");
        char cycles[100];
        scanf("%d", cycles);
        // printf("message: %s\r\n",message);
        // sleep_ms(50);
        // uint16_t result = adc_read();
        // printf("val: %d\r\n",result);
        for (int i=0; i<*cycles; i++){
            uint16_t result = adc_read();
            float converted = (float)result/1212.0;
            printf("%d: %f V\n",i,converted);
            sleep_ms(10);
        }
    }
}