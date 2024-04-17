#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

 /* Example code to talk to a BMP280 temperature and pressure sensor

    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.

    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.

    Connections on Raspberry Pi Pico board, other boards may vary.

    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on BMP280
    board
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP5 (pin 7)) -> SCL on
    BMP280 board
    3.3v (pin 36) -> VCC on BMP280 board
    GND (pin 38)  -> GND on BMP280 board
 */

 // device has default bus address of 0x76
#define ADDR _u(0b0100000)

// hardware registers
#define REG_IODIR _u(0x00)
#define REG_GPIO _u(0x09)
#define REG_OLAT _u(0x0A)

void init() {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];
    // send register number followed by its corresponding value
    buf[0] = REG_IODIR;
    buf[1] = 0b01111111;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

}

void set(char v) {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];
    // send register number followed by its corresponding value
    buf[0] = REG_OLAT;
    buf[1] = v<<7;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

}

void bmp280_init() {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];
    // send register number followed by its corresponding value
    buf[0] = REG_IODIR;
    buf[1] = 0b011111111;
    i2c_write_blocking(i2c_default, ADDR, buf, 2, false);

}

int read() {

    uint8_t buf[1];
    uint8_t reg = REG_GPIO;
    i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(i2c_default, ADDR, buf, 1, false);  // false - finished with bus

    if ((buf[0] & 0b1) == 0b1){
        return 1;
    }
    else{
        return 0;
    }
}
#define LED_PIN 25

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);



    // configure BMP280
    init();

    int count = 0;

    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (1) {
        // blink gp25

        count++;

        if (count < 50){
            gpio_put(LED_PIN, 1);
        }
        else if (count < 100){
            gpio_put(LED_PIN, 0);
        }
        else{
            count = 0;
        }

        sleep_ms(10);        
        if (read() == 1){ // if gp0 is high
            set(0); // set gp7 high
        }
        else{
            set(1); // set gp7 low
        }
        
        sleep_ms(10);
        
    }

}
