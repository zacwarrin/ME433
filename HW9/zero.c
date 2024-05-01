/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h> // for memset
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "ssd1306.h"
#include "font.h"

#define LED_PIN 25

// UART
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// static int chars_rxed = 0;

char message[50]; 
int i_count = 0;
// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        if (ch=='\n'){
            message[i_count] = 0;
            i_count = 0;
            printf("\nFrom Zero: %s\n",message);
        }
        else{
            message[i_count] = ch;
            i_count++;
        }
        // // Can we send it back?
        // if (uart_is_writable(UART_ID)) {
        //     // Change it slightly first!
        //     //ch++;
        //     uart_putc(UART_ID, ch);
        // }
        // chars_rxed++;
    }
}

int main() {
    stdio_init_all();

    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    printf("Start\n");

    while (1){
        int ifromcomp = 0;
        scanf("%d",&ifromcomp);
        printf("Comp sent: %d",ifromcomp);

        char txm[100];
        sprintf(txm, "%d\n", ifromcomp);
        uart_puts(UART_ID, txm);

        sleep_ms(250);
    }
    
}

/// \end:uart_advanced[]
