//
// Created by wine on 24-10-24.
//

#include "u8g2.h"
#include "main.h"
#include "i2c.h"

#ifndef UCPD_U8G2_STM32_H
#define UCPD_U8G2_STM32_H

#endif //UCPD_U8G2_STM32_H

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void delay_us(uint32_t time);
void u8g2Init(u8g2_t *u8g2);
