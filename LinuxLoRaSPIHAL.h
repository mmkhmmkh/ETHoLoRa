//
// Created by MMKH on 10/24/2021.
//

#ifndef SPIBRIDGE_LINUXLORASPIHAL_H
#define SPIBRIDGE_LINUXLORASPIHAL_H

#include <linux/unistd.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

void HAL_Init(uint16_t bus);
void HAL_End(void);
void HAL_Delay(uint32_t millis);
void HAL_GPIO_FreePin(uint8_t pin);
void HAL_GPIO_FreeIRQ(uint8_t pin);
void HAL_GPIO_SetupPin(uint8_t pin, uint8_t isOut);
void HAL_GPIO_WritePin(uint8_t pin, uint8_t value);
void HAL_GPIO_IRQPin(uint8_t pin, irq_handler_t handler);

void HAL_SPI_Transmit(const uint8_t* data, uint16_t length);
void HAL_SPI_Receive(const uint8_t* data, uint16_t length);

#endif //SPIBRIDGE_LINUXLORASPIHAL_H
