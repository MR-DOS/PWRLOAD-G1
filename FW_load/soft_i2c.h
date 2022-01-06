#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H
#include <stdint.h>

void softiic_init(void);
uint8_t receive(uint8_t ack);
uint8_t transmit(uint8_t d);
void sendstart(void);
void sendstop(void);

#endif
