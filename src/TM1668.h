#pragma once

#include <stdint.h>

#define READPORT(pin_number, reg) ((nrf_gpio_port_in_read(reg) >> pin_number) & 1UL)

void setupTM1668(uint32_t dio, uint32_t sclk, uint32_t stb);

bool readTM1668(void);

uint8_t mapSegment(uint8_t);

extern void parsePayload(const char *);