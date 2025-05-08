#pragma once

// define SERIAL_DEBUG_ENABLED

#ifdef SERIAL_DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)
#endif

#define SCALES_NAME "Tencent Scales"
#define UUID16_SVC_SCALES 0xFFF0
#define UUID16_CHR_SCALES_READ 0x36F5
#define UUID16_CHR_SCALES_WRITE 0xFFF4

#define DIO 25   // P0.13
#define SCLK 24  // P0.15
#define STB 1    // P0.24
#define BUT_P 19 // P0.03
#define BUT_T 17 // P0.28
#define LED_L 18 // P0.02
#define LED_R 4  // P1.10

#define STB_PIN ((uint32_t)24)  // STB
#define DIO_PIN ((uint32_t)13)  // DIO
#define SCLK_PIN ((uint32_t)15) // SCLK

#define BUFFER_LENGTH 16