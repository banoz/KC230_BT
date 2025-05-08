#include <Arduino.h>

#include <nrf_bitmask.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>

#include "main.h"
#include "TM1668.h"

uint32_t pinDIO = 0;
uint32_t pinSCLK = 0;
uint32_t pinSTB = 0;

volatile bool buffered = false;

volatile uint16_t TM1668state = 0xFFFFU; // {counter}{data}

char TM1668Buffer[BUFFER_LENGTH] = {0};
volatile uint16_t TM1668BufferIndex = 0;

void IRQHandler(void);

void setupTM1668(uint32_t dio, uint32_t sclk, uint32_t stb)
{
    pinDIO = g_ADigitalPinMap[dio];
    pinSCLK = g_ADigitalPinMap[sclk];
    pinSTB = g_ADigitalPinMap[stb];

    attachInterrupt(sclk, IRQHandler, RISING);
    attachInterrupt(stb, IRQHandler, CHANGE);
}

bool readTM1668(void)
{
    if (buffered)
    {
        parsePayload(TM1668Buffer);

        for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
        {
            TM1668Buffer[i] = 0;
        }

        buffered = false;
        return true;
    }

    return false;
}

void pushBuffer(uint8_t data)
{
    if (!buffered)
    {
        TM1668Buffer[TM1668BufferIndex++] = data;

        if (TM1668BufferIndex >= BUFFER_LENGTH)
        {
            buffered = true;
            TM1668BufferIndex = 0;
            TM1668state = 0xFFFFU;
        }
    }
}

typedef enum
{
    FLAG_NONE = 0,
    FLAG_STB_HIGH = 1 << 0, // 0x01
    FLAG_STB_LOW = 1 << 1,  // 0x02
    FLAG_DIN_HIGH = 1 << 2, // 0x04
    FLAG_DIN_LOW = 1 << 3   // 0x08
} EventFlags;

inline void processEvent(uint8_t event)
{
    if (buffered)
    {
        TM1668state = 0xFFFFU;
        return; // ignore events until buffer is reset
    }

    if (event & FLAG_STB_LOW) // start
    {
        TM1668state = 0;
        TM1668BufferIndex = 0;
        return;
    }

    if (event & FLAG_STB_HIGH) // stop
    {
        pushBuffer(0xFFu); // fill up the buffer to 16 bytes
        return;
    }

    if (TM1668state == 0xFFFFU)
    {
        return; // ignore events until start
    }

    if (event & FLAG_DIN_HIGH || event & FLAG_DIN_LOW) // CLK
    {
        uint8_t counter = TM1668state >> 8;

        if (event & FLAG_DIN_HIGH)
        {
            TM1668state |= (1U << counter);
        }

        TM1668state = (TM1668state & 0xFFu) | ((++counter) << 8);

        if (counter >= 8)
        {
            pushBuffer(TM1668state & 0xFFu);

            TM1668state = 0;
        }

        return;
    }
}

// extern "C" void GPIOTE_IRQHandler()
void IRQHandler(void)
{
    uint8_t event = 0;

    // {DIN LOW}{DIN HIGH}{STB LOW}{STB HIGH}

    if (0 != NRF_GPIOTE->EVENTS_IN[0]) // pinSCLK LOTOHI
    {
        READPORT(pinDIO, NRF_P0) ? (event |= FLAG_DIN_HIGH) : (event |= FLAG_DIN_LOW);

        NRF_GPIOTE->EVENTS_IN[0] = 0;
    }

    if (0 != NRF_GPIOTE->EVENTS_IN[1]) // pinSTB TOGGLE
    {
        READPORT(pinSTB, NRF_P0) ? (event |= FLAG_STB_HIGH) : (event |= FLAG_STB_LOW);

        NRF_GPIOTE->EVENTS_IN[1] = 0;
    }

    if (event > 0)
    {
        processEvent(event);
    }
}

uint8_t mapSegment(uint8_t segmentValue)
{
    switch (segmentValue)
    {
    case 0x00:
    case 0x40:
    case 0x3F:
        return 0;
    case 0x06:
        return 1;
    case 0x5B:
        return 2;
    case 0x4F:
        return 3;
    case 0x66:
        return 4;
    case 0x6D:
        return 5;
    case 0x7D:
        return 6;
    case 0x07:
        return 7;
    case 0x7F:
        return 8;
    case 0x6F:
        return 9;
    default:
        return 0xFF; // invalid segment value
    }
}