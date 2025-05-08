#include <Arduino.h>

#include "main.h"
#include "myble.h"
#include "TM1668.h"

bool weightChanged, timeChanged;
int16_t previousWeight, currentWeight;
int16_t previousTime, currentTime;

void setup(void)
{

#ifdef SERIAL_DEBUG_ENABLED
  Serial.begin(921600);
  while (!Serial)
    delay(10); // for nrf52840 with native usb
  // delay(10000);
  DEBUG_PRINTLN("Starting...");
#endif

  pinMode(DIO, INPUT);
  pinMode(SCLK, INPUT);
  pinMode(STB, INPUT);
  pinMode(BUT_P, INPUT);
  pinMode(BUT_T, INPUT);
  pinMode(LED_L, OUTPUT_D0H1);
  pinMode(LED_R, OUTPUT_D0H1);
  pinMode(PIN_VBAT, INPUT);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_L, LOW);

  setupTM1668(DIO, SCLK, STB);

  setup_ble();
}

uint32_t weightUpdateMillis = 0UL;
uint32_t inTareUpdateMillis = 0UL;
uint32_t batteryUpdateMillis = 0UL;

uint32_t lastDataUpdate = 0UL;

void loop()
{
  // digitalWrite(LED_L, HIGH);

  if (readTM1668())
  {
    lastDataUpdate = millis();
  }

  if (isInTare())
  {
    if (inTareUpdateMillis == 0UL)
    {
      pinMode(BUT_T, OUTPUT_S0D1);
      digitalWrite(BUT_T, LOW);
      inTareUpdateMillis = millis() + 100UL;
    }
    else if (inTareUpdateMillis < millis())
    {
      digitalWrite(BUT_T, HIGH);
      pinMode(BUT_T, INPUT);
      notifyTareDone();
    }
  }
  else
  {
    inTareUpdateMillis = 0UL;
  }

  if (isNotifyEnabled())
  {
    if (weightUpdateMillis < millis())
    {
      int16_t weight = currentWeight;

      if (weight < INT16_MAX)
      {
        notifyWeight(weight);
      }

      weightUpdateMillis = millis() + 100UL;
    }
  }
  else
  {
    weightUpdateMillis = 0;
  }

  if (isConnected())
  {
    if (batteryUpdateMillis < millis())
    {
      uint32_t battery = analogRead(PIN_VBAT);

      setBattery((battery - 666) / 2.34);

      batteryUpdateMillis = millis() + 1000UL;
    }
  }

  if (millis() > (lastDataUpdate + 3000))
  {
    if (isConnected())
    {
      disconnect();
    }
    else
    {
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_R, LOW);
      // this code will change SPIS pin setup so it is expected for the MCU to go into a reset after the wake up
      (void)nrf_gpio_pin_read(SCLK_PIN);
      if (nrf_gpio_pin_read(SCLK_PIN))
      {
        nrf_gpio_cfg_sense_set(SCLK_PIN, NRF_GPIO_PIN_SENSE_LOW);
      }
      else
      {
        nrf_gpio_cfg_sense_set(SCLK_PIN, NRF_GPIO_PIN_SENSE_HIGH);
      }
      if (sd_power_system_off() == NRF_SUCCESS)
      {
        while (1)
          ;
      }
    }
  }
}

void parsePayload(const char payload[])
{
  uint16_t seg[7];

  /* 1. Re-assemble the seven 16-bit segment masks (low byte first). */
  for (int s = 0; s < 7; s++)
  {
    seg[s] = (uint16_t)payload[(s * 2) + 1] | (((uint16_t)payload[(s * 2) + 2]) << 8);
  }

  char segments[16] = {0};

  /* 2. Build the per-digit bytes. */
  for (int d = 0; d < 16; d++) /* digit index / bit position */
  {
    uint8_t digit_byte = 0;

    for (int s = 0; s < 7; s++) /* segment index a-->g */
    {
      if (seg[s] & (1u << d))
      {
        digit_byte |= (1u << s); /* place that segment bit */
      }
    }

    segments[d] = digit_byte; /* DP (bit 7) left at 0 */
  }

  segments[0] = mapSegment(segments[0]);
  segments[1] = mapSegment(segments[1]);
  segments[2] = mapSegment(segments[2]);
  segments[5] = mapSegment(segments[5]);
  segments[6] = mapSegment(segments[6]);
  segments[7] = mapSegment(segments[7]);
  segments[8] = mapSegment(segments[8]);

  for (uint8_t i = 0; i < 16; i++)
  {
    DEBUG_PRINT((uint8_t)segments[i]);
    DEBUG_PRINT(" ");
  }

  bool g = (segments[9] & 0x10) != 0;
  bool oz = (segments[9] & 0x40) != 0;
  bool ml = (segments[9] & 0x20) != 0;

  int16_t time = segments[0] * 60 + segments[1] * 10 + segments[2];
  DEBUG_PRINT(" T:");
  DEBUG_PRINT(time);

  int16_t weight = segments[5] * 1000 + segments[6] * 100 + segments[7] * 10 + segments[8];
  DEBUG_PRINT(" W:");
  DEBUG_PRINT(weight);

  if (segments[4] & 0x40)
  {
    weight = -weight;
  }

  DEBUG_PRINT(" W2:");
  DEBUG_PRINT(weight);

  currentTime = time;
  if (g)
  {
    currentWeight = weight;
  }

  if (previousWeight != currentWeight)
  {
    weightChanged = true;
  }

  if (previousTime != currentTime)
  {
    timeChanged = true;
  }
}