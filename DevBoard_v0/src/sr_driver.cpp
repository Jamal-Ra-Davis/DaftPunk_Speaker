#include <Arduino.h>
#include "sr_driver.h"
#include "global_defines.h"

inline void sr_write_byte(uint8_t val)
{ 
  char buf[64];
  for (int i=7; i>=0; i--) {
    digitalWrite(SR_CLK_PIN, LOW);
    bool bit_ = (bool)((1 << i) & val);
    if (bit_)
      digitalWrite(SR_DIN_PIN, HIGH);
    else
      digitalWrite(SR_DIN_PIN, LOW);
    
    digitalWrite(SR_CLK_PIN, HIGH);
  }
}
void sr_write(uint8_t *data, int N)
{
  digitalWrite(SR_LAT_PIN, LOW);
  
  for (int i=0; i<N; i++) {
    sr_write_byte(data[i]);
  }
  digitalWrite(SR_LAT_PIN, HIGH);
}
void init_shift_registers()
{
  pinMode(SR_DIN_PIN, OUTPUT);
  pinMode(SR_CLK_PIN, OUTPUT);
  pinMode(SR_LAT_PIN, OUTPUT);

  digitalWrite(SR_DIN_PIN, LOW);
  digitalWrite(SR_CLK_PIN, LOW);
  digitalWrite(SR_LAT_PIN, LOW);

  uint8_t clear_data[SR_CNT] = {
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
  sr_write(clear_data, SR_CNT);
}