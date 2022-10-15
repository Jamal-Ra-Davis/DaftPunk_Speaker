#include <Arduino.h>
#include "sr_driver.h"

#define SR_DIN 23
#define SR_CLK 18
#define SR_LAT 5
#define SR_CNT 6

inline void sr_write_byte(uint8_t val)
{ 
  char buf[64];
  for (int i=7; i>=0; i--) {
    digitalWrite(SR_CLK, LOW);
    bool bit_ = (bool)((1 << i) & val);
    if (bit_)
      digitalWrite(SR_DIN, HIGH);
    else
      digitalWrite(SR_DIN, LOW);
    
    digitalWrite(SR_CLK, HIGH);
  }
}
void sr_write(uint8_t *data, int N)
{
  digitalWrite(SR_LAT, LOW);
  
  for (int i=0; i<N; i++) {
    sr_write_byte(data[i]);
  }
  digitalWrite(SR_LAT, HIGH);
}
void init_shift_registers()
{
  pinMode(SR_DIN, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_LAT, OUTPUT);

  digitalWrite(SR_DIN, LOW);
  digitalWrite(SR_CLK, LOW);
  digitalWrite(SR_LAT, LOW);

  uint8_t clear_data[SR_CNT] = {
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
  sr_write(clear_data, SR_CNT);
}