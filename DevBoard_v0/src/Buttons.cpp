#include <Arduino.h>
#include <BluetoothA2DPSink.h>
#include "Buttons.h"
#include "global_defines.h"

extern BluetoothA2DPSink a2dp_sink;

static const uint8_t MAX_VOLUME_LEVEL = 8;
static const uint8_t VOLUME_SCALE = 16;
static int8_t volume_level = 4;
static volatile bool pair_press = false;

static void volume_button_handler();
static void pair_button_handler();

void init_buttons()
{
  pinMode(VOL_P_PIN, INPUT);
  pinMode(VOL_M_PIN, INPUT);
  pinMode(PAIR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(VOL_P_PIN), volume_button_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VOL_M_PIN), volume_button_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PAIR_PIN), pair_button_handler, CHANGE);
}

// TODO: Want volume buttons to increase volume repeatedly if held down, no long press event. Use timer to achive this
static void volume_button_handler()
{
  if (digitalRead(VOL_P_PIN) == 0)
  {
    Serial.println("Volume Increase Pressed");

    volume_level++;
    if (volume_level > MAX_VOLUME_LEVEL)
    {
      volume_level = MAX_VOLUME_LEVEL;
    }
  }
  if (digitalRead(VOL_M_PIN) == 0)
  {
    Serial.println("Volume Decrease Pressed");
    volume_level--;
    if (volume_level < 0)
    {
      volume_level = 0;
    }
  }
  a2dp_sink.set_volume(volume_level * VOLUME_SCALE);
}

// Pair button can have normal short press long press behavior - LP: pair action, SP: select action
static void pair_button_handler()
{
  pair_press = true;
  Serial.println("Pair Button Pressed");
}