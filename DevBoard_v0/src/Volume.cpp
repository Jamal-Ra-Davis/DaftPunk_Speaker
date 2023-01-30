#include <Arduino.h>
#include "Volume.h"
#include "global_defines.h"
#include "Logging.h"
#include <BluetoothA2DPSink.h>

static const uint8_t MAX_VOLUME_LEVEL = 8;
static const uint8_t VOLUME_SCALE = 16;
static const uint8_t VOLUME_DEFAULT = 4;
static int8_t volume_level = VOLUME_DEFAULT;

extern BluetoothA2DPSink a2dp_sink;

int volume_init()
{
    return volume_set(VOLUME_DEFAULT);
}
int volume_set(int8_t vol)
{
    if (vol > MAX_VOLUME_LEVEL)
    {
        volume_level = MAX_VOLUME_LEVEL;
    }
    else if (vol < 0) {
        volume_level = 0;
    }
    else {
        volume_level = vol;
    }
    a2dp_sink.set_volume(volume_level * VOLUME_SCALE);
    return 0;
}
int8_t volume_get()
{
    return volume_level;
}
int volume_inc()
{
    int8_t vol = volume_get() + 1;
    return volume_set(vol);
}
int volume_dec()
{
    int8_t vol = volume_get() - 1;
    return volume_set(vol);
}
void volume_increase_cb(void *ctx)
{
    log_inf("Volume Increase Pressed");
    volume_inc();
}
void volume_decrease_cb(void *ctx)
{
    log_inf("Volume Decrease Pressed");
    volume_dec();
}