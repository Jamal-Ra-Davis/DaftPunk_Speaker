#pragma once
#include <Arduino.h>

int volume_init();
int volume_set(int8_t vol);
int8_t volume_get();
int volume_inc();
int volume_dec();
void volume_increase_cb(void *ctx);
void volume_decrease_cb(void *ctx);