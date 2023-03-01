#pragma once

#define NUM_TIMER_THREADS 16

typedef void (*timer_thread_func_t)(void *ctx);
int init_timer_thread_manager();
int register_timer_thread(timer_thread_func_t func, void *ctx, uint32_t period);
void update_timer_threads();
