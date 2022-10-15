#include "Arduino.h"
#include "timer_thread.h"

struct timer_thread{
    bool active;
    uint32_t period;
    uint32_t ts;
    timer_thread_func_t func;
    void *ctx;
};

static int thread_cnt = 0;
static struct timer_thread threads[NUM_TIMER_THREADS];

static bool check_timer(struct timer_thread *thread, uint32_t ts) {
  if (ts - thread->ts >= thread->period) {
    thread->ts = ts;
    return true;
  }
  return false;
}

int register_timer_thread(timer_thread_func_t func, void *ctx, uint32_t period)
{
    if (thread_cnt >= NUM_TIMER_THREADS) {
        return -1;
    }
    uint32_t ts = micros();
    threads[thread_cnt].active = true;
    threads[thread_cnt].period = period;
    threads[thread_cnt].ts = ts;
    threads[thread_cnt].func = func;
    threads[thread_cnt].ctx = ctx;
    thread_cnt++;
    return 0;
}
void update_timer_threads()
{
    uint32_t ts = micros();
    for (int i=0; i<NUM_TIMER_THREADS; i++) {
        if (!threads[i].active) {
            continue;
        }
        if (!check_timer(&threads[i], ts)) {
            continue;
        }
        if (threads[i].func != NULL) {
            threads[i].func(threads[i].ctx);
        }
    }
}