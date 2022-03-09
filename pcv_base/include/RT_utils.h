#ifndef RT_UTILS_H
#define RT_UTILS_H

/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

/* standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* for union sigval */
#include <signal.h>

#define MAX_PRIO		(80)

/* functions for rt applications */
int launch_rt_thread (void *(*func)(void *), pthread_t *t, void *aux, int priority);

/* timer init for rt threads */
int init_rt_timer (timer_t *t, void (*handler)(union sigval val), void *aux);

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif

#endif
