/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <zephyr.h>
#include "event_manager.h"
#include "sound_event.h"

/** Freq of warning start. (1000000 / 2000)*/
#define WARN_FREQ_INIT 500

/** Freq of max warning. (1000000 / 4200) */
#define WARN_FREQ_MAX 248

/** Duration of the warning tone in ms. */
#define WARN_MIN_DURATION_MS 5000

typedef enum { d_16 = 250, d_8 = 500 } duration_t;

/** Tone frequencies interval in microseconds. (1000000 / hz)
 * https://pages.mtu.edu/~suits/notefreqs.html
 */
typedef enum {
	tone_nothing = 0,
	tone_c_6 = 956,
	tone_d_6 = 851,
	tone_e_6 = 758,
	tone_f_6 = 716,
	tone_fiss_6 = 676,
	tone_g_6 = 638,
	tone_a_6 = 568,
	tone_aiss_6 = 536,
	tone_b_6 = 506,
	tone_c_7 = 478,
	tone_d_7 = 426,
	tone_e_7 = 379,
	tone_f_7 = 358
} tone_t;

typedef struct {
	/** Tone/frequency. Note being played. */
	tone_t t;

	/** Sustain. Duration of note being played. */
	duration_t s;
} note_t;

int buzzer_module_init(void);

#endif /* _BUZZER_H_ */