/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <zephyr.h>
#include "event_manager.h"
#include "sound_event.h"

/** Default loudness for the buzzer in percent. */
#define BUZZER_SOUND_VOLUME_PERCENT 100

typedef enum { d_16 = 250, d_8 = 500 } duration_t;

/** Tone frequencies in hz
 * https://pages.mtu.edu/~suits/notefreqs.html
 */
typedef enum {
	tone_nothing = 0,
	tone_c_6 = 1046,
	tone_d_6 = 1174,
	tone_e_6 = 1318,
	tone_f_6 = 1396,
	tone_fiss_6 = 1480,
	tone_g_6 = 1568,
	tone_a_6 = 1760,
	tone_aiss_6 = 1864,
	tone_b_6 = 1975,
	tone_c_7 = 2093,
	tone_d_7 = 2349,
	tone_e_7 = 2637,
	tone_f_7 = 2794
} tone_t;

typedef struct {
	/** Tone/frequency. Note being played. */
	tone_t t;

	/** Sustain. Duration of note being played. */
	duration_t s;
} note_t;

int buzzer_module_init(void);

#endif /* _BUZZER_H_ */