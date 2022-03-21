/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef MELODIES_H_
#define MELODIES_H_

#include <zephyr.h>
#include <device.h>
#include "buzzer.h"

static const note_t m_geiterams[] = {
	{ .t = tone_nothing, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_d_6, .s = d_16 },
	{ .t = tone_c_6, .s = d_8 },
	{ .t = tone_c_6, .s = d_8 },
	/* .. */
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_g_6, .s = d_16 },
	{ .t = tone_c_7, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_g_6, .s = d_8 },
	{ .t = tone_g_6, .s = d_8 },
	/* .. */
	{ .t = tone_b_6, .s = d_16 },
	{ .t = tone_b_6, .s = d_16 },
	{ .t = tone_b_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_f_6, .s = d_8 },
	{ .t = tone_f_6, .s = d_8 },
	/* .. */
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_g_6, .s = d_16 },
	{ .t = tone_e_6, .s = d_8 },
	{ .t = tone_e_6, .s = d_8 },
};

static const note_t m_perspelmann[] = {
	{ .t = tone_nothing, .s = d_16 },
	{ .t = tone_c_7, .s = d_16 },
	{ .t = tone_c_7, .s = d_8 },
	{ .t = tone_g_6, .s = d_8 },
	{ .t = tone_aiss_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_8 },
	{ .t = tone_f_6, .s = d_8 },
	{ .t = tone_f_7, .s = d_16 },
	{ .t = tone_e_6, .s = d_8 },
	/* .. */
	{ .t = tone_f_6, .s = d_8 },
	{ .t = tone_d_6, .s = d_16 },
	{ .t = tone_c_6, .s = d_8 },
};

static const note_t m_setupmode[] = {
	{ .t = tone_c_6, .s = 100 },	{ .t = tone_nothing, .s = 40 },
	{ .t = tone_d_6, .s = 100 },	{ .t = tone_nothing, .s = 40 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_e_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_f_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_g_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_g_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_c_6, .s = 300 }
};

#define n_geiterams_notes (sizeof(m_geiterams) / sizeof(m_geiterams[0]))
#define n_perspelmann_notes (sizeof(m_perspelmann) / sizeof(m_perspelmann[0]))
#define n_setupmode_notes (sizeof(m_setupmode) / sizeof(m_setupmode[0]))

/* Deprecated? 
static const note_t m_nsetupmode[] = {
	{ .t = tone_nothing, .s = 40 }, { .t = tone_e_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_g_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_f_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_e_6, .s = 100 },
	{ .t = tone_nothing, .s = 40 }, { .t = tone_d_6, .s = 500 }
};

#define n_nsetupmode_notes (sizeof(m_nsetupmode) / sizeof(m_nsetupmode[0])) 
*/

#endif /* MELODIES_H_ */