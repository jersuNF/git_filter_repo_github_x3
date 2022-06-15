#include <stdlib.h>
#include <stdio.h>
#include "time_use_helpers.h"
#include "stdint.h"
#include "trigonometry.h"

/** TimeuseDistance
 * Description	: Compute the distance from A to B
 * Return value	: the distance between a given point and most recent GPS
 * (freshx,freshy)
 * position. If the distance calculation results in overflow, the result is set
 * to max value a 16 bit integer can hold (32767)
 */
int16_t TimeuseDistance(int16_t *Pos, int16_t* fresh_pos) {
	int32_t d1;
	int32_t d2;
	uint32_t myDistance;

	d1 = (int32_t) Pos[0] - fresh_pos[0];
	d1 *= d1;
	d2 = (int32_t) Pos[1] - fresh_pos[1];
	d2 *= d2;

	myDistance = g_u32_SquareRootRounded((uint32_t) d1 + d2);

	if (myDistance > INT16_MAX) {
		myDistance = INT16_MAX;
	}
	return (int16_t) myDistance;
}