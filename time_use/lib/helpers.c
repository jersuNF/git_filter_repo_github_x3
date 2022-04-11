#include <stdlib.h>
#include <stdio.h>
#include "helpers.h"
#include "stdint.h"


/** Implemented at v3.21-17
 * \brief    Fast Square root algorithm, with rounding
 *
 * This does arithmetic rounding of the result. That is, if the real answer
 * would have a fractional part of 0.5 or greater, the result is rounded up to
 * the next integer.
 *      - SquareRootRounded(2) --> 1
 *      - SquareRootRounded(3) --> 2
 *      - SquareRootRounded(4) --> 2
 *      - SquareRootRounded(6) --> 2
 *      - SquareRootRounded(7) --> 3
 *      - SquareRootRounded(8) --> 3
 *      - SquareRootRounded(9) --> 3
 *
 * \param[in] a_nInput - unsigned integer for which to find the square root
 *
 * \return Integer square root of the input value.
 */
uint32_t g_u32_SquareRootRounded(uint32_t a_nInput)
{
	uint32_t op  = a_nInput;
	uint32_t res = 0;
	uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


	// "one" starts at the highest power of four <= than the argument.
	while (one > op)
	{
		one >>= 2;
	}

	while (one != 0)
	{
		if (op >= res + one)
		{
			op = op - (res + one);
			res = res +  2 * one;
		}
		res >>= 1;
		one >>= 2;
	}

	/* Do arithmetic rounding to nearest integer */
	if (op > res)
	{
		res++;
	}

	return res;
}

/** TimeuseDistance
 * Description	: Compute the distance from A to B
 * Return value	: the distance between a given point and most recent GPS
 * position. If the distance calculation results in overflow, the result is set
 * to max value a 16 bit integer can hold (32767)
 */
int16_t TimeuseDistance(int16_t *Pos) {
	int32_t d1;
	int32_t d2;
	uint32_t myDistance;

	d1 = (int32_t) Pos[0] - GPS()->X;
	d1 *= d1;
	d2 = (int32_t) Pos[1] - GPS()->Y;
	d2 *= d2;

	myDistance = g_u32_SquareRootRounded((uint32_t) d1 + d2);

	if (myDistance > INT16_MAX) {
		myDistance = INT16_MAX;
	}
	return (int16_t) myDistance;
}