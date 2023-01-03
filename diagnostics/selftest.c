#include "selftest.h"

static uint32_t selftest_done = 0;
static uint32_t selftest_passed = 0;

int selftest_init(void)
{
	selftest_done = 0;
	selftest_passed = 0;

	return 0;
}

int selftest_mark_state(uint8_t selftest_pos, bool passed)
{
	if (selftest_pos >= 32) {
		return -EINVAL;
	}

	selftest_done |= (1 << selftest_pos);
	if (passed) {
		selftest_passed |= (1 << selftest_pos);
	}

	return 0;
}

int selftest_get_result(uint32_t *test_done, uint32_t *test_passed)
{
	*test_done = selftest_done;
	*test_passed = selftest_passed;

	return 0;
}