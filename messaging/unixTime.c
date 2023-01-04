/*
 * unixTime.c
 *
 * Created: 15.05.2013 10:20:53
 *  Author: Oscar
 */

//#include <util/atomic.h>
#include "unixTime.h"

volatile uint32_t g_u32_UnixEpoch;
uint8_t g_u8_GPS_TimeErrorFlag;

//void nf_set_unix_time(uint32_t unix_time) {
//	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//		g_u32_UnixEpoch = unix_time;
//	}
//}
//uint32_t nf_get_unix_time() {
//	uint32_t unix_time;
//	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//		unix_time = g_u32_UnixEpoch;
//	}
//	return unix_time;
//}

const int16_t m_i16_mth[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

static uint8_t LeapYear(uint16_t year)
{
	return (((!(year % 4)) && (year % 100)) || (!(year % 400)));
}

#define JAN_1_2016 1451606400

uint32_t time2unix(nf_time_t *time)
{
	uint32_t myWorkYear;
	uint32_t timestamp = JAN_1_2016;

	if ((time->year < 2016) || (time->year > 2050) || (time->month < 1) || (time->month > 12) ||
	    (time->day < 1) || (time->day > 31)) {
		return 0; //test validity
	}

	for (myWorkYear = 2016; myWorkYear < time->year; myWorkYear++) {
		timestamp += (uint32_t)m_i16_mth[12] * SEC_DAY;
		if (LeapYear(myWorkYear))
			timestamp += (uint32_t)SEC_DAY; // add one day if leap year
	}

	timestamp += ((uint32_t)m_i16_mth[time->month - 1] * SEC_DAY) +
		     ((uint32_t)(time->day - 1) * SEC_DAY) + ((uint32_t)time->hour * SEC_HOUR) +
		     ((uint32_t)time->minute * SEC_MIN) + time->second;
	if (LeapYear(time->year) && (time->month > 2))
		timestamp += (uint32_t)SEC_DAY; // add one day if leap year and passed February

	return timestamp;
}

uint8_t unix2time(nf_time_t *time, uint32_t unixTimestamp)
{
	uint32_t myTimestamp;
	uint16_t myDaystamp, myTmpDaystamp;

	myDaystamp = unixTimestamp / SEC_DAY; //Days since 1.1.1970 (date)
	myTimestamp = unixTimestamp % SEC_DAY; //Seconds of day (time)

	myTmpDaystamp = myDaystamp;
	for (time->year = 1970; myDaystamp <= myTmpDaystamp;
	     time->year++) // work the years bottom up until daystamp wrap
	{
		myTmpDaystamp = myDaystamp;
		myDaystamp -= m_i16_mth[12];
		if (LeapYear(time->year))
			myDaystamp--; // remove one extra day if leap year
	}
	time->year--;

	myDaystamp = myTmpDaystamp; //set daystamp back to this years remaining days
	for (time->month = 1; myTmpDaystamp <= myDaystamp;
	     time->month++) // work the months until timestamp wrap
	{
		myTmpDaystamp = m_i16_mth[time->month];
		if (LeapYear(time->year) && (time->month == 3))
			myDaystamp--; // One extra day if past february of leap year
	}
	time->month--;
	time->day = myDaystamp - m_i16_mth[time->month - 1] + 1; //Save remaining days
	if ((time->month == 3) && (time->day == 0)) {
		time->month--;
		time->day = m_i16_mth[2] - m_i16_mth[1] + 1;
	} //leap day

	time->hour = myTimestamp / SEC_HOUR; //Find hours of remaining seconds of timestamp
	myTimestamp %= SEC_HOUR; // Remaining seconds after finding hour

	time->minute = myTimestamp / SEC_MIN; //Find minutes of remaining seconds of timestamp
	time->second = myTimestamp %
		       SEC_MIN; // Remaining seconds after finding minute is the resulting second

	if (time->second > SEC_MIN)
		return 0;
	return 1;
}

// Calculates the weekday from 0 to 6, where 0 is Monday, and 6 is Sunday. Assuming 1.1.1970 was a Thursday
uint8_t ut_Weekday(uint32_t unixTimestamp)
{
	uint16_t myDaystamp = unixTimestamp / SEC_DAY; //Days since 1.1.1970 (date)
	myDaystamp += 3; //because 1.1.1970 fell on a Thursday (weekdays = 0-6)
	return myDaystamp % 7; //seven days a week
}

uint8_t ut_HourOfDay(uint32_t unixTimestamp)
{
	uint32_t myTimestamp;

	myTimestamp = unixTimestamp % SEC_DAY; // Seconds of day (time)
	return (uint8_t)(myTimestamp / SEC_HOUR); // Find hours of remaining seconds of timestamp
}

uint8_t ut_MinOfHour(uint32_t unixTimestamp)
{
	uint32_t myTimestamp;

	myTimestamp = unixTimestamp % SEC_DAY; // Seconds of day (time)
	myTimestamp %= SEC_HOUR; // Remaining seconds after finding hour
	return (uint8_t)(myTimestamp / SEC_MIN); // Return minutes part of unix timestamp
}

bool ut_IsSameDayOrGreater(nf_time_t *now, nf_time_t *future)
{
	if (now->year > future->year)
		return false;
	if (now->year < future->year)
		return true;
	if (now->month > future->month)
		return false;
	if (now->month < future->month)
		return true;
	if (now->day > future->day)
		return false;
	return true;
}
