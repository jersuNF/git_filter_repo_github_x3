#ifndef X3_FW_UNIXTIME_H
#define X3_FW_UNIXTIME_H
/*
 * unicTime.h
 *
 * Created: 15.05.2013 10:25:20
 *  Author: Oscar
 */
#include <stdint.h>
#include <stdbool.h>
#ifndef UNIXTIME_H_
#define UNIXTIME_H_

#define SEC_MIN 60
#define SEC_HOUR 3600
#define SEC_DAY 86400

typedef struct _nf_time_t {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} nf_time_t;


uint32_t time2unix(nf_time_t *time);
uint8_t unix2time(nf_time_t *time, uint32_t unixTimestamp);
// Calculates the weekday from 0 to 6, where 0 is Monday, and 6 is Sunday. Assuming 1.1.1975 was a Thursday
uint8_t ut_Weekday(uint32_t unixTimestamp);
// Returns the hour of day (0 - 23)
uint8_t ut_HourOfDay(uint32_t unixTimestamp);
// Returns the minute of hour (0 - 59)
uint8_t ut_MinOfHour(uint32_t unixTimestamp);

bool ut_IsSameDayOrGreater(nf_time_t *now, nf_time_t *future);

#endif /* UNIXTIME_H_ */
#endif //X3_FW_UNIXTIME_H
