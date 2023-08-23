#include <zephyr.h>
#include "trice.h"
#include "nclogs.h"
#include <sys/ring_buffer.h>
#include "nf_version.h"
#include <date_time.h>
#include <messaging_module_events.h>

struct nclog_buffer_t {
	uint32_t rb_magic_number;
	struct ring_buf rb_trice;
	uint32_t _ring_buffer_data_rb_trice[CONFIG_NCLOG_BUFFER_SIZE];
};

struct nclog_framework_t {
	eNCLOG_LVL level[_eNCLOG_MODULE_MAX + 1];
	struct nclog_buffer_t nclog_buffer;
};

static struct nclog_framework_t __noinit nclogs;
static bool nclogs_initialized = false;
int nclogs_write(uint8_t *buf, size_t len)
{
	static unsigned counter = 0;
	size_t available_space = ring_buf_space_get(&nclogs.nclog_buffer.rb_trice);
	if (available_space <= len) {
		counter++;
		return -ENOMEM;
	}
	if (counter != 0) {
		unsigned tmp_counter=counter;
		counter = 0; /* we need to reset this counter before logging */
		NCLOG_ERR(UNDEFINED, TRice( iD( 7919),"err: nclogs_write: Not enough space in buffer. %u logs were discarded", tmp_counter ));
	}
	
	bool overflow = false;
	uint32_t bytes_written = ring_buf_put(&nclogs.nclog_buffer.rb_trice, buf, len);
	if (bytes_written < len) {
		/* Make room for data (discard oldest if buffer is full)*/
		ring_buf_get(&nclogs.nclog_buffer.rb_trice, NULL, len - bytes_written);

		bytes_written += ring_buf_put(&nclogs.nclog_buffer.rb_trice, &buf[bytes_written],
					      len - bytes_written);
		overflow = true;
	}
	return overflow ? -ENOMEM : 0;
}

int nclogs_read(uint8_t *buf, size_t cnt)
{
	return ring_buf_get(&nclogs.nclog_buffer.rb_trice, buf, cnt);
}
bool nclog_is_initialized()
{
	return nclogs_initialized;
}
int nclogs_read_with_sucess(int (*callback)(uint8_t *buf, size_t cnt))
{
	uint8_t *data = NULL;

	uint32_t cnt =
		ring_buf_get_claim(&nclogs.nclog_buffer.rb_trice, &data, CONFIG_NCLOG_BUFFER_SIZE);

	if (callback(data, cnt) == 0) {
		return ring_buf_get_finish(&nclogs.nclog_buffer.rb_trice, cnt);
	}

	return ring_buf_get_finish(&nclogs.nclog_buffer.rb_trice, 0);
}

bool nclog_is_enabled(eNCLOG_MODULE module, eNCLOG_LVL level)
{
	/* NClogs of greater severity are displayed */
	if (nclogs.level[module] >= level) {
		return nclogs_initialized;
	}
	return false;
}

int nclog_set_level(eNCLOG_MODULE module, eNCLOG_LVL level)
{
	if (level <= eNCLOG_LVL_MAX && level >= 0 && module <= _eNCLOG_MODULE_MAX && module >= 0) {
		nclogs.level[module] = level;
		return 0;
	}
	return -EINVAL;
}
int nclog_get_available_bytes()
{
	if (!nclogs_initialized) {
		return -EACCES;
	}
	return ring_buf_size_get(&nclogs.nclog_buffer.rb_trice);
}
eNCLOG_LVL nclog_get_level(eNCLOG_MODULE module)
{
	return nclogs.level[module];
}

void nclogs_module_init()
{
	/* IMPORTANT: No logging can happen before the write pointer is updated */
	trice_set_custom_write(&nclogs_write);
	nclogs_initialized = true;

	/* Check if buffer has been initialized by using an "unique" number for the build. */
	if (nclogs.nclog_buffer.rb_magic_number !=
	    (uint32_t)__TIME__[6] + __TIME__[7] + __TIME__[8] + NF_X25_VERSION_NUMBER) {
		nclogs.nclog_buffer.rb_magic_number =
			(uint32_t)__TIME__[6] + __TIME__[7] + __TIME__[8] + NF_X25_VERSION_NUMBER;
		ring_buf_init(&nclogs.nclog_buffer.rb_trice, CONFIG_NCLOG_BUFFER_SIZE,
			      nclogs.nclog_buffer._ring_buffer_data_rb_trice);
		for (size_t i = 0; i <= _eNCLOG_MODULE_MAX; i++) {
			nclogs.level[i] = CONFIG_NCLOG_DEFAULT_LEVEL;
		}
		NCLOG_INF(UNDEFINED, TRice0( iD( 3161),"inf: Magic number not set. Initializing nclogs module in noinit memory region"));
	} else {
		NCLOG_INF(UNDEFINED, TRice0( iD( 1458),"inf: Magic number set. Using existing data stored in noinit memory region"));
	}
}

/* We're logging with uptime in milliseconds */
uint32_t TriceStamp32(void)
{
	return k_uptime_get_32();
}
