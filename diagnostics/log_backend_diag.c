#include "log_backend_diag.h"

#include <logging/log_backend.h>
#include <logging/log_core.h>
#include <logging/log_msg.h>
#include <logging/log_output.h>
#include <logging/log_output_dict.h>
#include <logging/log_backend_std.h>
#include <logging/log.h>
#include <device.h>
#include <sys/__assert.h>

#define LOG_MODULE_NAME diag_back
LOG_MODULE_REGISTER(LOG_MODULE_NAME, 4);

static atomic_t log_backend_interface = ATOMIC_INIT(DIAGNOSTICS_NONE);

static struct log_backend_diag_action backend_diag_actions;

static uint8_t output_buffer[128];
static uint32_t output_buffer_count = 0;

static int char_out(uint8_t *data, size_t length, void *ctx)
{
        ARG_UNUSED(ctx);

        for (size_t i = 0; i < length; i++) {
                output_buffer[output_buffer_count] = data[i];
                output_buffer_count++;

                if ((data[i] == '\n') || 
                    (output_buffer_count >= sizeof(output_buffer))) {
                        enum diagnostics_interface intf = 
                                        atomic_get(&log_backend_interface);
                        if ((backend_diag_actions.send_resp != NULL) &&
                            (intf != DIAGNOSTICS_NONE)) {
                                backend_diag_actions.send_resp(
                                                        intf, 
                                                        output_buffer, 
                                                        output_buffer_count);
                                output_buffer_count = 0;
                        }
                }
        }

        return length;
}

static uint8_t diag_output_buf[100];

LOG_OUTPUT_DEFINE(log_output_diag, char_out, diag_output_buf, 100);

static void put(const struct log_backend *const backend,
		struct log_msg *msg)
{
	uint32_t flag = 0;

	log_backend_std_put(&log_output_diag, flag, msg);
}

static void process(const struct log_backend *const backend,
		union log_msg2_generic *msg)
{
	uint32_t flags = log_backend_std_get_flags();

	log_output_msg2_process(&log_output_diag, &msg->log, flags);
}

int log_backend_diag_init(struct log_backend_diag_action* actions)
{
	if (actions != NULL) {
		backend_diag_actions = *actions;
	}

	return 0;
}

int log_backend_diag_enable(enum diagnostics_interface intf)
{
        atomic_set(&log_backend_interface, intf);

        return 0;
}

int log_backend_diag_disable(void)
{
        atomic_set(&log_backend_interface, DIAGNOSTICS_NONE);

        return 0;
}

static void log_backend_diag_boot_init(struct log_backend const *const backend)
{
	/* TODO - Any init for log in diag? */
}

static void panic(struct log_backend const *const backend)
{
	log_backend_std_panic(&log_output_diag);
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	log_backend_std_dropped(&log_output_diag, cnt);
}

static void sync_string(const struct log_backend *const backend,
		     struct log_msg_ids src_level, uint32_t timestamp,
		     const char *fmt, va_list ap)
{
	uint32_t flag = 0;

	log_backend_std_sync_string(&log_output_diag, flag, src_level,
				    timestamp, fmt, ap);
}

static void sync_hexdump(const struct log_backend *const backend,
			 struct log_msg_ids src_level, uint32_t timestamp,
			 const char *metadata, 
                         const uint8_t *data, uint32_t length)
{
	uint32_t flag = 0;

	log_backend_std_sync_hexdump(&log_output_diag, flag, src_level,
				     timestamp, metadata, data, length);
}

const struct log_backend_api log_backend_diag_api = {
	.process = IS_ENABLED(CONFIG_LOG2) ? process : NULL,
	.put = IS_ENABLED(CONFIG_LOG_MODE_DEFERRED) ? put : NULL,
	.put_sync_string = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ?
			sync_string : NULL,
	.put_sync_hexdump = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ?
			sync_hexdump : NULL,
	.panic = panic,
	.init = log_backend_diag_boot_init,
	.dropped = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : dropped,
};

LOG_BACKEND_DEFINE(log_backend_diag, log_backend_diag_api, true);
