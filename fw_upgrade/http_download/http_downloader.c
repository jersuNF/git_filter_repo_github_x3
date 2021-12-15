/*
 * Copyright (c) 2021 Nofence AS
 */

#include <net/download_client.h>
#include <logging/log.h>
#include "http_downloader.h"
#include "fw_upgrade_events.h"

#define LOG_MODULE_NAME http_downloader
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_FW_UPGRADE_LOG_LEVEL);

#define HTTP_DOWNLOAD_BUF_LEN                                                  \
	((CONFIG_DOWNLOAD_CLIENT_MAX_FILENAME_SIZE * 2) + 1)

static struct download_client dlc;
static int socket_retries_left;

/* Global variable that is needed to keep track of 
 * how many bytes of firmware we've downloaded.
 */
static volatile int current_download_file_offset;

/**
 * @brief Download callback handler which handles download events and performs
 *        actions accordingly. The events and fragments received is then
 *        sent forward to the initied callback function from external source.
 *
 * @param[in] event Pointer to event struct that contains event trigger, error
 *                  code, fragment payload, size, etc...
 *
 * @return 0 on success, otherwise negative error code.
 */
static int http_download_handler(const struct download_client_evt *event)
{
	int err = 0;
	if (event == NULL) {
		err = -EINVAL;
		goto cleanup_error;
	}

	size_t file_size = 0;

	switch (event->id) {
	case DOWNLOAD_CLIENT_EVT_FRAGMENT: {
		if (current_download_file_offset == 0) {
			err = download_client_file_size_get(&dlc, &file_size);
			if (err) {
				LOG_DBG("download_client_file_size_get err: %d",
					err);
				goto cleanup_error;
			}

			/** @note File size might not be available from 
			 *  first fragment must be verified and tested properly. 
			 *  Might be better to get file_size from protobuf?
		 	 */
			if (file_size == 0) {
				LOG_ERR("File size read is 0, cannot send \
				fragment without knowing size of the file.");
				goto cleanup_error;
			}
		}
		/* Increase the file_offset. */
		current_download_file_offset += event->fragment.len;

		/* Send fragment to firmware upgrade event handler. */
		struct dfu_fragment_event *frag_event =
			new_dfu_fragment_event(event->fragment.len);

		/* Write data with variable size. */
		memcpy(frag_event->dyndata.data, event->fragment.buf,
		       event->fragment.len);

		/* Store other variables such as file size etc... */
		frag_event->file_size = file_size;

		/* Submit event. */
		EVENT_SUBMIT(frag_event);
		break;
	}

	case DOWNLOAD_CLIENT_EVT_DONE:
		err = download_client_disconnect(&dlc);
		if (err) {
			LOG_INF("Cannot disconnect the download client %i",
				err);
			goto cleanup_error;
		}
		current_download_file_offset = 0;
		break;

	case DOWNLOAD_CLIENT_EVT_ERROR: {
		/* In case of socket errors we can return 0 to retry/continue,
		 * or non-zero to stop.
		 */
		if ((socket_retries_left) && ((event->error == -ENOTCONN) ||
					      (event->error == -ECONNRESET))) {
			LOG_WRN("Download socket error. %d retries left...",
				socket_retries_left);
			socket_retries_left--;
			/* Fall through and return 0 below to tell
			 * download_client to retry.
			 */
		} else {
			download_client_disconnect(&dlc);
			err = event->error;
			LOG_ERR("Download client error %i", err);
			goto cleanup_error;
		}
	}
	default:
		LOG_INF("Unknown event in http_download_handler... %i",
			event->id);
		break;
	}
	return 0;
cleanup_error:
	/* Set file offset to 0 so we can re-trigger dl from scratch later. */
	current_download_file_offset = 0;
	return err;
}

int http_download_start(const char *host, const char *file, int sec_tag,
			const char *apn, size_t fragment_size)
{
	/* We need a static file buffer since the download client structure
	 * only keeps a pointer to the file buffer. This is problematic when
	 * a download needs to be restarted for some reason (e.g. if
	 * continuing a download operation from an offset).
	 */
	static char file_buf[HTTP_DOWNLOAD_BUF_LEN];
	const char *file_buf_ptr = file_buf;
	int err = -1;

	struct download_client_cfg config = {
		.sec_tag = sec_tag,
		.apn = apn,
		.frag_size_override = fragment_size,
		.set_tls_hostname = (sec_tag != -1),
	};

	if (host == NULL || file == NULL) {
		return -EINVAL;
	}

	socket_retries_left = CONFIG_HTTP_DOWNLOAD_SOCKET_RETRIES;

	strncpy(file_buf, file, sizeof(file_buf));

	err = download_client_connect(&dlc, host, &config);
	if (err) {
		return err;
	}

	err = download_client_start(&dlc, file_buf_ptr, 0);
	if (err) {
		download_client_disconnect(&dlc);
		return err;
	}
	return 0;
}

int http_download_init()
{
	current_download_file_offset = 0;

	int err = download_client_init(&dlc, http_download_handler);
	if (err) {
		return err;
	}
	return 0;
}

int http_download_cancel(void)
{
	/* We want to cancel download, so we clear the file offset to prepare
	 * for next download.
	 */
	current_download_file_offset = 0;

	if (dlc.fd == -1) {
		LOG_WRN("Cannot cancel, download not started, \
		or has been aborted or completed");
		return -EAGAIN;
	}

	int err = download_client_disconnect(&dlc);
	if (err) {
		LOG_ERR("Failed to disconnect: %d", err);
		return err;
	}
	return err;
}