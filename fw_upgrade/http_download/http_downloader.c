/*
 * Copyright (c) 2021 Nofence AS
 */

#include <net/download_client.h>
#include <logging/log.h>
#include "http_downloader.h"

#define LOG_MODULE_NAME http_downloader
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_FW_UPGRADE_LOG_LEVEL);

#define HTTP_DOWNLOAD_BUF_LEN                                                  \
	((CONFIG_DOWNLOAD_CLIENT_MAX_FILENAME_SIZE * 2) + 1)

static struct download_client dlc;
static int socket_retries_left;

/* Global variable that is needed to keep track of 
 * how many bytes of firmware we've downloaded.
 */
static int file_offset;
static bool first_fragment;

static http_download_cb app_callback;

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
	/* Data we will send further to the callback in main. */
	struct http_download_event file_evt = { .data.file_offset =
							file_offset };

	int err;
	if (event == NULL) {
		return -EINVAL;
	}

	switch (event->id) {
	case DOWNLOAD_CLIENT_EVT_FRAGMENT: {
		if (first_fragment) {
			file_offset = 0;
			err = download_client_file_size_get(
				&dlc, &file_evt.file_size);
			if (err) {
				LOG_DBG("download_client_file_size_get err: %d",
					err);
				return err;
			}
			first_fragment = false;
			file_evt.id = HTTP_DOWNLOAD_FIRST_FRAGMENT_RECEIVED;
		} else {
			file_evt.id = HTTP_DOWNLOAD_FRAGMENT_RECEIVED;
		}

		/* Here we have to assume the fragment is correctly 
		 * downloaded to event->fragment so we 
		 * fill in the necessary values 
		 * and attributes that will be sent to the callback at the end 
		 */
		file_evt.data.fragment = event->fragment.buf;
		file_evt.data.fragment_len = event->fragment.len;
		file_offset += event->fragment.len;
		break;
	}

	case DOWNLOAD_CLIENT_EVT_DONE:
		err = download_client_disconnect(&dlc);
		if (err) {
			LOG_INF("Cannot disconnect the download client %i",
				err);
			return err;
		}
		first_fragment = true;
		file_evt.id = HTTP_DOWNLOAD_FINISHED;
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
			LOG_ERR("Download client error");
			first_fragment = true;
			/* Return non-zero to tell download_client to stop
                         * callback and set event to error we received.
			 */
			file_evt.id = HTTP_DOWNLOAD_FAILED;
			app_callback(&file_evt);
			return event->error;
		}
	}
	default:
		LOG_INF("Unknown event in http_download_handler... %i",
			event->id);
		return 0;
		break;
	}
	if (file_evt.file_size <= 0) {
		err = download_client_file_size_get(&dlc, &file_evt.file_size);
		if (err) {
			LOG_WRN("Could not get file size %i", err);
		}
	}
	app_callback(&file_evt);
	return 0;
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

	if (host == NULL || file == NULL || app_callback == NULL) {
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

int http_download_init(http_download_cb cb)
{
	if (cb != NULL) {
		app_callback = cb;
	}

	first_fragment = true;
	file_offset = 0;

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
	file_offset = 0;

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