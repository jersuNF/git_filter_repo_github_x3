/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _HTTP_DOWNLOADER_H_
#define _HTTP_DOWNLOADER_H_

#include <zephyr.h>

/**
 * @brief DL_FILE event IDs.
 */
enum http_download_event_id {
	HTTP_DOWNLOAD_FIRST_FRAGMENT_RECEIVED,
	HTTP_DOWNLOAD_FRAGMENT_RECEIVED,
	HTTP_DOWNLOAD_FINISHED,
	HTTP_DOWNLOAD_FAILED
};

struct http_download_fragment_payload {
	/** Pointer to where we find the fragment data. */
	const void *fragment;
	/** Size of the fragment received. */
	size_t fragment_len;
	/** Start offset of where the fragment is located relative to
     	* total file size.
     	*/
	int file_offset;
};

/**
 * @brief Download fragment event
 */
struct http_download_event {
	/** Event that triggered the callback and 
	 *  error code from errno.h if event is
	 *  of the type HTTP_DOWNLOAD_FAILED.
	 */
	enum http_download_event_id id;
	int error_code;

	/** Overall file_size. */
	size_t file_size;
	/** Data containing the fragment and fragment size. */
	struct http_download_fragment_payload data;
	/** Download progress %. */
	int progress;
};

/**
 * @brief Initializes the dl_file module. 
 *        The passed function is a callback function
 *        in which we recieve raw fragments 
 *        and length of the file being processed.
 *
 * @return 0 on success, otherwise negative error code.
 */
int http_download_init(void);

/**
 * @brief Cancels current download process.
 * 
 * @return 0 on success, otherwise negative error code.
 */
int http_download_cancel(void);

/**
 * @brief Start the download function call. This should be called
 *        once we have retrieved a job, and an URL of where
 *        we can access the file.
 * 
 * @param[in] host Pointer to host name of where we find the file.
 * @param[in] file Pointer file path of 
 *                 where we can find the file on given host.
 * @param[in] sec_tag Security tag for tls downloads.
 * @param[in] apn Network access point, NULL if not needed.
 * @param[in] fragment_size Fragment_size we want to download with. 
 *                          Set to 0 to use default (2048).
 * 
 * @return 0 on success, otherwise negative error code.
 */
int http_download_start(const char *host, const char *file, int sec_tag,
			const char *apn, size_t fragment_size);

#endif /* _HTTP_DOWNLOADER_H_ */