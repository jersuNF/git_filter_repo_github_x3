/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _HTTP_DOWNLOADER_H_
#define _HTTP_DOWNLOADER_H_

#include <zephyr.h>

/**
 * @brief Initializes the dl_file module. Typically we would pass a
 *        callback function, but we're using events to send the fragment.
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
 * @param[in] fragment_size Fragment_size we want to download with. 
 *                          Set to 0 to use default (2048).
 * 
 * @return 0 on success, otherwise negative error code.
 */
int http_download_start(const char *host, const char *file, int sec_tag,
			size_t fragment_size);

#endif /* _HTTP_DOWNLOADER_H_ */