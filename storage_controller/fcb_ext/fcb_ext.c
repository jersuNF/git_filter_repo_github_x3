/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/fcb.h>
#include <string.h>

#include "fcb_ext.h"
#include "UBX.h"

int fcb_walk_from_entry(fcb_read_cb cb, struct fcb *fcb,
			struct fcb_entry *start_entry, uint16_t num_entries)
{
	int err = 0;
	int read_entry_counter = 0;

	if (fcb_is_empty(fcb)) {
		return -ENODATA;
	}

	struct fcb_entry target_entry;
	if (start_entry != NULL && start_entry->fe_sector != NULL) {
		memcpy(&target_entry, start_entry, sizeof(struct fcb_entry));
	} else {
		/* If we get passed NULL as entry, read all entries. */
		target_entry.fe_sector = NULL;
		target_entry.fe_elem_off = 0;
		err = fcb_getnext(fcb, &target_entry);
		if (err) {
			return err;
		}
	}

	while (true) {
		if (target_entry.fe_sector == NULL) {
			return -ENODATA;
		}

		uint8_t *data = k_malloc(target_entry.fe_data_len);

		err = flash_area_read(fcb->fap,
				      FCB_ENTRY_FA_DATA_OFF(target_entry), data,
				      target_entry.fe_data_len);

		if (err) {
			k_free(data);
			return err;
		}

		err = cb(data, target_entry.fe_data_len);
		k_free(data);

		if (err) {
			/* Used if caller wants to abort walk process. I.e -EINTR. */
			return err;
		}

		/* Output the new entry we just read if user wants to use it. */
		if (start_entry != NULL) {
			memcpy(start_entry, &target_entry,
			       sizeof(struct fcb_entry));
		}

		/* Check if we want to exit since caller 
                 * just requested last n entries. 
                 */
		if (num_entries > 0) {
			if (++read_entry_counter >= num_entries) {
				return 0;
			}
		}

		err = fcb_getnext(fcb, &target_entry);

		if (err) {
			/* We end up here if getnext doesn't find next entry
			 * in which case we have iterated through everything
			 * and need to break out of the loop. 
			 */
			return 0;
		}
	}
	return 0;
}