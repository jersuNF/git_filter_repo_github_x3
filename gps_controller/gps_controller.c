//
// Created by alaa on 18.02.2022.
//

#include "gps_controller.h"
#include "drivers/gps.h"
#include "gps_controller_events.h"
#define STACK_SIZE 128;
#define PRIORITY 7;

int gps_controller_init(void){
	const struct device *gnss_dev = bind_gnss();
	if (gnss_dev == NULL) {
		LOG_ERR("Couldn't get instance of the GNSS device!\n");
		return -1;
	}
	gnss_driver_register_callback(position_update_cb);
	K_THREAD_DEFINE(my_tid, STACK_SIZE,
			request_and_publish_position, NULL,
			PRIORITY, 0, 0);
	return 0;
}

extern void request_and_publish_position(void){

	while(true){
		struct last_fix_type *last_fix_ptr;
		int ret = gnss.get_postion(last_fix_ptr);
		if (ret == 0){
			//build and publish position event
		}
	}
}

void init()
{
	gnss_driver_register_callback(position_update_cb);

	start_thread(thread);
}

//position_data_t buffer;

void thread() {
	while (true)
	{
		semaphore_take(sem, forever);
		do_stuff_to_position(gnss_driver_get_position());
	}
}

void thread2() {
	while (true)
	{
		semaphore_take(event_sem, forever);

		if (gnss_driver_get_rate() != gnss_config.rate)
			gnss_driver_set_rate(gnss_config.rate);

		gnss_driver_set_power_save(gnss_config.power_save);
	}
}



event_handler()
{
	gnss_config.rate = 1;
	give(event_sem)
}


void position_update_cb(position_data_t* pos) {
	//memcpy(buffer, pos, sizeof(position_data_t));

	semaphore_give(sem);
}

