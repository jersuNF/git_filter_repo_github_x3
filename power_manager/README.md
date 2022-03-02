# Power Manager module
This module contain events regarding Power Manager (PWR)


## Create a new power state event
Remember to include `pwr_module.h` and to initialize the module with `pwr_module_init()`
Trigger a power state change with:
```
struct pwr_status_event *pwr_event = new_pwr_status_event();
pwr_event->pwr_state = PWR_SLEEP;
EVENT_SUBMIT(pwr_event);
```

## Unit test
The unit test can be triggered with the command
```
../zephyr/scripts/twister -T tests/power_manager -O twister-out -c
```
