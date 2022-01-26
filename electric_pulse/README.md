# Electric pulse module
This module contain events regarding electric pulse (EP)

If the application is built for nrf52840dk, LED 1 will light when pulse is given

## Create a new electric pulse event
Remember to include `ep_module.h` and to initialize the module with `ep_module_init()`
Trigger an electrical pulse with:
```
struct ep_status_event *ep_event = new_ep_status_event();
ep_event->ep_status = EP_RELEASE;
EVENT_SUBMIT(ep_event);
```
Due to a safety feature, the minimum time between each electrical pulse is 5 seconds.
## Unit test
The unit test can be triggered with the command
```
../zephyr/scripts/twister -T tests/electric_pulse -O twister-out -c
```
