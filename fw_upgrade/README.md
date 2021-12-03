# Firmware upgrade module
This module contains two events regarding firmware upgrade of the device.

1. The first event is an event that the firmware upgrade module subscribes to, which is a dynamic event that stores a fragment with given fragment length and an origin of the fragment (i.e from bluetooth or modem module).
2. Second event is contains information regarding the status of the upgrade, we set the status variable to 1 when we start DFU process, and we set it to 2 when we want the devices to reboot/configure for a device reboot and apply new firmware. Default 0 means operating normally. (We can add other flags if needed, i.e errors)

## Creating a new fragment
Since a fragment size varies, we have to use dynamic data storage in the events, hence struct dyndata is being used. The way to write a new fragment event can be seen below as an example:
```
/* Allocate event. */
struct dfu_fragment_event *event = dfu_fragment_event(fragment_size);

/* Write data with variable size. */
memcpy(event->dyndata.data, fragment, fragment_size);

/* Store other variables such as file size etc... */
event->file_size = file_size;

/* Submit event. */
EVENT_SUBMIT(event);
```
