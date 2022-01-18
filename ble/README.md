# Bluetooth module
This module contains events regarding bluetooth communication and configuration of the device.

## BLR UART receive
Large data frames is blockwise reveived, and a new_ble_data_event is submitted for each block inside bt_receive callback.

```
/* Allocate event. */
struct ble_data_event *event = new_ble_data_event();
copy_len = remainder > BLE_RX_BLOCK_SIZE ? BLE_RX_BLOCK_SIZE : remainder;
remainder -= copy_len;
memcpy(buf, data, copy_len);
event->buf = buf;
event->len = copy_len;
EVENT_SUBMIT(event);
```

## BLR UART send
An event from the message(msg) module can be triggered to send a BLE UART event:
```
/* Allocate event. */
struct msg_data_event *event = new_msg_data_event();

/* Create buffer and fill with dummy string */
struct msg_rx_buf *buf;
int len = sprintf(&buf, "Message counter %d", ++uart_msg_counter);

/* Store buffer and length */
event->buf = &buf;
event->len = len;

/* Submit event. */
EVENT_SUBMIT(event); // Submit dummy data for event test
```
