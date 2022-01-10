# Error handling module
This module subscribes to an error event, which other modules can publish to. This event handler then checks the sender, severity and performs actions accordingly.

## Submitting an error_event
To submit an error to the event, simply call the submit_error and pass the necessary parameters.

```
char* e_msg = "Stack overflow in fw module";
submit_error(ERR_SENDER_FW_UPGRADE, ERR_SEVERITY_FATAL, -ENOMEM, e_msg, strlen(e_msg));
```
Empty message is also supported:
```
submit_error(ERR_SENDER_FW_UPGRADE, ERR_SEVERITY_FATAL, -ENOMEM, NULL, 0);
```