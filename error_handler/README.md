# Error handling module

This module subscribes to an error event, which other modules can publish to. This event handler then checks the sender, severity and performs actions accordingly.

## Submitting an error_event

To submit an error to the event, simply call the nf_app_fatal, nf_app_error or nf_app_warning and pass the necessary parameters.

```
nf_app_fatal(ERR_FW_UPGRADE, -ENOMEM, e_msg, strlen(e_msg));
```
