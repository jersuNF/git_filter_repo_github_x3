# Cellular controller

* Handles the cellular communication by being the only entry point to the sara
  modem. It has a handle of the network interface initiated in the modem
  driver, to be able to control TCP socket create/connect/stop.
* When receiving a binary message from the server, it publishes an event
  'cellular_proto_in', which will be consumed by the messaging module.
* For outbound messages to the server, the messaging module will encode the
  proto message, publish an event 'messaging_proto_out_event' and this will be
  consumed by the cellular controller, which will send the binary message to
  the server and publish an acknowledge event (when receiving a response from
  the server).

**Next steps:

* Since http_downloader needs to access the modem's network interface; so
  slight re-arrangement needs to be made to avoid conflicts in socket
  management.
* Listening socket will be handled here, to notify the messaging module to
  send out a poll message on demand. This requires the +USOLI command to be
  implemented in the sara-r4 driver.

To build the real X3 app:

```
west build -b nf_x25_nrf52840
```

# Unit test with twister

Run the following command to run unit tests in folder 'twister_output'. The
./scripts/twister command path must be correct corresponding to your path of
where the twister script is located

```
./scripts/twister -T . --test  
tests/cellular_controller/cellular_controller.test -O twister_output
```

