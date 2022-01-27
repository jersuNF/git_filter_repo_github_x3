#include <zephyr.h>
#include "cellular_helpers_header.h"
#include "cellular_controller_events.h"
#include "messaging_module_events.h"

#ifndef  CONFIG_ZTEST
#define GSM_DEVICE DT_LABEL(DT_INST(0, u_blox_sara_r4))
#endif


#define MY_STACK_SIZE 1024
#define MY_PRIORITY 5
#define MODULE cellular_controller
LOG_MODULE_REGISTER(cellular_controller, LOG_LEVEL_DBG);

static bool messaging_ack = true;

int8_t socket_connect(struct data *, struct sockaddr *,
                          socklen_t);
//int8_t start_tcp(void);
//void stop_tcp(void);

uint8_t socket_receive(struct data *);
int8_t lteInit(void);
bool lteIsReady(void);

APP_DMEM struct configs conf = {
        .ipv4 = {
                .proto = "IPv4",
                .udp.sock = INVALID_SOCK,
                .tcp.sock = INVALID_SOCK,
        },
};

void submit_error(int8_t cause, int8_t err_code){
    struct cellular_error_event *err =
            new_cellular_error_event();
    err->cause = cause;
    err->err_code = err_code;
    EVENT_SUBMIT(err);
}

static APP_BMEM bool connected;

int8_t receive_tcp(struct data *sock_data)
{
    int8_t err, received;
    char buf[RECV_BUF_SIZE];
    uint8_t *pMsgIn = NULL;

    received = socket_receive(sock_data);
    if (received > 0)
    {
        LOG_WRN("received %d bytes!\n", received);

        if(messaging_ack == false) {/* TODO: notify the error handler */
            LOG_ERR("New message received while the messaging module "
                    "hasn't consumed the previous one!\n");
            err = -1;
            submit_error(OTHER, err);
            return err;
        }
        else{
            if (pMsgIn != NULL) {
                free(pMsgIn);
                pMsgIn = NULL;
            }

            pMsgIn = (uint8_t *) malloc(received);
            memcpy(pMsgIn, buf, received);
            messaging_ack = false;

            struct cellular_proto_in_event *msgIn =
                    new_cellular_proto_in_event();
            msgIn->buf = pMsgIn;
            msgIn->len = received;
            LOG_INF("Submitting msgIn event!\n");
            EVENT_SUBMIT(msgIn);
            return 0;
        }
    }
    else if (received < 0) {
        LOG_ERR("Socket receive error!\n");
        submit_error(SOCKET_RECV, received);
        return received;
    }
    else {
        return 0;
    }
    return -1;
}

int8_t start_tcp(void)
{
    int8_t ret = -1;
    struct sockaddr_in addr4;

    if (IS_ENABLED(CONFIG_NET_IPV4)) {
        addr4.sin_family = AF_INET;
        addr4.sin_port = htons(PEER_PORT);
        inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR,
                  &addr4.sin_addr);

        ret = socket_connect(&conf.ipv4, (struct sockaddr *)&addr4,
                             sizeof(addr4));
        if (ret < 0) {
            submit_error(SOCKET_CONNECT, ret);
            return ret;
        }
    }
    return ret;
}

static bool cellular_controller_event_handler(const struct event_header *eh)
{
    if (is_messaging_ack_event(eh))
    {
        messaging_ack = true;
        return true;
    }
    else if (is_messaging_stop_connection_event(eh))
    {
        stop_tcp();
        return true;
    }
    else if(is_messaging_proto_out_event(eh))
    {
        /* Accessing event data. */
        struct messaging_proto_out_event *event = cast_messaging_proto_out_event(eh);
        uint8_t *pCharMsgOut = event->buf;
        size_t MsgOutLen = event->len;

        int8_t err = start_tcp();
        if (err != 0){ /* TODO: notify error handler! */
            submit_error(SOCKET_CONNECT, err);
            return false;
        }

        /* make a local copy of the message to send.*/
        char *CharMsgOut;
        CharMsgOut = (char *) malloc(MsgOutLen);
        memcpy(CharMsgOut, pCharMsgOut, MsgOutLen);

        err = send_tcp(CharMsgOut, MsgOutLen);
        if (err != 0){ /* TODO: notify error handler! */
            submit_error(SOCKET_SEND, err);
            free(CharMsgOut);
            return false;
        }

        free(CharMsgOut);
        err = receive_tcp(&conf.ipv4);
        if (err != 0){ /* TODO: notify error handler! */
            submit_error(SOCKET_RECV, err);
            return false;
        }

        if(err == 0)
        {
            struct cellular_ack_event *ack = new_cellular_ack_event();
            EVENT_SUBMIT(ack);
            return true;
        }
    }
    return false;
}

int8_t cellular_controller_init(void)
{
    messaging_ack = true;
    int8_t ret;
    printk("Cellular controller starting!, %p\n", k_current_get());
    connected = false;
#ifndef  CONFIG_ZTEST
    const struct device *gsm_dev = device_get_binding(GSM_DEVICE);
    if (!gsm_dev) {
        LOG_ERR("GSM driver %s was not found!\n", GSM_DEVICE);
        return -1;
    }
#endif
    ret = lteInit();
    if (ret == 1)
    {
        LOG_INF("Cellular network interface ready!\n");

        if(lteIsReady()) {
            ret = start_tcp();
            if (ret == 0){
                LOG_INF("TCP connection started!\n");
                connected = true;
                return 0;
            }
            else{
                goto exit_cellular_controller;
            }
        }
        else{
            LOG_ERR("Check LTE network configuration!");
            /* TODO: notify error handler! */
            goto exit_cellular_controller;
        }
    }
    else{
        LOG_ERR("Failed to start LTE connection, check network interface!");
        /* TODO: notify error handler! */
        goto exit_cellular_controller;
    }

    exit_cellular_controller:
        stop_tcp();
        LOG_ERR("Cellular controller initialization failure!");
        return -1;
}

EVENT_LISTENER(MODULE, cellular_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, messaging_ack_event);
EVENT_SUBSCRIBE(MODULE, messaging_proto_out_event);
EVENT_SUBSCRIBE(MODULE, messaging_stop_connection_event);