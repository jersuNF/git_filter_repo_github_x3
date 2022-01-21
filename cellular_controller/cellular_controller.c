#include <zephyr.h>
#include "cellular_helpers_header.h"
#include "cellular_controller_events.h"


#define GSM_DEVICE DT_LABEL(DT_INST(0, u_blox_sara_r4))

#define INVALID_SOCK (-1)
#define MY_STACK_SIZE 1024
#define MY_PRIORITY 5
#define MODULE cellular_controller
LOG_MODULE_REGISTER(cellular_controller, LOG_LEVEL_DBG);

/******************************************************************************/
/* Local Data Definitions                                                     */
/******************************************************************************/
static struct net_if *iface;
static struct net_if_config *cfg;

static bool ack = true;

void submit_error(int8_t cause, int8_t err_code){
    struct cellular_error_event *err =
            new_cellular_error_event();
    err->cause = cause;
    err->err_code = err_code;
    EVENT_SUBMIT(err);
}

int8_t lteInit(void)
{
    int rc = 1;

    /* wait for network interface to be ready */
    iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("Could not get iface (network interface)!");
        rc = -1;
        goto exit;
    }

    cfg = net_if_get_config(iface);
    if (!cfg) {
        LOG_ERR("Could not get iface config!");
        rc = -2;
        goto exit;
    }
    return rc;

    exit:
    return rc;
}

bool lteIsReady(void)
{
    if (iface != NULL && cfg != NULL) {
        return true;
    }
    else{
        return false;
    }
}

APP_DMEM struct configs conf = {
        .ipv4 = {
                .proto = "IPv4",
                .udp.sock = INVALID_SOCK,
                .tcp.sock = INVALID_SOCK,
        },
};

static APP_BMEM bool connected;

uint8_t receive_tcp(struct data *data)
{
    int err, received;
    char buf[RECV_BUF_SIZE];
    uint8_t *pMsgIn;

    do {
        received = recv(data->tcp.sock, buf, sizeof(buf), MSG_DONTWAIT);
        if (received > 0)
        {
            LOG_WRN("received %d bytes!\n", received);

            if(!ack) {/* TODO: notify the error handler */
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
                ack = false;

                struct cellular_proto_in_event *msgIn =
                        new_cellular_proto_in_event();
                msgIn->buf = pMsgIn;
                msgIn->len = received;
                EVENT_SUBMIT(msgIn);
            }
        }
        else if (received < 0) {
            LOG_ERR("Socket receive error!\n");
            submit_error(SOCKET_RECV, received);
            return received;
        }
        else {
            continue;
        }
    } while (lteIsReady());

    submit_error(CONNECTION_LOST, -1);
    return -1;
}

int8_t start_tcp(void)
{
    int8_t ret = 0;
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
        receive_tcp(&conf.ipv4);
    }
    return ret;
}

static bool cellular_controller_event_handler(const struct event_header *eh)
{
    if (is_messaging_ack_event(eh)) {
        ack = true;
        return true;
    }
    else if(is_messaging_proto_out_event(eh)){
        /* Accessing event data. */
        struct messaging_proto_out_event *event = cast_messaging_proto_out_event(eh);
        uint8_t *pCharMsgOut = event->buf;
        size_t MsgOutLen = event->len;

        /* make a local copy of the message to send.*/
        char *CharMsgOut;
        CharMsgOut = (char *) malloc(MsgOutLen);
        memcpy(CharMsgOut, pCharMsgOut, MsgOutLen);

        int8_t err = send_tcp(CharMsgOut, MsgOutLen);

        if(err == 0) {
            struct cellular_ack_event *ack = new_cellular_ack_event();
            EVENT_SUBMIT(ack);
            free(CharMsgOut);
            return true;
        }
        else{ /* TODO: notify error handler! */
            submit_error(SOCKET_SEND, err);
            free(CharMsgOut);
            return false;
        }
    }
    return false;
}

int cellular_controller_init(void)
{
    int8_t ret;
    printk("Cellular controller starting!, %p\n", k_current_get());
    const struct device *gsm_dev = device_get_binding(GSM_DEVICE);
    if (!gsm_dev) {
        LOG_ERR("GSM driver %s was not found!\n", GSM_DEVICE);
        return -1;
    }

    ret = lteInit();
    if (ret == 1)
    {
        LOG_INF("Cellular network interface ready!\n");
        connected = true;
        if(lteIsReady()) {
            ret = start_tcp();
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
        return -1;
}

EVENT_LISTENER(MODULE, cellular_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, messaging_ack_event);
EVENT_SUBSCRIBE(MODULE, messaging_proto_out_event);