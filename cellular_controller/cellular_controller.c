#include "cellular_helpers.h"
#include "cellular_controller_events.h"


/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate cellular communication over Ublox SARA-R4(22) modem.
 */

#include <autoconf.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <stdio.h>
#include <sys/printk.h>
#include <zephyr.h>

#include <net/net_if.h>
#include <net/net_mgmt.h>
#include <net/net_event.h>
#include <net/net_conn_mgr.h>
#include <net/socket.h>

#include "cellular_helpers.h"

//LOG_MODULE_REGISTER(main);
LOG_MODULE_REGISTER(cellular_controller, LOG_LEVEL_DBG);
#define GSM_DEVICE DT_LABEL(DT_INST(0, ublox_sara_r4))
#define APP_BANNER "Starting cellular_controller"

#define INVALID_SOCK (-1)
#define MY_STACK_SIZE 1024
#define MY_PRIORITY 5

/******************************************************************************/
/* Local Data Definitions                                                     */
/******************************************************************************/
static struct net_if *iface;
static struct net_if_config *cfg;

static bool ack = true;

int lteInit(void)
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
#ifdef CONFIG_DNS_RESOLVER
    struct sockaddr_in *dnsAddr;

	if (iface != NULL && cfg != NULL && &dns->servers[0] != NULL) {
		dnsAddr = net_sin(&dns->servers[0].dns_server);
		return net_if_is_up(iface) && cfg->ip.ipv4 &&
		       !net_ipv4_is_addr_unspecified(&dnsAddr->sin_addr);
	}
#else
    if (iface != NULL && cfg != NULL) {
        return net_if_is_up(iface) && cfg->ip.ipv4;
    }
#endif /* CONFIG_DNS_RESOLVER */
    return false;
}

APP_DMEM struct configs conf = {
        .ipv4 = {
                .proto = "IPv4",
                .udp.sock = INVALID_SOCK,
                .tcp.sock = INVALID_SOCK,
        },
};

static APP_BMEM bool connected;

void cellular_controller_init(void)
{
    int_least8_t ret;
    printk("Cellular controller starting!, %p\n", k_current_get());
    const struct device *gsm_dev = device_get_binding(GSM_DEVICE);
    if (!gsm_dev) {
        LOG_ERR("GSM driver %s was not found!\n", GSM_DEVICE);
        return;
    }

    ret = lteInit();
    if (ret == 1)
    {
        LOG_INF("Cellular network interface ready!\n");
        connected = true;
        if(lteIsReady) {
            start_tcp();
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

    /* start the receiving thread */
    K_THREAD_DEFINE(rcv_tid, MY_STACK_SIZE,
                    receive_tcp, &conf.ipv4,
                    MY_PRIORITY, 0, 0);

    EVENT_LISTENER(cellular_controller, cellular_controller_event_handler);
    EVENT_SUBSCRIBE(cellular_controller, messaging_ack_event);
    EVENT_SUBSCRIBE(cellular_controller, messaging_proto_out_event);

    exit_cellular_controller;
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
            struct cellular_ack_event *err = new_cellular_sending_error_event();
            EVENT_SUBMIT(err);
            free(CharMsgOut);
            return false;
        }
    }

    return false;
}

static int send_tcp(char* msg, size_t len){
    uint8_t attempts = 0;
    ret = sendall(data->tcp.sock, msg, len);
    if (ret < 0) {
        LOG_ERR("%s TCP: Failed to send data, errno %d", data->proto,
                errno);
    } else {
        LOG_DBG("%s TCP: Sent %d bytes", data->proto,
                ret);
    }
    /* TODO: how to handle partial sends? sendall() will keep retrying,
     * this should be handled here as well.*/
    return ret;
}

static int receive_tcp(struct data *data)
{
    int ret, received;
    char buf[RECV_BUF_SIZE];
    static int total_sends;
    uint8_t *pMsgIn;

    do {
        received = recv(data->tcp.sock, buf, sizeof(buf), MSG_DONTWAIT);
        if (received > 0)
        {
            LOG_WRN("received %d bytes!\n", received);

            if(!ack) {/* TODO: notify the error handler */
                LOG_ERR("New message received while the messaging module "
                        "hasn't consumed the previous one!\n");
                ret = -1;
                break;
            }
            else{
                if (pMsgIn) {
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
            ret = -1;
            break;
            //TODO: handle the error (reset socket?)
        }
        else {
            continue;
        }
    } while (1);

    return ret;
}
