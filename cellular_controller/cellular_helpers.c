/* tcp.c - TCP specific code for echo client */

/*
 * Copyright (c) 2017 Intel Corporation.
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_DECLARE(net_echo_client_sample, LOG_LEVEL_DBG);

#include <stdbool.h>

#include <zephyr.h>
#include <errno.h>
#include <stdio.h>

#include <net/socket.h>

#include "cellular_helpers.h"

#define RECV_BUF_SIZE 128

#define SOCKS5_PROXY_V4_ADDR CONFIG_NET_CONFIG_PEER_IPV4_ADDR
#define SOCKS5_PROXY_PORT 1080

static ssize_t sendall(int sock, const void *buf, size_t len)
{
    while (len) {
        ssize_t out_len = send(sock, buf, len, 0);

        if (out_len < 0) {
            return out_len;
        }
        buf = (const char *)buf + out_len;
        len -= out_len;
    }

    return 0;
}

static int socket_connect(struct data *data, struct sockaddr *addr,
                           socklen_t addrlen)
{
    int ret;

    data->tcp.sock = socket(addr->sa_family, SOCK_STREAM, IPPROTO_TCP);

    if (data->tcp.sock < 0) {
        LOG_ERR("Failed to create TCP socket (%s): %d", data->proto,
                errno);
        return -errno;
    }
    else
    {
        LOG_INF("Created TCP socket (%s): %d\n", data->proto, data->tcp.sock);
    }

    if (IS_ENABLED(CONFIG_SOCKS)) {
        struct sockaddr proxy_addr;
        socklen_t proxy_addrlen;

        if (addr->sa_family == AF_INET) {
            struct sockaddr_in *proxy4 =
                    (struct sockaddr_in *)&proxy_addr;

            proxy4->sin_family = AF_INET;
            proxy4->sin_port = htons(SOCKS5_PROXY_PORT);
            inet_pton(AF_INET, SOCKS5_PROXY_V4_ADDR,
                      &proxy4->sin_addr);
            proxy_addrlen = sizeof(struct sockaddr_in);
        }
        else {
            return -EINVAL;
        }

        ret = setsockopt(data->tcp.sock, SOL_SOCKET, SO_SOCKS5,
                         &proxy_addr, proxy_addrlen);
        if (ret < 0) {
            return ret;
        }
    }

    ret = connect(data->tcp.sock, addr, addrlen);
    if (ret < 0) {
        LOG_ERR("Cannot connect to TCP remote (%s): %d", data->proto,
                errno);
        ret = -errno;
    }

    return ret;
}

int start_tcp(void)
{
    int ret = 0;
    struct sockaddr_in addr4;

    if (IS_ENABLED(CONFIG_NET_IPV4)) {
        addr4.sin_family = AF_INET;
        addr4.sin_port = htons(PEER_PORT);
        inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR,
                  &addr4.sin_addr);

        ret = socket_connect(&conf.ipv4, (struct sockaddr *)&addr4,
                              sizeof(addr4));
        if (ret < 0) {
            return ret;
        }
    }
    return ret;
}

void stop_tcp(void)
{
    if (IS_ENABLED(CONFIG_NET_IPV6)) {
        if (conf.ipv6.tcp.sock >= 0) {
            (void)close(conf.ipv6.tcp.sock);
        }
    }

    if (IS_ENABLED(CONFIG_NET_IPV4)) {
        if (conf.ipv4.tcp.sock >= 0) {
            (void)close(conf.ipv4.tcp.sock);
        }
    }
}