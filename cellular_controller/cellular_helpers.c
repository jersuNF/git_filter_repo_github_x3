#include <stdbool.h>
#include <stdlib.h>
#include <zephyr.h>
#include <errno.h>
#include <stdio.h>
#include <net/net_if.h>
#include <net/net_mgmt.h>
#include <net/net_event.h>
#include <net/net_conn_mgr.h>
#include <net/socket.h>
#include "cellular_helpers_header.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(cellular_helpers, LOG_LEVEL_DBG);

static size_t sendall(int sock, const void *buf, size_t len)
{
    while (len) {
        size_t out_len = send(sock, buf, len, 0);

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

void stop_tcp(void)
{
    if (IS_ENABLED(CONFIG_NET_IPV6)) {
        if (conf.ipv6.tcp.sock >= 0) {
            (void)close(conf.ipv6.tcp.sock);
            memset(&conf, 0, sizeof(conf));
        }
    }

    if (IS_ENABLED(CONFIG_NET_IPV4)) {
        if (conf.ipv4.tcp.sock >= 0) {
            (void)close(conf.ipv4.tcp.sock);
            memset(&conf, 0, sizeof(conf));
        }
    }
}

int8_t send_tcp(char* msg, size_t len){
    size_t ret;
    ret = sendall(conf.ipv4.tcp.sock, msg, len);
    if (ret < 0) {
        LOG_ERR("%s TCP: Failed to send data, errno %d", conf.ipv4.proto,
                ret);
    } else {
        LOG_DBG("%s TCP: Sent %d bytes", conf.ipv4.proto,
                ret);
    }
    /* TODO: how to handle partial sends? sendall() will keep retrying,
     * this should be handled here as well.*/
    return ret;
}