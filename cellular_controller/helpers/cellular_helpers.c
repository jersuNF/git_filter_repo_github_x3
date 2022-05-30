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
#include <modem_nf.h>
#include "cellular_helpers_header.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(cellular_helpers, LOG_LEVEL_DBG);
static struct net_if *iface;
static struct net_if_config *cfg;
#define GSM_DEVICE DT_LABEL(DT_INST(0, u_blox_sara_r4))

int8_t lte_init(void)
{
	int rc = 0;

	/* wait for network interface to be ready */
	iface = net_if_get_default();
	if (iface == NULL) {
		LOG_ERR("Could not get iface (network interface)!");
		rc = -1;
		goto exit;
	}

	cfg = net_if_get_config(iface);
	if (cfg == NULL) {
		LOG_ERR("Could not get iface config!");
		rc = -2;
		goto exit;
	}
	return rc;

exit:
	return rc;
}

bool lte_is_ready(void)
{
	if (iface != NULL && cfg != NULL) {
		return true;
	} else {
		return false;
	}
}

static size_t sendall(int sock, const void *buf, size_t len)
{
	size_t to_send = len;
	while (len) {
		size_t out_len = send(sock, buf, len, 0);

		if (out_len < 0) {
			return out_len;
		}
		buf = (const char *)buf + out_len;
		len -= out_len;
	}
	return to_send;
}

int8_t socket_connect(struct data *data, struct sockaddr *addr,
		      socklen_t addrlen)
{
	int ret;

	data->tcp.sock = socket(addr->sa_family, SOCK_STREAM, IPPROTO_TCP);

	if (data->tcp.sock < 0) {
		LOG_ERR("Failed to create TCP socket (%s): %d", data->proto,
			errno);
		return -errno;
	} else {
		LOG_INF("Created TCP socket (%s): %d\n", data->proto,
			data->tcp.sock);
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
		} else {
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
	} else {
		ret = data->tcp.sock;
	}
	return ret;
}

int socket_listen(struct data *data)
{
	int ret;

	data->tcp.sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (data->tcp.sock < 0) {
		LOG_ERR("Failed to create TCP listening socket (%s): %d",
			data->proto, errno);
		return -errno;
	} else {
		LOG_INF("Created TCP listening socket (%s): %d\n", data->proto,
			data->tcp.sock);
	}

	k_sleep(K_MSEC(50));
	ret = listen(data->tcp.sock, 1); //2nd parameter is backlog size, not
	// important as we are not interested in the incoming data anyways.
	if (ret < 0) {
		LOG_ERR("Cannot start TCP listening socket (%s): %d",
			data->proto, errno);
		ret = -errno;
	} else {
		ret = data->tcp.sock;
	}
	return ret;
}

int socket_receive(struct data *data, char **msg)
{
	int received;
	static char buf[RECV_BUF_SIZE];
	received = recv(data->tcp.sock, buf, sizeof(buf), MSG_DONTWAIT);

	if (received > 0) {
		*msg = buf;
#if defined(CONFIG_CELLULAR_CONTROLLER_VERBOSE)
		LOG_DBG("Socket received %d bytes!\n", received);
		for (int i = 0; i < received; i++) {
			printk("\\x%02x", buf[i]);
		}
		printk("\n");
#endif
		return received;
	} else if (received < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return 0;
		} else {
			return -errno;
		}
	}
	return 0;
}

int reset_modem(void)
{
	return modem_nf_reset();
}

/**
 * Reads a 17 byte string potentially with an ipv4 address "xxx.xxx.xxx.xxx",
 * representing the ip address given to the sim card. The string will
 * have garbage bytes if the quoted ip address is shorter than 17 bytes. e.g:
 * for "10.12.225.223" we would have 2 garbage bytes.
 * @param collar_ip
 * @return
 */
int get_ip(char **collar_ip)
{ /*TODO: extract the quoted address if needed and return the exact length. */
	get_pdp_addr(collar_ip);
	return 0;
}

/**
 * will close the latest TCP socket with id greater than zero, as zero is
 * reserved for the listening socket. */
/* TODO: enhance robustness. */
void stop_tcp(void)
{
	if (IS_ENABLED(CONFIG_NET_IPV6)) {
		if (conf.ipv6.tcp.sock > 0) {
			(void)close(conf.ipv6.tcp.sock);
			memset(&conf, 0, sizeof(conf));
		}
	}

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		if (conf.ipv4.tcp.sock > 0) {
			(void)close(conf.ipv4.tcp.sock);
			memset(&conf, 0, sizeof(conf));
		}
	}
}

int8_t send_tcp(char *msg, size_t len)
{
	size_t ret;
	ret = sendall(conf.ipv4.tcp.sock, msg, len);
	if (ret < 0) {
		LOG_ERR("%s TCP: Failed to send data, errno %d",
			conf.ipv4.proto, ret);
	} else {
		LOG_DBG("%s TCP: Sent %d bytes", conf.ipv4.proto, ret);
	}
	/* TODO: how to handle partial sends? sendall() will keep retrying,
     * this should be handled here as well.*/
	return ret;
}

const struct device *bind_modem(void)
{
	return device_get_binding(GSM_DEVICE);
}

int check_ip(void)
{
	k_sleep(K_MSEC(500));
	char *collar_ip = NULL;
	uint8_t timeout_counter = 0;
	while (timeout_counter++ <= 40) {
		int ret = get_ip(&collar_ip);
		if (ret != 0) {
			LOG_ERR("Failed to get ip from sara r4 driver!");
			return -1;
			/*TODO: reset modem?*/
		} else {
			ret = memcmp(collar_ip, "\"0.0.0.0\"", 9);
			if (ret != 0) {
				return 0;
			}
		}
		k_sleep(K_MSEC(500));
	}
	LOG_ERR("Failed to acquire ip!");
	return -1;
};
