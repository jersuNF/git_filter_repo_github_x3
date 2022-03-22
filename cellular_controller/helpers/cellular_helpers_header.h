#include <stdlib.h>
#include <net/net_ip.h>
#include <autoconf.h>
#include <device.h>
#include <devicetree.h>
#include <sys/printk.h>
#include "event_manager.h"
#include <logging/log.h>
#include <stdbool.h>
#include <errno.h>
#include <net/net_if.h>
#include <net/net_event.h>
#include <net/socket.h>
#include "nf_eeprom.h"
#define INVALID_SOCK (-1)
#define PEER_PORT CONFIG_SERVER_PORT
#define RECV_BUF_SIZE CONFIG_RECV_BUF_MAX

#define SOCKS5_PROXY_V4_ADDR ""
#define SOCKS5_PROXY_PORT 1080

#if defined(CONFIG_USERSPACE)
#include <app_memory/app_memdomain.h>

extern struct k_mem_partition app_partition;
extern struct k_mem_domain app_domain;
#define APP_BMEM K_APP_BMEM(app_partition)
#define APP_DMEM K_APP_DMEM(app_partition)
#else
#define APP_BMEM
#define APP_DMEM
#endif

#if IS_ENABLED(CONFIG_NET_TC_THREAD_PREEMPTIVE)
#define THREAD_PRIORITY K_PRIO_PREEMPT(8)
#else
#define THREAD_PRIORITY K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES - 1)
#endif

/**
     * A structure for socket meta data in addition to work delayables for
     * udp sockets -only TCP sockets are used for now
     */
struct data {
	const char *proto;

	struct {
		int sock;
		/* Work controlling udp data sending */
		struct k_work_delayable recv;
		struct k_work_delayable transmit;
		uint32_t expecting;
		uint32_t counter;
		uint32_t mtu;
	} udp;

	struct {
		int sock;
		uint32_t expecting;
		uint32_t received;
		uint32_t counter;
	} tcp;
};

struct configs {
	struct data ipv4;
	struct data ipv6;
};

/**
     * A structure for socket initialization
     */
extern struct configs conf;

int8_t send_tcp(char *, size_t);
void stop_tcp(void);
const struct device *bind_modem(void);
