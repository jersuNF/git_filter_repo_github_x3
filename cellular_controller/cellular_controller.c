#include "cellular_controller_events.h"
#include "cellular_helpers_header.h"
#include "messaging_module_events.h"
#include <zephyr.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 30
#define SOCKET_POLL_INTERVAL 0.25
#define SOCK_RECV_TIMEOUT 60
#define MODULE cellular_controller
LOG_MODULE_REGISTER(cellular_controller, LOG_LEVEL_DBG);

static bool messaging_ack = true;

char server_address[EEP_HOST_PORT_BUF_SIZE];
static int server_port;
static char server_ip[15];

int8_t socket_connect(struct data *, struct sockaddr *, socklen_t);
uint8_t socket_receive(struct data *, char **);
int8_t lte_init(void);
bool lte_is_ready(void);

APP_DMEM struct configs conf = {
	.ipv4 = {
		.proto = "IPv4",
		.udp.sock = INVALID_SOCK,
		.tcp.sock = INVALID_SOCK,
	},
};

void submit_error(int8_t cause, int8_t err_code)
{
	struct cellular_error_event *err = new_cellular_error_event();
	err->cause = cause;
	err->err_code = err_code;
	EVENT_SUBMIT(err);
}

void receive_tcp(struct data *);
K_THREAD_DEFINE(my_tid, MY_STACK_SIZE,
		receive_tcp, &conf.ipv4, NULL, NULL,
		MY_PRIORITY, 0, 0);

static APP_BMEM bool connected;

void receive_tcp(struct data *sock_data)
{
	int8_t  received;
	char *buf = NULL;
	uint8_t *pMsgIn = NULL;
	static uint8_t socket_idle_count;
	while(1){
		k_sleep(K_SECONDS(SOCKET_POLL_INTERVAL));
		if(connected){
			socket_idle_count++;
			if (socket_idle_count >
			    SOCK_RECV_TIMEOUT/SOCKET_POLL_INTERVAL){
				connected = false;
				stop_tcp();
				socket_idle_count = 0;
			}
			received = socket_receive(sock_data, &buf);
			if (received == 0) {
//				return 0;
			} else if (received > 0) {
				socket_idle_count = 0;
				LOG_WRN("received %d bytes!\n", received);

				if (messaging_ack ==
				    false) { /* TODO: notify the error handler */
					LOG_ERR("New message received while the messaging module "
						"hasn't consumed the previous one!\n");
				} else {
					if (pMsgIn != NULL) {
						free(pMsgIn);
						pMsgIn = NULL;
					}
					pMsgIn = (uint8_t *)malloc(received);
					memcpy(pMsgIn, buf, received);
					messaging_ack = false;
					struct cellular_proto_in_event *msgIn =
						new_cellular_proto_in_event();
					msgIn->buf = pMsgIn;
					msgIn->len = received;
					LOG_INF("Submitting msgIn event!\n");
					EVENT_SUBMIT(msgIn);
				}
			} else {
				LOG_ERR("Socket receive error!\n");
				submit_error(SOCKET_RECV, received);
				connected = false;
				stop_tcp();
				socket_idle_count = 0;
			}
		}
	}
}

int8_t start_tcp(void)
{
	int8_t ret = -1;
	struct sockaddr_in addr4;
//	if (!connected){
//		stop_tcp();
//		k_sleep(K_SECONDS(0.1));
//	}

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		addr4.sin_family = AF_INET;
		addr4.sin_port = htons(server_port);
		inet_pton(AF_INET, server_ip, &addr4.sin_addr);

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
	if (is_messaging_ack_event(eh)) {
		messaging_ack = true;
		LOG_WRN("ACK received!\n");
		return true;
	} else if (is_messaging_stop_connection_event(eh)) {
		stop_tcp();
		connected = false;
		return true;
	} else if (is_messaging_proto_out_event(eh)) {
		/* Accessing event data. */
		struct messaging_proto_out_event *event =
			cast_messaging_proto_out_event(eh);
		uint8_t *pCharMsgOut = event->buf;
		size_t MsgOutLen = event->len;

/*TODO: cleanup this mess, we need to be aware of the socket '0' actual state of
 * cennection.*/

//		while (start_tcp() != 0) {
//			stop_tcp();
//			LOG_WRN("Retry socket connect!\n");
//			k_sleep(K_SECONDS(0.1));
//		}
		start_tcp();
		connected = true;
		/* make a local copy of the message to send.*/
		char *CharMsgOut;
		CharMsgOut = (char *)malloc(MsgOutLen);
		memcpy(CharMsgOut, pCharMsgOut, MsgOutLen);

		if (*CharMsgOut == *pCharMsgOut) {
			LOG_DBG("Publishing ack to messaging!\n");
			struct cellular_ack_event *ack =
				new_cellular_ack_event();
			EVENT_SUBMIT(ack);
		}

		int8_t err = send_tcp(CharMsgOut, MsgOutLen);
		if (err < 0) { /* TODO: notify error handler! */
			submit_error(SOCKET_SEND, err);
			free(CharMsgOut);
			return false;
		}
		free(CharMsgOut);
		return true;
	}
	return false;
}

/**
	 * Read full server address from eeprom and cache the ip as char array
	 * and the port as an unsigned int.
	 * @return: 0 on success, -1 on failure
	 */
int8_t cache_server_address(void)
{
	int err =
		eep_read_host_port(&server_address[0], sizeof(server_address));
	if (err != 0 || server_address[0] == '\0') {
		return -1;
	}
	char *ptr_port;
	ptr_port = strchr(server_address, ':') + 1;
	server_port = atoi(ptr_port);
	if (server_port <= 0) {
		return -1;
	}
	uint8_t ip_len;
	ip_len = ptr_port - 1 - &server_address[0];
	memcpy(&server_ip[0], &server_address[0], ip_len);
	if (server_ip[0] != '\0') {
		LOG_DBG("Host address read from eeprom: %s : %d", &server_ip[0],
			server_port);
		return 0;
	} else {
		return -1;
	}
}

int8_t cellular_controller_init(void)
{
	messaging_ack = true;
	int8_t ret;
	printk("Cellular controller starting!, %p\n", k_current_get());
	connected = false;

	const struct device *gsm_dev = bind_modem();
	if (gsm_dev == NULL) {
		LOG_ERR("GSM driver was not found!\n");
		return -1;
	}
	ret = lte_init();
	if (ret == 1) {
		LOG_INF("Cellular network interface ready!\n");

		if (lte_is_ready()) {
			ret = cache_server_address();
			if (ret == 0) {
				LOG_INF("Server ip address successfully cached from "
					"eeprom!\n");
			} else { //172.31.36.11:4321
				//172.31.33.243:9876
				strcpy(server_ip,"172.31.36.11");
				server_port = 4321;
				LOG_WRN("Default server ip address will be "
					"used. \n");
//				goto exit_cellular_controller;
			}
			ret = 0;//start_tcp();
			if (ret == 0) {
				LOG_INF("TCP connection started!\n");
				connected = true;
				return 0;
			} else {
				goto exit_cellular_controller;
			}
		} else {
			LOG_ERR("Check LTE network configuration!");
			/* TODO: notify error handler! */
			goto exit_cellular_controller;
		}
	} else {
		LOG_ERR("Failed to start LTE connection, check network interface!");
		/* TODO: notify error handler! */
		goto exit_cellular_controller;
	}

exit_cellular_controller:
	stop_tcp();
	connected = false;
	LOG_ERR("Cellular controller initialization failure!");
	return -1;
}

EVENT_LISTENER(MODULE, cellular_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, messaging_ack_event);
EVENT_SUBSCRIBE(MODULE, messaging_proto_out_event);
EVENT_SUBSCRIBE(MODULE, messaging_stop_connection_event);
