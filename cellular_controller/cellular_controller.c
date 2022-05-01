#include "cellular_controller.h"
#include "cellular_helpers_header.h"
#include "messaging_module_events.h"
#include <zephyr.h>
#include "nf_eeprom.h"
#include "error_event.h"
#define RCV_THREAD_STACK CONFIG_RECV_THREAD_STACK_SIZE
#define MY_PRIORITY CONFIG_RECV_THREAD_PRIORITY
#define SOCKET_POLL_INTERVAL 0.25
#define SOCK_RECV_TIMEOUT 15
#define MODULE cellular_controller
#define MESSAGING_ACK_TIMEOUT CONFIG_MESSAGING_ACK_TIMEOUT_SEC
#include <modem_nf.h>

LOG_MODULE_REGISTER(cellular_controller, LOG_LEVEL_DBG);

K_SEM_DEFINE(messaging_ack, 1, 1);

K_THREAD_DEFINE(send_tcp_from_q, CONFIG_SEND_THREAD_STACK_SIZE,
		send_tcp_fn, NULL, NULL, NULL,
		CONFIG_SEND_THREAD_PRIORITY, 0, 0);

char server_address[EEP_HOST_PORT_BUF_SIZE-1];
char server_address_tmp[EEP_HOST_PORT_BUF_SIZE-1];
static int server_port;
static char server_ip[15];

/* Connection keep-alive thread structures */
K_KERNEL_STACK_DEFINE(keep_alive_stack,
		      CONFIG_CELLULAR_KEEP_ALIVE_STACK_SIZE);
struct k_thread keep_alive_thread;
static struct k_sem connection_state_sem;

int8_t socket_connect(struct data *, struct sockaddr *, socklen_t);
int socket_receive(struct data *, char **);
int8_t lte_init(void);
bool lte_is_ready(void);

static bool modem_is_ready = false;

APP_DMEM struct configs conf = {
	.ipv4 = {
		.proto = "IPv4",
		.udp.sock = INVALID_SOCK,
		.tcp.sock = INVALID_SOCK,
	},
};

void receive_tcp(struct data *);
K_THREAD_DEFINE(recv_tid, RCV_THREAD_STACK,
		receive_tcp, &conf.ipv4, NULL, NULL,
		MY_PRIORITY, 0, 0);

static bool connected = false;

void receive_tcp(struct data *sock_data)
{
	int  received;
	char *buf = NULL;
	uint8_t *pMsgIn = NULL;
	static float socket_idle_count;
	while(1){
		if(connected){
			received = socket_receive(sock_data, &buf);
			if (received > 0) {
				socket_idle_count = 0;
#if defined(CONFIG_CELLULAR_CONTROLLER_VERBOSE)
				LOG_WRN("received %d bytes!\n", received);
#endif
				LOG_WRN("will take semaphore!\n");
				if (k_sem_take(&messaging_ack, K_SECONDS
					       (MESSAGING_ACK_TIMEOUT)) !=0) {
					char *e_msg = "Missed messaging ack!";
					nf_app_error(ERR_MESSAGING, -EINPROGRESS, e_msg, strlen
						(e_msg));
				} else {
					if (pMsgIn != NULL) {
						k_free(pMsgIn);
						pMsgIn = NULL;
					}
					pMsgIn = (uint8_t *) k_malloc(received);
					memcpy(pMsgIn, buf, received);
					struct cellular_proto_in_event *msgIn =
						new_cellular_proto_in_event();
					msgIn->buf = pMsgIn;
					msgIn->len = received;
					LOG_INF("Submitting msgIn event!\n");
					EVENT_SUBMIT(msgIn);
				}
			} else if (received == 0) {
				socket_idle_count += SOCKET_POLL_INTERVAL;
				if (socket_idle_count > SOCK_RECV_TIMEOUT){
					char *e_msg = "Socket receive timed out!";
					nf_app_error(ERR_MESSAGING, -ETIMEDOUT,
						     e_msg, strlen
						(e_msg));
					stop_tcp();
					connected = false;
					socket_idle_count = 0;
				}
			} else {
				char *e_msg = "Socket receive error!";
				nf_app_error(ERR_MESSAGING, -EIO, e_msg, strlen
					(e_msg));
				stop_tcp();
				connected = false;
				socket_idle_count = 0;
			}
		}
		k_sleep(K_SECONDS(SOCKET_POLL_INTERVAL));
	}
}

int start_tcp(void)
{
	int ret = modem_nf_wakeup();
	if (ret != 0) {
		LOG_ERR("Failed to wake up the modem!");
		modem_nf_reset();
		return ret;
	}
	ret = check_ip();
	if (ret != 0){
		LOG_ERR("Failed to get ip address!");
		char *e_msg = "Failed to get ip address!";
		nf_app_error(ERR_MESSAGING, -EIO, e_msg, strlen(e_msg));
		return ret;
	}
	struct sockaddr_in addr4;

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		addr4.sin_family = AF_INET;
		addr4.sin_port = htons(server_port);
		inet_pton(AF_INET, server_ip, &addr4.sin_addr);

		ret = socket_connect(&conf.ipv4, (struct sockaddr *)&addr4,
				     sizeof(addr4));
		if (ret < 0) {
			char *e_msg = "Socket connect error!";
			nf_app_error(ERR_MESSAGING, -ECONNREFUSED, e_msg, strlen
				(e_msg));
			return ret;
		}
	}
	return ret;
}

static bool cellular_controller_event_handler(const struct event_header *eh)
{
	if (is_messaging_ack_event(eh)) {
		k_sem_give(&messaging_ack);
		LOG_WRN("ACK received!\n");
		return true;
	} else if (is_messaging_stop_connection_event(eh)) {
		stop_tcp();
		connected = false;
		return true;
	}else if (is_messaging_host_address_event(eh)) {
		int ret = eep_read_host_port(&server_address_tmp[0], EEP_HOST_PORT_BUF_SIZE-1);
		if (ret != 0){
			LOG_ERR("Failed to read host address from eeprom!\n");
		}
		struct messaging_host_address_event *event =
			cast_messaging_host_address_event(eh);
		memcpy(&server_address[0], event->address,
		       sizeof(event->address)-1);
		char *ptr_port;
		ptr_port = strchr(server_address, ':') + 1;
		server_port = atoi(ptr_port);
		if (server_port <= 0) {
			char *e_msg = "Failed to parse port number from new "
				      "host address!";
			nf_app_error(ERR_MESSAGING, -EIO, e_msg, strlen
				     (e_msg));
			return false;
		}
		uint8_t ip_len;
		ip_len = ptr_port - 1 - &server_address[0];
		memcpy(&server_ip[0], &server_address[0], ip_len);
		ret = memcmp(server_address, server_address_tmp, EEP_HOST_PORT_BUF_SIZE-1);
		if (ret != 0){
			LOG_INF("New host address received!\n");
			ret = eep_write_host_port(server_address);
			if (ret != 0){
				LOG_ERR("Failed to write new host address to "
					"eeprom!\n");
			}
		}
		return false;
	}else if (is_messaging_proto_out_event(eh)) {
		/* Accessing event data. */
		struct messaging_proto_out_event *event =
			cast_messaging_proto_out_event(eh);
		uint8_t *pCharMsgOut = event->buf;
		size_t MsgOutLen = event->len;
		
		/* make a local copy of the message to send.*/
		uint8_t *CharMsgOut;
		CharMsgOut = (char *) k_malloc(MsgOutLen);
		if (CharMsgOut == memcpy(CharMsgOut, pCharMsgOut, MsgOutLen)) {
			LOG_DBG("Publishing ack to messaging!\n");
			struct cellular_ack_event *ack =
				new_cellular_ack_event();
			EVENT_SUBMIT(ack);
		}

		int err = send_tcp_q(CharMsgOut, MsgOutLen);
		if (err != 0) {
			char *e_msg = "Couldn't push message to queue!";
			nf_app_error(ERR_MESSAGING, -EAGAIN, e_msg, strlen
				(e_msg));
			k_free(CharMsgOut);
			return false;
		}
		k_free(CharMsgOut);
		return false;
	}else if(is_check_connection(eh)){
		k_sem_give(&connection_state_sem);
		return false;
	} else if (is_cellular_error_event(eh)) {
		modem_is_ready = false;
		connected = false;
		return false;
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
		LOG_INF("Host address read from eeprom: %s : %d", &server_ip[0],
			server_port);
		return 0;
	} else {
		return -1;
	}
}

static int cellular_controller_connect(void* dev)
{
	int ret = lte_init();
	if (ret != 0) {
		LOG_ERR("Failed to start LTE connection, "
			"check network interface!");
		goto exit;
	}

	LOG_INF("Cellular network interface ready!\n");

	ret = cache_server_address();
	if (ret != 0) { //172.31.36.11:4321
		//172.31.33.243:9876
		strcpy(server_ip,"172.31.36.11");
		server_port = 4321;
		LOG_INF("Default server ip address will be "
			"used.");
	}

	ret = 0;

exit:
	return ret;
}

static void cellular_controller_keep_alive(void* dev)
{
	while (true) {
		if (k_sem_take(&connection_state_sem, K_FOREVER) == 0) {
			if (!cellular_controller_is_ready()) {
				int ret = reset_modem();
				if (ret == 0) {
					ret = cellular_controller_connect(dev);
					if (ret == 0) {
						modem_is_ready = true;
					}
				}
			}
			if (cellular_controller_is_ready()) {
				if(!connected){//check_ip
					// takes place in start_tcp() in this
					// case.
					int  ret = start_tcp();
					if (ret == 0){
						connected = true;
						announce_connection_state(true);
					}else {
//						stop_tcp();
						announce_connection_state
							(false);
					}
				} else {
					int ret = modem_nf_wakeup();
					if (ret != 0) {
						LOG_ERR("Failed to wake up the modem!");
						modem_nf_reset();
					}
					ret = check_ip();
					if (ret != 0){
						announce_connection_state
							(false);
						char *e_msg = "Failed to get ip address!";
						nf_app_error(ERR_MESSAGING,
							     -EIO, e_msg, strlen
							(e_msg));
					}else {
						announce_connection_state(true);
					}
				}
			}
		}
	}
}

void announce_connection_state(bool state){
	struct connection_state_event *ev
		= new_connection_state_event();
	ev->state = state;
	EVENT_SUBMIT(ev);
	if (state == false){
		modem_is_ready = false;
		connected = false;
	}
}

bool cellular_controller_is_ready(void)
{
	return modem_is_ready;
}

int8_t cellular_controller_init(void)
{
	connected = false;
	
	const struct device *gsm_dev = bind_modem();
	if (gsm_dev == NULL) {
		char *e_msg = "GSM driver was not found!";
		nf_app_error(ERR_MESSAGING, -EBUSY, e_msg, strlen
			(e_msg));
		return -1;
	}

	/* Start connection keep-alive thread */
	modem_is_ready = false;
	k_sem_init(&connection_state_sem, 0, 1);
	k_thread_create(&keep_alive_thread, keep_alive_stack,
			K_KERNEL_STACK_SIZEOF(keep_alive_stack),
			(k_thread_entry_t) cellular_controller_keep_alive,
			(void*)gsm_dev, NULL, NULL, 
			K_PRIO_COOP(CONFIG_CELLULAR_KEEP_ALIVE_THREAD_PRIORITY),
			0, K_NO_WAIT);

	return 0;
}

EVENT_LISTENER(MODULE, cellular_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, messaging_ack_event);
EVENT_SUBSCRIBE(MODULE, messaging_proto_out_event);
EVENT_SUBSCRIBE(MODULE, messaging_stop_connection_event);
EVENT_SUBSCRIBE(MODULE, messaging_host_address_event);
EVENT_SUBSCRIBE(MODULE, check_connection);
EVENT_SUBSCRIBE(MODULE, cellular_error_event);