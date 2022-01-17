//common.h -begin

#define MY_PORT 0
#define CONFIG_NET_SAMPLE_SEND_ITERATIONS 100
#define PEER_PORT CONFIG_SERVER_PORT

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

#if !defined(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
#define CONFIG_NET_CONFIG_PEER_IPV4_ADDR ""
#endif

extern struct configs conf;

int start_tcp(void);
int process_tcp(void);
void stop_tcp(void);
