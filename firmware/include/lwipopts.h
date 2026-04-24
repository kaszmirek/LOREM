#pragma once

// lwIP configuration for LOREM robot dashboard (no-OS, poll mode)

#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0

// Memory
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_TCP_SEG            16
#define MEMP_NUM_TCP_PCB            4
#define MEMP_NUM_UDP_PCB            2
#define PBUF_POOL_SIZE              8

// TCP
#define LWIP_TCP                    1
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_WND                     (2 * TCP_MSS)

// UDP
#define LWIP_UDP                    1

// Raw API
#define LWIP_RAW                    1

// DHCP client (used in STA mode to get IP from router)
#define LWIP_DHCP                   1

// No high-level APIs
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

// Netif callbacks (needed by cyw43 driver)
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1

// Save RAM
#define LWIP_STATS                  0
#define LWIP_DNS                    0
#define ARP_TABLE_SIZE              4
