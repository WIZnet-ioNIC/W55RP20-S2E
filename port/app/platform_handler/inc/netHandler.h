#ifndef NETHANDLER_H_
#define NETHANDLER_H_

typedef enum
{
  NET_LINK_DISCONNECTED = 0,
  NET_LINK_CONNECTED,
  NET_IP_UP,
} NetStatus;

#define DHCP_RETRY_COUNT 5
NetStatus get_net_status (void);
void net_status_task(void *argument);

uint8_t set_stop_dhcp_flag(uint8_t flag);
uint8_t get_stop_dhcp_flag(void);
int8_t process_dhcp(void);


#endif /* NETHANDLER_H_ */
