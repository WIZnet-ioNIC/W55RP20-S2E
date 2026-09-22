#ifndef PPPOEHANDLER_H_
#define PPPOEHANDLER_H_

#include <stdint.h>

#define PPPOE_RET_SUCCESS       0
#define PPPOE_RET_FAILED        (-1)
#define PPPOE_RET_NO_ACCOUNT    (-2)

// Runs the whole negotiation and, on success, leaves the chip in PPPoE mode with
// the assigned address in place. Blocks for as long as the far end takes.
int8_t process_pppoe(void);

void pppoe_disconnect(void);
uint8_t is_pppoe_connected(void);
uint8_t pppoe_link_lost(void);
void pppoe_get_assigned_ip(uint8_t *ip);

#endif /* PPPOEHANDLER_H_ */
