#ifndef BUFFERHANDLER_H_
#define BUFFERHANDLER_H_

#include <stdint.h>
#include "port_common.h"
#include "common.h"

#ifndef DATA_BUF_SIZE
    #define DATA_BUF_SIZE 2048
#endif

#define MEM_FREE(mem_p) do{ if(mem_p) { free(mem_p); mem_p = NULL; } }while(0)	//

#define BUFFER_DEFINITION(_name, _size) \
    uint8_t _name##_buf[_size]; \
    volatile uint16_t _name##_wr=0; \
    volatile uint16_t _name##_rd=0; \
    volatile uint16_t _name##_sz=_size;
#define BUFFER_DECLARATION(_name) \
    extern uint8_t _name##_buf[]; \
    extern uint16_t _name##_wr, _name##_rd, _name##_sz;
#define BUFFER_CLEAR(_name) \
    _name##_wr=0;\
    _name##_rd=0;

#define BUFFER_USED_SIZE(_name) ((_name##_sz + _name##_wr - _name##_rd) % _name##_sz)
#define BUFFER_FREE_SIZE(_name) ((_name##_sz + _name##_rd - _name##_wr - 1) % _name##_sz)
#define IS_BUFFER_EMPTY(_name) ( (_name##_rd) == (_name##_wr))
#define IS_BUFFER_FULL(_name) (BUFFER_FREE_SIZE(_name) == 0)	// I guess % calc takes time a lot, so...
//#define IS_BUFFER_FULL(_name) ((_name##_rd!=0 && _name##_wr==_name##_rd-1)||(_name##_rd==0 && _name##_wr==_name##_sz-1))

#define BUFFER_IN(_name) _name##_buf[_name##_wr]
#define BUFFER_IN_OFFSET(_name, _offset) _name##_buf[_name##_wr + _offset]
#define BUFFER_IN_MOVE(_name, _num) _name##_wr = (_name##_wr + _num) % _name##_sz
#define BUFFER_IN_1ST_SIZE(_name) (_name##_sz - _name##_wr - ((_name##_rd==0)?1:0))
#define BUFFER_IN_2ND_SIZE(_name) ((_name##_rd==0) ? 0 : _name##_rd-1)
#define IS_BUFFER_IN_SEPARATED(_name) (_name##_rd <= _name##_wr)

#define BUFFER_OUT(_name) _name##_buf[_name##_rd]
#define BUFFER_OUT_OFFSET(_name, _offset) _name##_buf[_name##_rd + _offset]
#define BUFFER_OUT_MOVE(_name, _num) _name##_rd = (_name##_rd + _num) % _name##_sz
#define BUFFER_OUT_1ST_SIZE(_name) (_name##_sz - _name##_rd)
#define BUFFER_OUT_2ND_SIZE(_name) (_name##_wr)
#define IS_BUFFER_OUT_SEPARATED(_name) (_name##_rd > _name##_wr)

#define BUFFER_PTR(_name) _name##_buf

void data_buffer_flush(void);
void put_byte_to_data_buffer(uint8_t ch);
uint16_t get_data_buffer_usedsize(void);
uint16_t get_data_buffer_freesize(void);
uint8_t *get_data_buffer_ptr(void);
int8_t is_data_buffer_empty(void);
int8_t is_data_buffer_full(void);
int32_t data_buffer_getc(void);
int32_t data_buffer_getc_nonblk(void);
int32_t data_buffer_gets(uint8_t* buf, uint16_t bytes);

#endif /* BUFFERHANDLER_H_ */
