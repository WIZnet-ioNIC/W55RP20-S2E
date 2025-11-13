#ifndef STORAGEHANDLER_H_
#define STORAGEHANDLER_H_

#include <stdint.h>

typedef enum {
    STORAGE_MAC,
    STORAGE_CONFIG,
    STORAGE_APPBOOT,
    STORAGE_APPBANK,
    STORAGE_BINBANK,
    STORAGE_ROOTCA0,
    STORAGE_CLICA0,
    STORAGE_PKEY0,
    STORAGE_ROOTCA1,
    STORAGE_CLICA1,
    STORAGE_PKEY1
} teDATASTORAGE;



void read_storage(teDATASTORAGE stype, void *data, uint16_t size);
void write_storage(teDATASTORAGE stype, uint32_t addr, void *data, uint16_t size);
void erase_storage(teDATASTORAGE stype);

#endif /* STORAGEHANDLER_H_ */
