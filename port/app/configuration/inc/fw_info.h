#ifndef _FW_INFO_H_
#define _FW_INFO_H_

#include <stdint.h>

/*
 * Firmware Info Block
 *
 * Located at flash offset 0x100 (immediately after 2nd stage bootloader).
 * This block is embedded in every firmware binary and can be read
 * without running the firmware (e.g. during OTA before flashing).
 *
 * Flash map:
 *   0x00000000  [0x100]  2nd stage bootloader
 *   0x00000100  [0x100]  fw_info_t  <-- here
 *   0x00000200  [...]    vector table + code
 */

#define FW_INFO_MAGIC       0x57495A4E  /* 'WIZN' */
#define FW_INFO_OFFSET      0x100       /* bytes from start of binary */

typedef struct __attribute__((packed)) {
    uint32_t magic;             /* FW_INFO_MAGIC - validity check              */
    uint8_t  major;             /* Major version                               */
    uint8_t  minor;             /* Minor version                               */
    uint8_t  patch;             /* Patch version                               */
    uint8_t  reserved;
    char     version_str[16];   /* e.g. "1.2.1"                                */
    char     board[16];         /* e.g. "PLATYPUS_S2E"                         */
    char     manufacturer[16];  /* e.g. "WIZnet"                               */
    char     status[12];        /* e.g. "Stable" / "Develop"                   */
    uint32_t build_date;        /* BCD: 0x20260403                             */
} fw_info_t;

/* Placed at a fixed section, linker assigns it to FW_INFO_OFFSET */
extern const fw_info_t fw_info;

#endif /* _FW_INFO_H_ */
