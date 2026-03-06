# ============================================================
# ProjectOptions.cmake — All user-configurable build options
# ============================================================

# ------------------------------------
# Board selection
# ------------------------------------
# Uncomment one board (or pass -DBOARD_NAME=... on the cmake command line)
if(NOT DEFINED BOARD_NAME)
    #set(BOARD_NAME WIZ5XXSR_RP)
    set(BOARD_NAME W55RP20_S2E)
    #set(BOARD_NAME W232N)
    #set(BOARD_NAME IP20)
    #set(BOARD_NAME PLATYPUS_S2E)
endif()

if(${BOARD_NAME} STREQUAL WIZ5XXSR_RP)
    set(WIZCHIP W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
    add_definitions(-DDEVICE_BOARD_NAME=WIZ5XXSR_RP)
elseif(${BOARD_NAME} STREQUAL W55RP20_S2E)
    set(WIZNET_CHIP W5500)
    add_definitions(-D_WIZCHIP_=W5500)
    add_definitions(-DDEVICE_BOARD_NAME=W55RP20_S2E)
elseif(${BOARD_NAME} STREQUAL W232N)
    set(WIZNET_CHIP W5500)
    add_definitions(-D_WIZCHIP_=W5500)
    add_definitions(-DDEVICE_BOARD_NAME=W232N)
elseif(${BOARD_NAME} STREQUAL IP20)
    set(WIZNET_CHIP W5500)
    add_definitions(-D_WIZCHIP_=W5500)
    add_definitions(-DDEVICE_BOARD_NAME=IP20)
elseif(${BOARD_NAME} STREQUAL PLATYPUS_S2E)
    set(WIZNET_CHIP W5500)
    add_definitions(-D_WIZCHIP_=W5500)
    add_definitions(-DDEVICE_BOARD_NAME=PLATYPUS_S2E)
else()
    message(FATAL_ERROR "BOARD_NAME is wrong = ${BOARD_NAME}")
endif()

message(STATUS "BOARD_NAME  = ${BOARD_NAME}")
message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")

# ============================================================
# SEG Operation mode  (common.h: TCP_CLIENT_MODE=0 ... MQTTS_CLIENT_MODE=6)
# ============================================================
# Uncomment ONE line only:
# set(OPMODE "TCP_CLIENT_MODE"     CACHE STRING "SEG operation mode" FORCE)
set(OPMODE "TCP_SERVER_MODE"    CACHE STRING "SEG operation mode" FORCE)
#set(OPMODE "TCP_MIXED_MODE"     CACHE STRING "SEG operation mode" FORCE)
#set(OPMODE "UDP_MODE"           CACHE STRING "SEG operation mode" FORCE)
#set(OPMODE "SSL_TCP_CLIENT_MODE" CACHE STRING "SEG operation mode" FORCE)
#set(OPMODE "MQTT_CLIENT_MODE"   CACHE STRING "SEG operation mode" FORCE)
#set(OPMODE "MQTTS_CLIENT_MODE"  CACHE STRING "SEG operation mode" FORCE)

add_definitions(-DOPMODE=${OPMODE})
message(STATUS "OPMODE = ${OPMODE}")

# ============================================================
# Feature flags
# ============================================================
# set(ENABLE_SEGCP ON  CACHE BOOL "Enable config tool tasks (UDP/TCP/Serial)" FORCE)
set(ENABLE_SEGCP OFF CACHE BOOL "Enable config tool tasks (UDP/TCP/Serial)" FORCE)

if(ENABLE_SEGCP)
    add_definitions(-DENABLE_SEGCP)
endif()
message(STATUS "ENABLE_SEGCP = ${ENABLE_SEGCP}")

# ============================================================
# Serial protocol  (seg.h: SEG_SERIAL_PROTOCOL_NONE=0, RTU=1, ASCII=2)
# ============================================================
# Uncomment ONE line only:
set(SERIAL_MODE "SEG_SERIAL_PROTOCOL_NONE" CACHE STRING "Compile-time serial protocol selection" FORCE)
#set(SERIAL_MODE "SEG_SERIAL_MODBUS_RTU"   CACHE STRING "Compile-time serial protocol selection" FORCE)
#set(SERIAL_MODE "SEG_SERIAL_MODBUS_ASCII" CACHE STRING "Compile-time serial protocol selection" FORCE)

add_definitions(-DSERIAL_MODE=${SERIAL_MODE})
message(STATUS "SERIAL_MODE = ${SERIAL_MODE}")
