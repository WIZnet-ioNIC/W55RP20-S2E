# Ethernet
add_library(ETHERNET_FILES STATIC)

target_sources(ETHERNET_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        )

message(STATUS "Libraries WIZNET_CHIP = ${WIZNET_CHIP}")

if(${WIZNET_CHIP} STREQUAL W5100S)
target_include_directories(ETHERNET_FILES INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5100S
        )

target_link_libraries(ETHERNET_FILES PUBLIC
        W5100S_FILES
        )
        elseif((${WIZNET_CHIP} STREQUAL W5500) OR (${WIZNET_CHIP} STREQUAL W55RP20))
target_include_directories(ETHERNET_FILES INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5500
        )

target_link_libraries(ETHERNET_FILES PUBLIC
        W5500_FILES
        )
endif()

# W5100S
if(${WIZNET_CHIP} STREQUAL W5100S)
add_library(W5100S_FILES STATIC)

target_sources(W5100S_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet/W5100S/w5100s.c
        )

target_include_directories(W5100S_FILES INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5100S
        )

target_link_libraries(W5100S_FILES PRIVATE
        ETHERNET_FILES
        )

# W5500 or W55RP20
elseif((${WIZNET_CHIP} STREQUAL W5500) OR (${WIZNET_CHIP} STREQUAL W55RP20))
add_library(W5500_FILES STATIC)

target_sources(W5500_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet/W5500/w5500.c
        )

target_include_directories(W5500_FILES INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5500
        )

target_link_libraries(W5500_FILES PRIVATE
        ETHERNET_FILES
        )
endif()

# Loopback
add_library(LOOPBACK_FILES STATIC)

target_sources(LOOPBACK_FILES PUBLIC
        ${WIZNET_DIR}/Application/loopback/loopback.c
        )

target_include_directories(LOOPBACK_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Application/loopback
        )

# DHCP
add_library(DHCP_FILES STATIC)

target_sources(DHCP_FILES PUBLIC
        ${WIZNET_DIR}/Internet/DHCP/dhcp.c
        )

target_include_directories(DHCP_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/DHCP
        )

# DNS
add_library(DNS_FILES STATIC)

target_sources(DNS_FILES PUBLIC
        ${WIZNET_DIR}/Internet/DNS/dns.c
        )

target_include_directories(DNS_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/DNS
        )

# FTP Client
#add_library(FTPCLIENT_FILES STATIC)

#target_sources(FTPCLIENT_FILES PUBLIC
#        ${WIZNET_DIR}/Internet/FTPClient/ftpc.c
#        )

#target_include_directories(FTPCLIENT_FILES PUBLIC
#        ${WIZNET_DIR}/Ethernet
#        ${WIZNET_DIR}/Internet/FTPClient
#        )

# FTP Server
#add_library(FTPSERVER_FILES STATIC)

#target_sources(FTPSERVER_FILES PUBLIC
#        ${WIZNET_DIR}/Internet/FTPServer/ftpd.c
#        )

#target_include_directories(FTPSERVER_FILES PUBLIC
#        ${WIZNET_DIR}/Ethernet
#        ${WIZNET_DIR}/Internet/FTPServer
#        )

# HTTP Server
add_library(HTTPSERVER_FILES STATIC)

target_sources(HTTPSERVER_FILES PUBLIC
        ${WIZNET_DIR}/Internet/httpServer/httpParser.c
        )

target_include_directories(HTTPSERVER_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/httpServer

        )

# MQTT
add_library(MQTT_FILES STATIC)

target_sources(MQTT_FILES PUBLIC
        ${WIZNET_DIR}/Internet/MQTT/mqtt_interface.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTConnectClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTConnectServer.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTDeserializePublish.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTFormat.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTPacket.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTSerializePublish.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTSubscribeClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTSubscribeServer.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeClient.c
        ${WIZNET_DIR}/Internet/MQTT/MQTTPacket/src/MQTTUnsubscribeServer.c
        )

target_include_directories(MQTT_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/MQTT
        ${WIZNET_DIR}/Internet/MQTTPacket/src
        ) 

# SNTP
add_library(SNTP_FILES STATIC)

target_sources(SNTP_FILES PUBLIC
        ${WIZNET_DIR}/Internet/SNTP/sntp.c
        )

target_include_directories(SNTP_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Internet/SNTP
        )

include(${AWS_SDK_DIR}/libraries/standard/coreHTTP/httpFilePaths.cmake)
include(${AWS_SDK_DIR}/libraries/standard/coreMQTT/mqttFilePaths.cmake)

add_library(AWS_SDK_FILES STATIC)

target_sources(AWS_SDK_FILES PUBLIC
        ${HTTP_SOURCES}
        ${MQTT_SOURCES}
        ${MQTT_SERIALIZER_SOURCES}
        )

target_include_directories(AWS_SDK_FILES PUBLIC
        ${HTTP_INCLUDE_PUBLIC_DIRS}
        ${MQTT_INCLUDE_PUBLIC_DIRS}
        )
        