#pragma once
#define MG_ARCH MG_ARCH_NEWLIB     // For all ARM GCC based environments
#define MG_ENABLE_TCPIP 1          // Enables built-in TCP/IP stack
#define MG_ENABLE_CUSTOM_MILLIS 1  // We must implement mg_millis()
#define MG_ENABLE_TCPIP_PRINT_DEBUG_STATS 1  // Enable debug stats log
#define MG_ENABLE_CUSTOM_RANDOM 1
#define MG_TCPIP_PHY_ADDR 0

// For static IP configuration, define MG_TCPIP_{IP,MASK,GW}
// By default, those are set to zero, meaning that DHCP is used
 #define MG_TCPIP_IP MG_IPV4(192, 168, 0, 10)     // IP
 #define MG_TCPIP_GW MG_IPV4(192, 168, 0, 1)      // Gateway
 #define MG_TCPIP_MASK MG_IPV4(255, 255, 255, 0)  // Netmask

// Uncomment the driver for your device
// #define MG_ENABLE_DRIVER_STM32H 1
 #define MG_ENABLE_DRIVER_STM32F 1

#define MG_ENABLE_PACKED_FS 1  // Enable "embedded", or packed, filesystem
#define MG_ENABLE_POSIX_FS 0   // Disable POSIX filesystem
