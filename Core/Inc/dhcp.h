#ifndef DHCP_H
#define DHCP_H

#include <stdint.h>

// =============================================================================
// SOCKET DHCP
// =============================================================================
#define DHCP_SOCKET         1           // Socket 1 untuk DHCP (Socket 0 = HTTP)
#define DHCP_CLIENT_PORT    68
#define DHCP_SERVER_PORT    67

// =============================================================================
// DHCP STATE
// =============================================================================
#define DHCP_STATE_IDLE         0
#define DHCP_STATE_DISCOVER     1
#define DHCP_STATE_OFFER        2
#define DHCP_STATE_REQUEST      3
#define DHCP_STATE_ACK          4
#define DHCP_STATE_BOUND        5       // Sudah dapat IP, jalan normal
#define DHCP_STATE_RENEWING     6       // Sedang renewal
#define DHCP_STATE_LINK_DOWN    7       // Kabel dicabut

// =============================================================================
// TIMING
// =============================================================================
#define DHCP_TIMEOUT_MS         5000    // Timeout tiap step (ms)
#define DHCP_RETRY_MAX          3       // Max retry sebelum mulai ulang

// =============================================================================
// VARIABEL GLOBAL — pantau via Live Expression
// =============================================================================
extern uint8_t  g_dhcp_ip[4];
extern uint8_t  g_dhcp_gw[4];
extern uint8_t  g_dhcp_sub[4];
extern uint8_t  g_dhcp_dns[4];
extern uint32_t g_dhcp_lease;          // Lease time dalam detik
extern uint8_t  g_dhcp_state;          // Status DHCP saat ini
extern uint8_t  g_link_status;         // 0 = down, 1 = up

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

/**
 * @brief  Init DHCP — panggil sekali di awal setelah W5500_InitNetwork()
 */
void DHCP_Init(void);

/**
 * @brief  Proses DHCP — panggil terus di loop utama seperti Process_Web_OTA()
 *         Menangani: link detection, discover, offer, request, ack, renewal
 */
void DHCP_Process(void);

/**
 * @brief  Cek apakah DHCP sudah bound (dapat IP dan siap dipakai)
 * @retval 1 = sudah bound, 0 = belum
 */
uint8_t DHCP_IsBound(void);

#endif /* DHCP_H */
