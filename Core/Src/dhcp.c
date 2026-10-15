#include "dhcp.h"
#include "main.h"
#include <string.h>

//#define DHCP_DEBUG

// =============================================================================
// FORWARD DECLARATION
// =============================================================================
extern uint8_t W5500_ReadReg  (uint16_t addr, uint8_t block);
extern void    W5500_WriteReg (uint16_t addr, uint8_t block, uint8_t data);
extern void    W5500_WriteBuf (uint16_t addr, uint8_t block, const uint8_t *buf, uint16_t len);
extern void    W5500_ReadBuf  (uint16_t addr, uint8_t block, uint8_t *buf, uint16_t len);
extern volatile uint32_t g_ms_tick;

// =============================================================================
// REGISTER W5500 SOCKET 1
// =============================================================================
#define S1_REG_OP    0x28   // Socket 1 register block
#define S1_TX_OP     0x30   // Socket 1 TX buffer block
#define S1_RX_OP     0x38   // Socket 1 RX buffer block

#define Sn_MR        0x0000
#define Sn_CR        0x0001
#define Sn_IR        0x0002
#define Sn_SR        0x0003
#define Sn_PORT      0x0004
#define Sn_DIPR      0x000C
#define Sn_DPORT     0x0010
#define Sn_TX_FSR    0x0020
#define Sn_TX_WR     0x0024
#define Sn_RX_RSR    0x0026
#define Sn_RX_RD     0x0028

#define SOCK_UDP         0x02
#define SOCK_CLOSED      0x00
#define SOCK_UDP_OPEN    0x22
#define CMD_OPEN         0x01
#define CMD_CLOSE        0x10
#define CMD_SEND         0x20
#define CMD_RECV         0x40

#define S1_BUF_MASK      0x7FF

#define PHYCFGR          0x002E
#define W5500_COMMON_REG 0x00

// =============================================================================
// DHCP DEFINES
// =============================================================================
#define DHCP_PACKET_SIZE    300

#define DHCP_DISCOVER    1
#define DHCP_OFFER       2
#define DHCP_REQUEST     3
#define DHCP_ACK         5
#define DHCP_NAK         6

#define OPT_SUBNET       1
#define OPT_ROUTER       3
#define OPT_DNS          6
#define OPT_HOSTNAME     12
#define OPT_REQUESTED_IP 50
#define OPT_LEASE_TIME   51
#define OPT_MSG_TYPE     53
#define OPT_SERVER_ID    54
#define OPT_RENEWAL_TIME 58
#define OPT_END          255

#define W5500_CMD_TIMEOUT_MS   100
#define W5500_SR_TIMEOUT_MS    50

// =============================================================================
// VARIABEL GLOBAL
// =============================================================================
//uint8_t  g_dhcp_ip[4]  = {0};
//uint8_t  g_dhcp_gw[4]  = {0};
//uint8_t  g_dhcp_sub[4] = {0};
//uint8_t  g_dhcp_dns[4] = {0};
//uint32_t g_dhcp_lease  = 0;
//uint8_t  g_dhcp_state  = DHCP_STATE_IDLE;
//uint8_t  g_link_status = 0;
uint8_t  g_dhcp_ip[4];
uint8_t  g_dhcp_gw[4];
uint8_t  g_dhcp_sub[4];
uint8_t  g_dhcp_dns[4];
uint32_t g_dhcp_lease;
uint8_t  g_dhcp_state;
uint8_t  g_link_status;

// =============================================================================
// VARIABEL DEBUG
// Aktifkan dengan: #define DHCP_DEBUG di dhcp.h atau compiler flags
// =============================================================================
#ifdef DHCP_DEBUG
uint16_t g_dbg_rx_len        = 0;
uint8_t  g_dbg_rx_buf[8]     = {0};
uint16_t g_dbg_plen          = 0;
uint8_t  g_dbg_sr_after_send = 0;
uint8_t  g_dbg_ir_after_send = 0;
uint8_t  g_dbg_dhar[6]       = {0};
uint8_t  g_dbg_dipr[4]       = {0};
uint8_t  g_dbg_s1_sr         = 0;
#endif

// =============================================================================
// VARIABEL INTERNAL
// =============================================================================
static uint8_t  s_xid[4]        = {0x12, 0x34, 0x56, 0x78};
static uint8_t  s_mac[6]        = {0x00, 0x08, 0xDC, 0x11, 0x22, 0x33};
static uint8_t  s_server_ip[4]  = {0};
static uint8_t  s_offered_ip[4] = {0};
static uint32_t s_lease_ms      = 0;
static uint32_t s_renewal_ms    = 0;
static uint32_t s_tick_start    = 0;
static uint32_t s_bound_tick    = 0;
static uint8_t  s_retry         = 0;

static uint8_t  s_pkt_buf[DHCP_PACKET_SIZE];

// =============================================================================
// HELPER
// =============================================================================
static uint32_t get_tick_ms(void)
{
    return g_ms_tick;
}

static void wait_cr_clear(uint32_t timeout_ms)
{
    uint32_t t = g_ms_tick;
    while (W5500_ReadReg(Sn_CR, S1_REG_OP))
    {
        if ((g_ms_tick - t) > timeout_ms) break;
    }
}

static void wait_sr(uint8_t expected, uint32_t timeout_ms)
{
    uint32_t t = g_ms_tick;
    while (W5500_ReadReg(Sn_SR, S1_REG_OP) != expected)
    {
        if ((g_ms_tick - t) > timeout_ms) break;
    }
}

static uint8_t is_link_up(void)
{
    uint8_t phy = W5500_ReadReg(PHYCFGR, W5500_COMMON_REG);
    return (phy & 0x01) ? 1 : 0;
}

// =============================================================================
// SOCKET 1 UDP
// =============================================================================
static void S1_OpenUDP(void)
{
    W5500_WriteReg(Sn_CR, S1_REG_OP, CMD_CLOSE);
    wait_cr_clear(W5500_CMD_TIMEOUT_MS);
    for (int i = 0; i < 6; i++)
        W5500_WriteReg(0x0006 + i, S1_REG_OP, 0xFF);  // DHAR broadcast
    uint8_t bcast[4] = {255, 255, 255, 255};
    W5500_WriteBuf(Sn_DIPR,   S1_REG_OP, bcast, 4);
    W5500_WriteReg(Sn_DPORT,     S1_REG_OP, (uint8_t)(DHCP_SERVER_PORT >> 8));
    W5500_WriteReg(Sn_DPORT + 1, S1_REG_OP, (uint8_t)(DHCP_SERVER_PORT));
    W5500_WriteReg(Sn_MR,        S1_REG_OP, SOCK_UDP);
    W5500_WriteReg(Sn_PORT,      S1_REG_OP, (uint8_t)(DHCP_CLIENT_PORT >> 8));
    W5500_WriteReg(Sn_PORT + 1,  S1_REG_OP, (uint8_t)(DHCP_CLIENT_PORT));
    W5500_WriteReg(Sn_CR, S1_REG_OP, CMD_OPEN);
    wait_cr_clear(W5500_CMD_TIMEOUT_MS);
    wait_sr(SOCK_UDP_OPEN, W5500_SR_TIMEOUT_MS);

#ifdef DHCP_DEBUG
    g_dbg_s1_sr = W5500_ReadReg(Sn_SR, S1_REG_OP);
#endif
}

static void S1_Close(void)
{
    W5500_WriteReg(Sn_CR, S1_REG_OP, CMD_CLOSE);
    wait_cr_clear(W5500_CMD_TIMEOUT_MS);
}

// =============================================================================
// KIRIM UDP BROADCAST
// =============================================================================
static void S1_SendUDP(const uint8_t *data, uint16_t len)
{
    for (int i = 0; i < 6; i++)
        W5500_WriteReg(0x0006 + i, S1_REG_OP, 0xFF);  // DHAR broadcast
    uint8_t bcast[4] = {255, 255, 255, 255};
    W5500_WriteBuf(Sn_DIPR,      S1_REG_OP, bcast, 4);
    W5500_WriteReg(Sn_DPORT,     S1_REG_OP, (uint8_t)(DHCP_SERVER_PORT >> 8));
    W5500_WriteReg(Sn_DPORT + 1, S1_REG_OP, (uint8_t)(DHCP_SERVER_PORT));
    uint16_t ptr = ((uint16_t)W5500_ReadReg(Sn_TX_WR,     S1_REG_OP) << 8) |
                               W5500_ReadReg(Sn_TX_WR + 1, S1_REG_OP);
    W5500_WriteBuf(ptr, S1_TX_OP, data, len);
    ptr += len;
    W5500_WriteReg(Sn_TX_WR,     S1_REG_OP, (uint8_t)(ptr >> 8));
    W5500_WriteReg(Sn_TX_WR + 1, S1_REG_OP, (uint8_t)(ptr));
    W5500_WriteReg(Sn_IR, S1_REG_OP, 0xFF);
    W5500_WriteReg(Sn_CR, S1_REG_OP, CMD_SEND);
    wait_cr_clear(W5500_CMD_TIMEOUT_MS);
    // Tunggu SENDOK (0x01) atau TIMEOUT (0x10)
    uint32_t t = g_ms_tick;
    uint8_t ir;
    do {
        ir = W5500_ReadReg(Sn_IR, S1_REG_OP);
    } while (!(ir & 0x11) && (g_ms_tick - t) < 500);

    W5500_WriteReg(Sn_IR, S1_REG_OP, ir);  // clear IR flag

#ifdef DHCP_DEBUG
    g_dbg_ir_after_send = ir;
    g_dbg_sr_after_send = W5500_ReadReg(Sn_SR, S1_REG_OP);
#endif
}

// =============================================================================
// BACA UDP DARI RX BUFFER
// W5500 UDP RX header: [src IP 4B][src port 2B][data len 2B][data]
// =============================================================================
static uint16_t S1_RecvUDP(uint8_t *buf, uint16_t max_len)
{
    uint16_t rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     S1_REG_OP) << 8) |
                                 W5500_ReadReg(Sn_RX_RSR + 1, S1_REG_OP);

#ifdef DHCP_DEBUG
    g_dbg_rx_len = rx_len;
#endif

    if (rx_len == 0) return 0;
    uint16_t rx_rd = ((uint16_t)W5500_ReadReg(Sn_RX_RD,     S1_REG_OP) << 8) |
                                W5500_ReadReg(Sn_RX_RD + 1, S1_REG_OP);
    uint8_t udp_hdr[8];
    W5500_ReadBuf(rx_rd, S1_RX_OP, udp_hdr, 8);
    rx_rd += 8;
    uint16_t data_len = ((uint16_t)udp_hdr[6] << 8) | udp_hdr[7];
    if (data_len > max_len) data_len = max_len;
    W5500_ReadBuf(rx_rd, S1_RX_OP, buf, data_len);
    rx_rd += data_len;
    W5500_WriteReg(Sn_RX_RD,     S1_REG_OP, (uint8_t)(rx_rd >> 8));
    W5500_WriteReg(Sn_RX_RD + 1, S1_REG_OP, (uint8_t)(rx_rd));
    W5500_WriteReg(Sn_CR,        S1_REG_OP, CMD_RECV);
    wait_cr_clear(W5500_CMD_TIMEOUT_MS);
    return data_len;
}

// =============================================================================
// BUILD DHCP PACKET
// =============================================================================
static uint16_t build_dhcp_packet(uint8_t msg_type)
{
    memset(s_pkt_buf, 0, DHCP_PACKET_SIZE);
    s_pkt_buf[0]  = 0x01;  // op: BOOTREQUEST
    s_pkt_buf[1]  = 0x01;  // htype: Ethernet
    s_pkt_buf[2]  = 0x06;  // hlen: 6
    s_pkt_buf[3]  = 0x00;  // hops
    s_pkt_buf[4]  = s_xid[0];
    s_pkt_buf[5]  = s_xid[1];
    s_pkt_buf[6]  = s_xid[2];
    s_pkt_buf[7]  = s_xid[3];
    s_pkt_buf[10] = 0x80;  // flags: broadcast
    memcpy(&s_pkt_buf[28], s_mac, 6);
    s_pkt_buf[236] = 0x63;
    s_pkt_buf[237] = 0x82;
    s_pkt_buf[238] = 0x53;
    s_pkt_buf[239] = 0x63;
    uint16_t i = 240;
    s_pkt_buf[i++] = OPT_MSG_TYPE;
    s_pkt_buf[i++] = 1;
    s_pkt_buf[i++] = msg_type;
    if (msg_type == DHCP_REQUEST)
    {
        s_pkt_buf[i++] = OPT_REQUESTED_IP;
        s_pkt_buf[i++] = 4;
        memcpy(&s_pkt_buf[i], s_offered_ip, 4);
        i += 4;
        s_pkt_buf[i++] = OPT_SERVER_ID;
        s_pkt_buf[i++] = 4;
        memcpy(&s_pkt_buf[i], s_server_ip, 4);
        i += 4;
    }

    const char *hostname = "STM32-W5500";
    uint8_t hlen = (uint8_t)strlen(hostname);
    s_pkt_buf[i++] = OPT_HOSTNAME;
    s_pkt_buf[i++] = hlen;
    memcpy(&s_pkt_buf[i], hostname, hlen);
    i += hlen;
    s_pkt_buf[i++] = OPT_END;
    while (i < 300) s_pkt_buf[i++] = 0;

#ifdef DHCP_DEBUG
    g_dbg_plen = i;
#endif

    return i;
}

// =============================================================================
// PARSE DHCP RESPONSE
// =============================================================================
static uint8_t parse_dhcp_response(uint8_t *buf, uint16_t len, uint8_t expected_type)
{
    if (len < 240) return 0;
    if (buf[4] != s_xid[0] || buf[5] != s_xid[1] ||
        buf[6] != s_xid[2] || buf[7] != s_xid[3]) return 0;
    if (buf[236] != 0x63 || buf[237] != 0x82 ||
        buf[238] != 0x53 || buf[239] != 0x63) return 0;
    uint8_t  offered[4]   = {0};
    uint8_t  msg_type     = 0;
    uint8_t  subnet[4]    = {0};
    uint8_t  router[4]    = {0};
    uint8_t  dns[4]       = {0};
    uint8_t  server_id[4] = {0};
    uint32_t lease        = 0;
    uint32_t renewal      = 0;
    memcpy(offered, &buf[16], 4);
    uint16_t i = 240;
    while (i < len)
    {
        uint8_t opt = buf[i++];
        if (opt == OPT_END) break;
        if (opt == 0) continue;
        uint8_t opt_len = buf[i++];
        switch (opt)
        {
            case OPT_MSG_TYPE:    msg_type = buf[i]; break;
            case OPT_SUBNET:      memcpy(subnet,    &buf[i], 4); break;
            case OPT_ROUTER:      memcpy(router,    &buf[i], 4); break;
            case OPT_DNS:         memcpy(dns,        &buf[i], 4); break;
            case OPT_SERVER_ID:   memcpy(server_id, &buf[i], 4); break;
            case OPT_LEASE_TIME:
                lease = ((uint32_t)buf[i]   << 24) | ((uint32_t)buf[i+1] << 16) |
                        ((uint32_t)buf[i+2] <<  8) |  (uint32_t)buf[i+3];
                break;
            case OPT_RENEWAL_TIME:
                renewal = ((uint32_t)buf[i]   << 24) | ((uint32_t)buf[i+1] << 16) |
                          ((uint32_t)buf[i+2] <<  8) |  (uint32_t)buf[i+3];
                break;
            default: break;
        }
        i += opt_len;
    }

    if (msg_type != expected_type) return 0;
    memcpy(s_offered_ip, offered,   4);
    memcpy(s_server_ip,  server_id, 4);
    memcpy(g_dhcp_sub,   subnet,    4);
    memcpy(g_dhcp_gw,    router,    4);
    memcpy(g_dhcp_dns,   dns,       4);
    if (lease   == 0) lease   = 86400;
    if (renewal == 0) renewal = lease / 2;
    g_dhcp_lease = lease;
    s_lease_ms   = lease   * 1000UL;
    s_renewal_ms = renewal * 1000UL;
    return 1;
}

// =============================================================================
// APPLY IP KE W5500
// =============================================================================
static void apply_ip_to_w5500(void)
{
    W5500_WriteBuf(0x000F, W5500_COMMON_REG, g_dhcp_ip,  4);
    W5500_WriteBuf(0x0005, W5500_COMMON_REG, g_dhcp_sub, 4);
    W5500_WriteBuf(0x0001, W5500_COMMON_REG, g_dhcp_gw,  4);
}

// =============================================================================
// RESET STATE
// =============================================================================
static void dhcp_reset(void)
{
    memset(g_dhcp_ip,  0, 4);
    memset(g_dhcp_gw,  0, 4);
    memset(g_dhcp_sub, 0, 4);
    memset(g_dhcp_dns, 0, 4);
    g_dhcp_lease = 0;
    s_retry      = 0;
    g_dhcp_state = DHCP_STATE_IDLE;
}

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================
void DHCP_Init(void)
{
	memset(g_dhcp_ip,  0, 4);
	    memset(g_dhcp_gw,  0, 4);
	    memset(g_dhcp_sub, 0, 4);
	    memset(g_dhcp_dns, 0, 4);
	    g_dhcp_lease = 0;
	    g_dhcp_state = DHCP_STATE_IDLE;
	    g_link_status = 0;
    dhcp_reset();
    g_link_status = is_link_up();
}

uint8_t DHCP_IsBound(void)
{
    return (g_dhcp_state == DHCP_STATE_BOUND ||
            g_dhcp_state == DHCP_STATE_RENEWING) ? 1 : 0;
}

void DHCP_Process(void)
{
    uint32_t now = get_tick_ms();
    uint8_t link = is_link_up();
    if (link != g_link_status)
    {
        g_link_status = link;
        if (link == 0)
        {
            g_dhcp_state = DHCP_STATE_LINK_DOWN;
            S1_Close();
            return;
        }
        else
        {
            dhcp_reset();
        }
    }

    if (g_link_status == 0) return;
    switch (g_dhcp_state)
    {
        case DHCP_STATE_IDLE:
            S1_OpenUDP();
            {
                uint16_t plen = build_dhcp_packet(DHCP_DISCOVER);
                S1_SendUDP(s_pkt_buf, plen);
            }
            g_dhcp_state = DHCP_STATE_DISCOVER;
            s_tick_start = now;
            break;
        case DHCP_STATE_DISCOVER: {
            uint16_t rlen = S1_RecvUDP(s_pkt_buf, DHCP_PACKET_SIZE);
            if (rlen > 0)
            {
                if (parse_dhcp_response(s_pkt_buf, rlen, DHCP_OFFER))
                {
                    uint16_t plen = build_dhcp_packet(DHCP_REQUEST);
                    S1_SendUDP(s_pkt_buf, plen);
                    g_dhcp_state = DHCP_STATE_REQUEST;
                    s_tick_start = now;
                    s_retry      = 0;
                }
            }
            if ((now - s_tick_start) > DHCP_TIMEOUT_MS)
            {
                if (++s_retry >= DHCP_RETRY_MAX)
                {
                    dhcp_reset();
                }
                else
                {
                    uint16_t plen = build_dhcp_packet(DHCP_DISCOVER);
                    S1_SendUDP(s_pkt_buf, plen);
                    s_tick_start = now;
                }
            }
            break;
        }

        case DHCP_STATE_REQUEST: {
            uint16_t rlen = S1_RecvUDP(s_pkt_buf, DHCP_PACKET_SIZE);
            if (rlen > 0)
            {
                if (parse_dhcp_response(s_pkt_buf, rlen, DHCP_ACK))
                {
                    memcpy(g_dhcp_ip, s_offered_ip, 4);
                    apply_ip_to_w5500();
                    S1_Close();
                    g_dhcp_state = DHCP_STATE_BOUND;
                    s_bound_tick = now;
                    s_retry      = 0;
                }
            }
            if ((now - s_tick_start) > DHCP_TIMEOUT_MS)
            {
                if (++s_retry >= DHCP_RETRY_MAX)
                {
                    dhcp_reset();
                }
                else
                {
                    uint16_t plen = build_dhcp_packet(DHCP_REQUEST);
                    S1_SendUDP(s_pkt_buf, plen);
                    s_tick_start = now;
                }
            }
            break;
        }

        case DHCP_STATE_BOUND:
            if ((now - s_bound_tick) >= s_renewal_ms)
            {
                S1_OpenUDP();
                uint16_t plen = build_dhcp_packet(DHCP_REQUEST);
                S1_SendUDP(s_pkt_buf, plen);
                g_dhcp_state = DHCP_STATE_RENEWING;
                s_tick_start = now;
                s_retry      = 0;
            }
            break;
        case DHCP_STATE_RENEWING: {
            uint16_t rlen = S1_RecvUDP(s_pkt_buf, DHCP_PACKET_SIZE);
            if (rlen > 0)
            {
                if (parse_dhcp_response(s_pkt_buf, rlen, DHCP_ACK))
                {
                    memcpy(g_dhcp_ip, s_offered_ip, 4);
                    apply_ip_to_w5500();
                    S1_Close();
                    g_dhcp_state = DHCP_STATE_BOUND;
                    s_bound_tick = now;
                    s_retry      = 0;
                }
            }
            if ((now - s_tick_start) > DHCP_TIMEOUT_MS)
            {
                if (++s_retry >= DHCP_RETRY_MAX)
                {
                    S1_Close();
                    dhcp_reset();
                }
                else
                {
                    uint16_t plen = build_dhcp_packet(DHCP_REQUEST);
                    S1_SendUDP(s_pkt_buf, plen);
                    s_tick_start = now;
                }
            }
            break;
        }

        case DHCP_STATE_LINK_DOWN:
            break;
        default:
            break;
    }
}
