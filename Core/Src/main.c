/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Backup / Default Recovery Program (STM32H523CET6)
  *                   - Konsep kerja identik dengan referensi sha256
  *                   - Bebas snprintf/newlib crash (zero heap, static buffers)
  *                   - W5500 via SPI1 + DHCP Socket 1 + Web OTA Socket 0
  *                   - Target: Tulis firmware ke OctoSPI Flash (XIP Slot: 0x90001000)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "octospi_ll.h"
#include "dhcp.h"
#include "html_updater.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --------------------------------------------------------------------------
 * PARTISI FLASH EKSTERNAL UNTUK APLIKASI UTAMA (Sesuai Bootloader)
 * -------------------------------------------------------------------------- */
#define FLASH_APP_BASE          0x00000000UL                           /* Sektor 0 (0 KB) */
#define FLASH_APP_META          (FLASH_APP_BASE)                       /* 0x00000000 */
#define FLASH_APP_BIN           (FLASH_APP_META + 4096UL)              /* 0x00001000 */
#define STAGING_FLAG_VALID      0xA5U
#define MAX_FIRMWARE_SIZE       (1UL * 1024UL * 1024UL)                /* 1 MB */

/* --------------------------------------------------------------------------
 * W5500 ETHERNET DEFINES (Persis sha256)
 * -------------------------------------------------------------------------- */
#define CHUNK_BUFFER_SIZE       512
#define W5500_COMMON_REG_OP     0x00
#define W5500_S0_REG_OP         0x08
#define W5500_S0_TX_OP          0x10
#define W5500_S0_RX_OP          0x18
#define W5500_S0_BUF_MASK       0x1FFF /* 8 KB buffer mask untuk Socket 0 */

#define Sn_MR                   0x0000
#define Sn_CR                   0x0001
#define Sn_IR                   0x0002
#define Sn_SR                   0x0003
#define Sn_PORT                 0x0004
#define Sn_TX_FSR               0x0020
#define Sn_TX_WR                0x0024
#define Sn_RX_RSR               0x0026
#define Sn_RX_RD                0x0028

#define SOCK_CLOSED             0x00
#define SOCK_INIT               0x13
#define SOCK_LISTEN             0x14
#define SOCK_ESTABLISHED        0x17
#define SOCK_CLOSE_WAIT         0x1C
#define SOCK_FIN_WAIT           0x18

#define CMD_OPEN                0x01
#define CMD_LISTEN              0x02
#define CMD_DISCON              0x08
#define CMD_CLOSE               0x10
#define CMD_SEND                0x20
#define CMD_RECV                0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_ms_tick = 0;
static uint8_t ram_buffer[CHUNK_BUFFER_SIZE + 64];
static uint8_t overlap[4 + CHUNK_BUFFER_SIZE];
static char s_hdr_buf[160];
static char s_ok_buf[160];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void Process_Web_OTA(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Helper format integer ke string tanpa snprintf / libc overhead (bebas HardFault) */
static void uint32_to_str(uint32_t val, char *out)
{
    char tmp[12];
    int i = 0;
    if (val == 0)
    {
        *out++ = '0';
        *out = '\0';
        return;
    }
    while (val > 0)
    {
        tmp[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (i > 0)
    {
        *out++ = tmp[--i];
    }
    *out = '\0';
}

/* ==============================================================================
 * 1. DRIVER SPI1 & W5500 BARE-METAL (Konsep persis sha256)
 * ============================================================================== */
static inline uint8_t SPI1_Transfer(uint8_t data)
{
    while (!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, data);
    while (!LL_SPI_IsActiveFlag_RXP(SPI1));
    return LL_SPI_ReceiveData8(SPI1);
}

void W5500_WriteReg(uint16_t addr, uint8_t block, uint8_t data)
{
    LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
    SPI1_Transfer((uint8_t)(addr >> 8));
    SPI1_Transfer((uint8_t)addr);
    SPI1_Transfer(block | 0x04);
    SPI1_Transfer(data);
    LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
}

uint8_t W5500_ReadReg(uint16_t addr, uint8_t block)
{
    LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
    SPI1_Transfer((uint8_t)(addr >> 8));
    SPI1_Transfer((uint8_t)addr);
    SPI1_Transfer(block | 0x00);
    uint8_t val = SPI1_Transfer(0xFF);
    LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
    return val;
}

void W5500_WriteBuf(uint16_t addr, uint8_t block, const uint8_t *buf, uint16_t len)
{
    LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
    SPI1_Transfer((uint8_t)(addr >> 8));
    SPI1_Transfer((uint8_t)addr);
    SPI1_Transfer(block | 0x04);
    for (uint16_t i = 0; i < len; i++)
    {
        SPI1_Transfer(buf[i]);
    }
    LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
}

void W5500_ReadBuf(uint16_t addr, uint8_t block, uint8_t *buf, uint16_t len)
{
    LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
    SPI1_Transfer((uint8_t)(addr >> 8));
    SPI1_Transfer((uint8_t)addr);
    SPI1_Transfer(block | 0x00);
    for (uint16_t i = 0; i < len; i++)
    {
        buf[i] = SPI1_Transfer(0xFF);
    }
    LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
}

/* W5500 Reset & Init: persis sha256 (busy loop volatile, bebas deadlock HAL) */
void W5500_InitNetwork(void)
{
    LL_GPIO_ResetOutputPin(ETH_RST_GPIO_Port, ETH_RST_Pin);
    for (volatile int i = 0; i < 250000; i++); /* Tahan LOW */
    LL_GPIO_SetOutputPin(ETH_RST_GPIO_Port, ETH_RST_Pin);
    for (volatile int i = 0; i < 2500000; i++); /* Tunggu PLL W5500 siap */

    uint8_t mac[]  = {0x00, 0x08, 0xDC, 0x52, 0x3C, 0x01};
    uint8_t zero[] = {0, 0, 0, 0};
    W5500_WriteBuf(0x0009, W5500_COMMON_REG_OP, mac,  6);
    W5500_WriteBuf(0x0001, W5500_COMMON_REG_OP, zero, 4); /* GW */
    W5500_WriteBuf(0x0005, W5500_COMMON_REG_OP, zero, 4); /* Subnet */
    W5500_WriteBuf(0x000F, W5500_COMMON_REG_OP, zero, 4); /* IP */

    /* Alokasi buffer W5500 (Total TX/RX maks 16 KB):
     * Socket 0 (Web OTA) = 8 KB (Mask 0x1FFF)
     * Socket 1 (DHCP)    = 2 KB (Mask 0x7FF)
     * Socket 2..7        = 0 KB (Unused, agar total 10 KB <= 16 KB) */
    W5500_WriteReg(0x001E, W5500_S0_REG_OP, 8);
    W5500_WriteReg(0x001F, W5500_S0_REG_OP, 8);
    W5500_WriteReg(0x001E, 0x28,            2);
    W5500_WriteReg(0x001F, 0x28,            2);

    for (uint8_t s = 2; s < 8; s++)
    {
        uint8_t blk = (s << 5) | 0x08; /* Socket s register block (0x48..0xE8) */
        W5500_WriteReg(0x001E, blk, 0);
        W5500_WriteReg(0x001F, blk, 0);
    }
}

static void W5500_WaitCR(uint8_t block, uint32_t timeout_ms)
{
    uint32_t t = g_ms_tick;
    while (W5500_ReadReg(Sn_CR, block))
    {
        if ((g_ms_tick - t) > timeout_ms) break;
    }
}

static void W5500_WaitSR(uint8_t block, uint8_t expected, uint32_t timeout_ms)
{
    uint32_t t = g_ms_tick;
    while (W5500_ReadReg(Sn_SR, block) != expected)
    {
        if ((g_ms_tick - t) > timeout_ms) break;
    }
}

void W5500_SocketOpenListen(uint16_t port)
{
    W5500_WriteReg(Sn_CR,       W5500_S0_REG_OP, CMD_CLOSE);
    W5500_WaitCR(W5500_S0_REG_OP, 10);
    W5500_WriteReg(Sn_MR,       W5500_S0_REG_OP, 0x01); /* TCP mode */
    W5500_WriteReg(Sn_PORT,     W5500_S0_REG_OP, (uint8_t)(port >> 8));
    W5500_WriteReg(Sn_PORT + 1, W5500_S0_REG_OP, (uint8_t)port);
    W5500_WriteReg(Sn_CR,       W5500_S0_REG_OP, CMD_OPEN);
    W5500_WaitCR(W5500_S0_REG_OP, 10);
    W5500_WaitSR(W5500_S0_REG_OP, SOCK_INIT, 50);
    W5500_WriteReg(Sn_CR,       W5500_S0_REG_OP, CMD_LISTEN);
    W5500_WaitCR(W5500_S0_REG_OP, 10);
    W5500_WaitSR(W5500_S0_REG_OP, SOCK_LISTEN, 50);
}

void W5500_SendTCP(const uint8_t *data, uint16_t len)
{
    uint16_t ptr = ((uint16_t)W5500_ReadReg(Sn_TX_WR,     W5500_S0_REG_OP) << 8) |
                              W5500_ReadReg(Sn_TX_WR + 1, W5500_S0_REG_OP);
    W5500_WriteBuf(ptr, W5500_S0_TX_OP, data, len);
    ptr += len;
    W5500_WriteReg(Sn_TX_WR,     W5500_S0_REG_OP, (uint8_t)(ptr >> 8));
    W5500_WriteReg(Sn_TX_WR + 1, W5500_S0_REG_OP, (uint8_t)ptr);
    W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_SEND);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
}

/* ==============================================================================
 * 2. HITUNG HARDWARE CRC32 DARI FLASH EKSTERNAL (Persis sha256)
 * ============================================================================== */
static uint32_t CRC32_ComputeFromFlash(uint32_t flash_offset, uint32_t size_bytes)
{
    uint8_t buf[64];
    uint32_t remaining = size_bytes;
    uint32_t offset = flash_offset;

    LL_CRC_ResetCRCCalculationUnit(CRC);

    while (remaining > 0U)
    {
        uint32_t chunk = (remaining > sizeof(buf)) ? sizeof(buf) : remaining;
        uint32_t chunk_aligned = (chunk + 3U) & ~3U;

        for (uint32_t z = 0; z < chunk_aligned; z++) buf[z] = 0xFFU;

        Flash_ReadBytes(offset, buf, chunk);

        for (uint32_t i = 0; i < chunk_aligned; i += 4U)
        {
            uint32_t word = (uint32_t)buf[i]
                          | ((uint32_t)buf[i + 1U] << 8U)
                          | ((uint32_t)buf[i + 2U] << 16U)
                          | ((uint32_t)buf[i + 3U] << 24U);
            LL_CRC_FeedData32(CRC, word);
        }

        offset += chunk;
        remaining -= chunk;
    }

    return LL_CRC_ReadData32(CRC);
}

/* ==============================================================================
 * 3. WEB SERVER & FIRMWARE OTA HANDLER (Konsep persis sha256)
 * ============================================================================== */
void Process_Web_OTA(void)
{
    uint8_t sr = W5500_ReadReg(Sn_SR, W5500_S0_REG_OP);
    if (sr == SOCK_CLOSED)
    {
        W5500_SocketOpenListen(80);
        return;
    }

    if (sr == SOCK_CLOSE_WAIT || sr == SOCK_FIN_WAIT)
    {
        W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
        return;
    }

    if (sr != SOCK_ESTABLISHED) return;

    uint16_t rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
                                 W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
    if (rx_len == 0) return;

    uint16_t rx_rd = ((uint16_t)W5500_ReadReg(Sn_RX_RD,     W5500_S0_REG_OP) << 8) |
                                W5500_ReadReg(Sn_RX_RD + 1, W5500_S0_REG_OP);
    uint16_t fetch = (rx_len > (CHUNK_BUFFER_SIZE - 1)) ? (CHUNK_BUFFER_SIZE - 1) : rx_len;
    uint16_t s_off = rx_rd & W5500_S0_BUF_MASK;
    if ((s_off + fetch) > (W5500_S0_BUF_MASK + 1))
    {
        uint16_t part1 = (W5500_S0_BUF_MASK + 1) - s_off;
        W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, part1);
        W5500_ReadBuf(0x0000, W5500_S0_RX_OP, ram_buffer + part1, fetch - part1);
    }
    else
    {
        W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, fetch);
    }
    ram_buffer[fetch] = '\0';

    /* -------------------------------------------------------------
     * GET / — Kirim Web UI Updater
     * ------------------------------------------------------------- */
    if (strstr((char *)ram_buffer, "GET /") != NULL && strstr((char *)ram_buffer, "POST") == NULL)
    {
        uint32_t page_len = strlen(HTML_UPDATER);
        char len_str[12];
        uint32_to_str(page_len, len_str);

        /* Format header HTTP aman tanpa snprintf/libc (zero crash risk) */
        strcpy(s_hdr_buf, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: ");
        strcat(s_hdr_buf, len_str);
        strcat(s_hdr_buf, "\r\nConnection: close\r\n\r\n");

        W5500_SendTCP((uint8_t *)s_hdr_buf, (uint16_t)strlen(s_hdr_buf));

        /* Stream HTML per chunk */
        uint32_t sent = 0;
        while (sent < page_len)
        {
            uint16_t chunk = (page_len - sent > 1024) ? 1024 : (uint16_t)(page_len - sent);
            W5500_SendTCP((const uint8_t *)HTML_UPDATER + sent, chunk);
            sent += chunk;
        }

        rx_rd += rx_len;
        W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
        W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
        W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
        W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_DISCON);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
        return;
    }

    /* -------------------------------------------------------------
     * POST /ota — Terima file firmware .bin (Persis sha256)
     * ------------------------------------------------------------- */
    if (strstr((char *)ram_buffer, "POST /ota") == NULL) return;

    char *pSize = strstr((char *)ram_buffer, "?sz=");
    char *pCrc  = strstr((char *)ram_buffer, "&crc=");
    if (!pSize || !pCrc) return;

    uint32_t expected_size = strtoul(pSize + 4, NULL, 10);
    uint32_t expected_crc  = strtoul(pCrc + 5, NULL, 0);

    if (expected_size == 0 || expected_size > MAX_FIRMWARE_SIZE)
    {
        const char err[] = "HTTP/1.1 400 Bad\r\nConnection: close\r\n\r\nFile terlalu besar!";
        W5500_SendTCP((uint8_t *)err, strlen(err));
        W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
        return;
    }

    uint8_t rx_already_updated = 0;
    uint8_t tail[4] = { 0 };
    char *pBody = NULL;

    while (pBody == NULL)
    {
        memcpy(overlap, tail, 4);
        memcpy(overlap + 4, ram_buffer, fetch);
        overlap[4 + fetch] = '\0';
        char *found = strstr((char *)overlap, "\r\n\r\n");
        if (found)
        {
            int pos_in_overlap = (found - (char *)overlap);
            int pos_in_buf = pos_in_overlap - 4;
            int body_start = pos_in_buf + 4;
            if (body_start < 0) body_start = 0;
            if (body_start > fetch) body_start = fetch;
            pBody = (char *)ram_buffer + body_start;
            rx_rd += fetch;
            W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
            W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
            W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
            while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
            rx_already_updated = 1;
        }
        else
        {
            memcpy(tail, ram_buffer + fetch - 4, 4);
            rx_rd += fetch;
            W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
            W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
            W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
            while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
            rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
                                 W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
            if (rx_len == 0) continue;
            fetch = (rx_len > (CHUNK_BUFFER_SIZE - 1)) ? (CHUNK_BUFFER_SIZE - 1) : rx_len;
            s_off = rx_rd & W5500_S0_BUF_MASK;
            W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, fetch);
            ram_buffer[fetch] = '\0';
        }
    }

    uint32_t target_meta = FLASH_APP_META;
    uint32_t target_bin  = FLASH_APP_BIN;

    /* Erase target sectors di flash eksternal */
    ExtFlash_EraseSector(target_meta);

    uint32_t sectors = (expected_size + 4095U) / 4096U;
    for (uint32_t s = 0; s < sectors; s++)
    {
        ExtFlash_EraseSector(target_bin + (s * 4096U));
    }

    uint16_t header_len = (uint8_t *)pBody - ram_buffer;
    uint16_t body_in_packet = fetch - header_len;
    uint32_t bytes_received = 0;

    if (body_in_packet > 0)
    {
        uint16_t written = 0;
        while (written < body_in_packet)
        {
            uint32_t cur_flash = target_bin + written;
            uint16_t page_offset = cur_flash & 0xFF;
            uint16_t space_left = 256 - page_offset;
            uint16_t page_len = (body_in_packet - written > space_left)
                                ? space_left
                                : (body_in_packet - written);
            ExtFlash_WritePage(cur_flash, (uint8_t *)pBody + written, page_len);
            written += page_len;
        }
        bytes_received += body_in_packet;
    }

    if (!rx_already_updated)
    {
        rx_rd += fetch;
        W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
        W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
        W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    }

    /* Stream sisa payload binary */
    while (bytes_received < expected_size)
    {
        rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
                             W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
        if (rx_len == 0) continue;
        rx_rd = ((uint16_t)W5500_ReadReg(Sn_RX_RD,     W5500_S0_REG_OP) << 8) |
                            W5500_ReadReg(Sn_RX_RD + 1, W5500_S0_REG_OP);
        uint16_t grab = (rx_len > CHUNK_BUFFER_SIZE) ? CHUNK_BUFFER_SIZE : rx_len;
        if (bytes_received + grab > expected_size) grab = expected_size - bytes_received;

        s_off = rx_rd & W5500_S0_BUF_MASK;
        if ((s_off + grab) > (W5500_S0_BUF_MASK + 1))
        {
            uint16_t p1 = (W5500_S0_BUF_MASK + 1) - s_off;
            W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, p1);
            W5500_ReadBuf(0x0000, W5500_S0_RX_OP, ram_buffer + p1, grab - p1);
        }
        else
        {
            W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, grab);
        }

        uint16_t written = 0;
        while (written < grab)
        {
            uint32_t cur_flash = target_bin + bytes_received + written;
            uint16_t page_offset = cur_flash & 0xFF;
            uint16_t space_left = 256 - page_offset;
            uint16_t page_len = (grab - written > space_left)
                                ? space_left
                                : (grab - written);
            ExtFlash_WritePage(cur_flash, &ram_buffer[written], page_len);
            written += page_len;
        }

        bytes_received += grab;
        rx_rd += grab;
        W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
        W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
        W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    }

    /* Hitung Hardware CRC32 dari flash eksternal */
    uint32_t calc_crc = CRC32_ComputeFromFlash(target_bin, expected_size);

    if (calc_crc == expected_crc)
    {
        /* Metadata: [0..3]=size, [4..7]=crc, [8]=0xA5 (Persis format sha256) */
        uint8_t meta[10] = { 0 };
        meta[0] = (uint8_t)(expected_size);
        meta[1] = (uint8_t)(expected_size >> 8);
        meta[2] = (uint8_t)(expected_size >> 16);
        meta[3] = (uint8_t)(expected_size >> 24);
        meta[4] = (uint8_t)(expected_crc);
        meta[5] = (uint8_t)(expected_crc >> 8);
        meta[6] = (uint8_t)(expected_crc >> 16);
        meta[7] = (uint8_t)(expected_crc >> 24);
        meta[8] = STAGING_FLAG_VALID; /* 0xA5 */

        ExtFlash_WritePage(target_meta, meta, 10);

        const char ok_body[] = "Update Terkirim! Firmware tersimpan di flash. Device Rebooting...";
        char ok_len_str[12];
        uint32_to_str((uint32_t)strlen(ok_body), ok_len_str);

        strcpy(s_ok_buf, "HTTP/1.1 200 OK\r\nContent-Length: ");
        strcat(s_ok_buf, ok_len_str);
        strcat(s_ok_buf, "\r\nConnection: close\r\n\r\n");
        strcat(s_ok_buf, ok_body);

        W5500_SendTCP((uint8_t *)s_ok_buf, (uint16_t)strlen(s_ok_buf));

        /* Delay persis sha256: loop volatile sebelum reboot */
        for (volatile int d = 0; d < 25000000; d++);
        NVIC_SystemReset();
    }
    else
    {
        const char fail_msg[] = "HTTP/1.1 500 Error\r\nConnection: close\r\n\r\nCRC Mismatch!";
        W5500_SendTCP((uint8_t *)fail_msg, strlen(fail_msg));
        W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
        while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    }
}

/* ==============================================================================
 * UART DHCP STATUS REPORTER
 * ============================================================================== */
static void UART_PrintDHCPStatus(void)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%d.%d.%d.%d (Status: %d)\r\n",
                       (int)g_dhcp_ip[0], (int)g_dhcp_ip[1],
                       (int)g_dhcp_ip[2], (int)g_dhcp_ip[3],
                       (int)g_dhcp_state);
    if (len > 0)
    {
        for (int i = 0; i < len; i++)
        {
            while (!LL_USART_IsActiveFlag_TXE_TXFNF(USART1));
            LL_USART_TransmitData8(USART1, (uint8_t)buf[i]);
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Relokasi Vector Table ke alamat aplikasi di internal flash */
  SCB->VTOR = 0x08002000UL;
  __enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Enable SysTick interrupt (persis baris 1115 di sha256) */
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Enable SPI1 Master Transfer */
  LL_SPI_SetTransferSize(SPI1, 0);
  LL_SPI_Enable(SPI1);
  LL_SPI_StartMasterTransfer(SPI1);

  /* Init OctoSPI ke Indirect mode untuk akses flash eksternal */
  OCTOSPI1_LL_IndirectInit();

  /* Init W5500 Hardware & Network Buffer (persis sha256) */
  W5500_InitNetwork();

  /* Init DHCP Client (persis sha256) */
  DHCP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DHCP_Process();

    /* Trigger 1: Cetak saat berhasil dapat IP DHCP (edge-triggered) */
    static uint8_t s_dhcp_bound_last = 0;
    uint8_t is_bound = DHCP_IsBound();
    if (is_bound && !s_dhcp_bound_last)
    {
        UART_PrintDHCPStatus();
    }
    s_dhcp_bound_last = is_bound;

    /* Trigger 2: Cetak saat ada karakter masuk di USART1 RX */
    if (LL_USART_IsActiveFlag_ORE(USART1))
    {
        LL_USART_ClearFlag_ORE(USART1);
    }
    if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART1))
    {
        (void)LL_USART_ReceiveData8(USART1);
        UART_PrintDHCPStatus();
    }

    if (DHCP_IsBound())
    {
        Process_Web_OTA();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL1_SetSource(LL_RCC_PLL1SOURCE_HSE);
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(2);
  LL_RCC_PLL1_SetN(40);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);
  LL_SetSystemCoreClock(250000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2_SetSource(LL_RCC_PLL2SOURCE_HSE);
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(2);
  LL_RCC_PLL2_SetN(40);
  LL_RCC_PLL2_SetP(4);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
