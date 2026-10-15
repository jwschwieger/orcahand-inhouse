#include "dynamixel.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "dxl";
static uart_port_t s_uart = UART_NUM_1;

#define DXL_HDR0 0xFF
#define DXL_HDR1 0xFF
#define DXL_HDR2 0xFD
#define DXL_RSV  0x00
#define DXL_INST_PING 0x01
#define DXL_INST_WRITE 0x03
#define DXL_INST_SYNC_WRITE 0x83
#define DXL_BROADCAST 0xFE
#define ADDR_GOAL_POSITION 116
#define ADDR_TORQUE_ENABLE 64
#define ADDR_OPERATING_MODE 11

static uint16_t crc16(const uint8_t *data, size_t n)
{
    uint16_t crc = 0;
    for (size_t j = 0; j < n; j++) {
        crc ^= (uint16_t)data[j] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x8005);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static size_t stuff(const uint8_t *in, size_t n, uint8_t *out, size_t out_max)
{
    /* Copy header (4 bytes) unstuffed, stuff the rest except we recompute length later. */
    if (n < 4 || out_max < n) {
        return 0;
    }
    memcpy(out, in, 4);
    size_t o = 4;
    for (size_t i = 4; i < n; i++) {
        if (o + 2 >= out_max) {
            return 0;
        }
        out[o++] = in[i];
        if (o >= 3 && out[o - 3] == 0xFF && out[o - 2] == 0xFF && out[o - 1] == 0xFD) {
            out[o++] = 0xFD;
        }
    }
    return o;
}

static bool tx_raw(const uint8_t *pkt, size_t n)
{
    dxl_flush_rx();
    int w = uart_write_bytes(s_uart, (const char *)pkt, n);
    if (w != (int)n) {
        return false;
    }
    uart_wait_tx_done(s_uart, pdMS_TO_TICKS(20));
    dxl_flush_rx();
    return true;
}

static bool send_packet(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t nparams)
{
    uint8_t raw[256];
    uint16_t length = (uint16_t)(nparams + 3);
    size_t n = 0;
    raw[n++] = DXL_HDR0;
    raw[n++] = DXL_HDR1;
    raw[n++] = DXL_HDR2;
    raw[n++] = DXL_RSV;
    raw[n++] = id;
    raw[n++] = (uint8_t)(length & 0xFF);
    raw[n++] = (uint8_t)(length >> 8);
    raw[n++] = inst;
    if (nparams) {
        memcpy(&raw[n], params, nparams);
        n += nparams;
    }
    uint16_t crc = crc16(raw, n);
    raw[n++] = (uint8_t)(crc & 0xFF);
    raw[n++] = (uint8_t)(crc >> 8);

    uint8_t stuffed[320];
    size_t sn = stuff(raw, n, stuffed, sizeof stuffed);
    if (sn == 0) {
        return false;
    }
    return tx_raw(stuffed, sn);
}

void dxl_flush_rx(void)
{
    uart_flush_input(s_uart);
}

void dxl_init(int uart_num, int tx_gpio, int rx_gpio, int rts_gpio, int baud)
{
    s_uart = (uart_port_t)uart_num;
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(s_uart, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(s_uart, tx_gpio, rx_gpio, rts_gpio, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(s_uart, UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "uart%d %d baud tx=%d rx=%d rts=%d", uart_num, baud, tx_gpio, rx_gpio, rts_gpio);
}

bool dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value)
{
    uint8_t p[3] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), value };
    return send_packet(id, DXL_INST_WRITE, p, 3);
}

bool dxl_ping(uint8_t id)
{
    return send_packet(id, DXL_INST_PING, NULL, 0);
}

bool dxl_sync_write_goal(const uint8_t *ids, const int32_t *ticks, int n)
{
    if (n <= 0 || n > 17) {
        return false;
    }
    uint8_t params[4 + 17 * 5];
    params[0] = ADDR_GOAL_POSITION & 0xFF;
    params[1] = ADDR_GOAL_POSITION >> 8;
    params[2] = 4;
    params[3] = 0;
    size_t o = 4;
    for (int i = 0; i < n; i++) {
        int32_t t = ticks[i];
        params[o++] = ids[i];
        params[o++] = (uint8_t)(t & 0xFF);
        params[o++] = (uint8_t)((t >> 8) & 0xFF);
        params[o++] = (uint8_t)((t >> 16) & 0xFF);
        params[o++] = (uint8_t)((t >> 24) & 0xFF);
    }
    return send_packet(DXL_BROADCAST, DXL_INST_SYNC_WRITE, params, (uint16_t)o);
}
