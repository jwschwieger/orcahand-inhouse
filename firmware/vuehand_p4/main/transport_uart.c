#include "transport.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Default transport: talks the Feetech bus directly over the P4's own UART
 * peripheral through a GPIO-wired half-duplex line (with an external
 * level-shifter/driver on the data line -- see main.c's VH_SERVO_* pins).
 * This is exactly what feetech.c's ft_init() did itself before the transport
 * split; behavior is unchanged, just moved to its own file so feetech.c can
 * also be built against transport_usb.cpp instead. */

static const char *TAG = "transport_uart";
static uart_port_t s_uart = UART_NUM_1;
static bool s_ready = false;

/* Adapters that switch the half-duplex line themselves (rts_gpio < 0, e.g.
 * the Waveshare Bus Servo Adapter (A) in UART mode) may hand our own packet
 * back on RX. An echoed read request is a checksum-valid packet carrying the
 * polled ID, so find_status() would accept it as the reply. Whether this
 * adapter echoes is unverified, so it is detected on the first write:
 * s_echo -1 = unknown, 0 = no echo, 1 = echo (stripped after every write).
 * Bytes read while checking that turn out not to be an echo are kept in
 * s_pushback and returned first by transport_read(). */
#define ECHO_WAIT_MS 3
static int s_echo = -1;
static bool s_self_switched = false;
static uint8_t s_pushback[256];
static size_t s_pushback_n = 0;

bool transport_init(const transport_uart_cfg_t *cfg)
{
    s_uart = (uart_port_t)cfg->uart_num;
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(s_uart, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_uart, &uart_cfg));
    s_self_switched = cfg->rts_gpio < 0;
    ESP_ERROR_CHECK(uart_set_pin(s_uart, cfg->tx_gpio, cfg->rx_gpio,
                                 s_self_switched ? UART_PIN_NO_CHANGE : cfg->rts_gpio, UART_PIN_NO_CHANGE));
    /* With a direction pin: same half-duplex assumption as the Dynamixel bus
     * -- verify this matches how the Touch's Feetech bus is actually wired
     * before trusting it. Without one the adapter drives the line, so plain
     * full-duplex UART on separate TX/RX pins. */
    ESP_ERROR_CHECK(uart_set_mode(s_uart, s_self_switched ? UART_MODE_UART : UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "uart%d %d baud tx=%d rx=%d rts=%d", cfg->uart_num, cfg->baud, cfg->tx_gpio, cfg->rx_gpio, cfg->rts_gpio);
    s_ready = true;
    return true;
}

bool transport_is_ready(void)
{
    return s_ready;
}

/* Reads back up to `len` bytes after a write. An exact copy of what was sent
 * is the echo and is dropped; anything else is a reply and is pushed back. */
static void strip_echo(const uint8_t *data, size_t len)
{
    s_pushback_n = 0;
    int got = uart_read_bytes(s_uart, s_pushback, len, pdMS_TO_TICKS(ECHO_WAIT_MS));
    if (got <= 0) {
        if (s_echo < 0) {
            s_echo = 0;
            ESP_LOGI(TAG, "adapter does not echo TX");
        }
        return;
    }
    if ((size_t)got == len && memcmp(s_pushback, data, len) == 0) {
        if (s_echo < 0) {
            s_echo = 1;
            ESP_LOGI(TAG, "adapter echoes TX -- stripping it after every write");
        }
        return;
    }
    s_pushback_n = (size_t)got;
    if (s_echo < 0) {
        s_echo = 0;
        ESP_LOGI(TAG, "adapter does not echo TX");
    } else {
        ESP_LOGW(TAG, "expected echo of %u bytes, got %d other bytes", (unsigned)len, got);
    }
}

bool transport_write(const uint8_t *data, size_t len)
{
    int w = uart_write_bytes(s_uart, (const char *)data, len);
    if (w != (int)len) {
        return false;
    }
    uart_wait_tx_done(s_uart, pdMS_TO_TICKS(20));
    if (s_self_switched && s_echo != 0 && len <= sizeof s_pushback) {
        strip_echo(data, len);
    }
    return true;
}

void transport_flush_rx(void)
{
    uart_flush_input(s_uart);
    s_pushback_n = 0;
}

size_t transport_read(uint8_t *out, size_t maxlen, int timeout_ms)
{
    if (maxlen == 0) {
        return 0;
    }
    if (s_pushback_n > 0) {
        size_t n = s_pushback_n < maxlen ? s_pushback_n : maxlen;
        memcpy(out, s_pushback, n);
        memmove(s_pushback, s_pushback + n, s_pushback_n - n);
        s_pushback_n -= n;
        return n;
    }
    /* Same contract as the USB backend: wait for the first byte, then take
     * what's already buffered without waiting. Asking uart_read_bytes() for
     * `maxlen` up front would block for the whole timeout on every read. */
    int first = uart_read_bytes(s_uart, out, 1, pdMS_TO_TICKS(timeout_ms));
    if (first <= 0) {
        return 0;
    }
    size_t n = 1;
    if (n < maxlen) {
        int more = uart_read_bytes(s_uart, out + n, maxlen - n, 0);
        if (more > 0) {
            n += (size_t)more;
        }
    }
    return n;
}
