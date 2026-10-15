#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Byte-stream transport used underneath the Feetech packet layer (feetech.c).
 * Exactly one backend is compiled in, picked at build time via the
 * VH_SERVO_TRANSPORT CMake cache variable in CMakeLists.txt:
 *   - "uart" (default): raw GPIO/UART half-duplex bus -- transport_uart.c.
 *     Build normally: `idf.py build`.
 *   - "usb": talk to the bus through a USB-to-serial adapter (e.g. a URT-2 /
 *     U2D2) plugged into the P4's USB-host-capable port, using ESP-IDF's USB
 *     Host + VCP (virtual COM port) drivers -- transport_usb.cpp. Build with
 *     `idf.py -D VH_SERVO_TRANSPORT=usb build`.
 * uart_cfg is only meaningful to the uart backend; the usb backend ignores
 * the GPIO fields (there are none -- it's USB) and only looks at baud, which
 * it negotiates over the USB CDC line coding instead of a UART peripheral. */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int uart_num;
    int tx_gpio;
    int rx_gpio;
    int rts_gpio;
    int baud;
} transport_uart_cfg_t;

/* Returns false if the transport could not be brought up (e.g. no USB VCP
 * adapter enumerated within the connection timeout). feetech.c's ft_init()
 * logs but does not hard-fail on this -- matches the existing fire-and-forget
 * design (nothing here reads back a response to confirm success anyway). */
bool transport_init(const transport_uart_cfg_t *uart_cfg);
bool transport_write(const uint8_t *data, size_t len);
void transport_flush_rx(void);

/* Waits up to timeout_ms for the *first* byte, then copies whatever else is
 * already buffered (up to maxlen) without waiting further. Returns the number
 * of bytes copied, 0 if nothing arrived in time. Deliberately returns as soon
 * as there is something rather than filling maxlen: feetech.c's
 * ft_read_bytes() calls this in short slices and parses after each one, so a
 * reply that lands immediately costs ~1ms instead of the whole budget. */
size_t transport_read(uint8_t *out, size_t maxlen, int timeout_ms);

/* True once this transport has a genuinely usable link -- on the USB backend
 * that means a VCP device is open AND line coding is set (not just that
 * usb_host_install() succeeded); on the UART backend, true as soon as
 * transport_init() returns. Added after a real bug: servo_init() used to run
 * immediately after ft_init() and assume the bus was already usable, but on
 * USB the adapter isn't open yet at that point -- its torque-off writes were
 * silently failing (transport_write() returns false while s_vcp is null). */
bool transport_is_ready(void);

#ifdef __cplusplus
}
#endif
