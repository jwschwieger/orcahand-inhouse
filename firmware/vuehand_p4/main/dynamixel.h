#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void dxl_init(int uart_num, int tx_gpio, int rx_gpio, int rts_gpio, int baud);
bool dxl_ping(uint8_t id);
bool dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value);
bool dxl_sync_write_goal(const uint8_t *ids, const int32_t *ticks, int n);
void dxl_flush_rx(void);
