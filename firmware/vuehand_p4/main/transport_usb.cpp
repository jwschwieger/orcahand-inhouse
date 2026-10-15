/*
 * USB-host transport: talks to the Feetech bus through a USB-to-serial
 * adapter (URT-2/U2D2 or similar -- FTDI, CP210x and CH34x chips are all
 * registered below, whichever one it actually is) plugged into the P4's
 * USB-host-capable port, instead of wiring raw GPIO pins to the bus.
 *
 * API and structure ported from Espressif's own official example
 * (esp-idf release/v5.4, examples/peripherals/usb/host/cdc/cdc_acm_vcp).
 * Both directions are wired up: transport_write() sends, and handle_rx()
 * feeds a byte queue that transport_read() drains for feetech.c's
 * ft_read_bytes().
 *
 * VERIFIED on real hardware (ORCA Touch, 2026-09-02):
 *   - The adapter goes in one of the board's four USB-A ports, fed by the
 *     onboard CH334F hub off the native USB OTG pins. The USB-C port used for
 *     flashing/console is the DEVICE side and is not involved.
 *   - The board powers the adapter over those USB-A ports with no external
 *     supply on the second USB-C port (12V for the servos themselves comes
 *     from the hand's own PSU either way -- see CLAUDE.md).
 *   - This hand's adapter is NOT FTDI/CP210x/CH34x. Those three are still
 *     registered first, but it falls through to the standards-compliant
 *     CDC-ACM path below, which is what actually claims it.
 *   - All 17 servos reply to reads over this link (see servo_init()).
 */

#include <memory>
#include <cstdint>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

extern "C" {
#include "transport.h"
}

using namespace esp_usb;

namespace {
const char *TAG = "transport_usb";
std::unique_ptr<CdcAcmDevice> s_vcp;
bool s_drivers_registered = false;
QueueHandle_t s_rx_q = nullptr;   /* byte queue: handle_rx producer, transport_read() consumer */
volatile bool s_ready = false;   /* true only once a device is open AND line-coding is set */

void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: all devices freed");
        }
    }
}

bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    /* Runs on the CDC-ACM host driver's own task context, not the app task
     * that will eventually call transport_read() -- hence the queue instead
     * of a plain buffer. Dropping on overflow (0 timeout send) rather than
     * blocking here: this callback must return quickly. */
    (void)arg;
    if (s_rx_q) {
        for (size_t i = 0; i < data_len; i++) {
            xQueueSend(s_rx_q, &data[i], 0);
        }
    }
    return true;
}

void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error, err_no = %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "USB VCP adapter disconnected");
        s_ready = false;
        s_vcp.reset();
        break;
    default:
        break;
    }
}
} // namespace

/* Background retry loop: the initial hardware bring-up (usb_host_install,
 * cdc_acm_host_install) only has to happen once and is done synchronously in
 * transport_init() below. Actually *grabbing* a device is handled here, in a
 * task that keeps trying for as long as the board is powered on, instead of
 * giving up for good after one fixed-length window.
 *
 * This split matters because of something found the hard way on real
 * hardware: running VCP::open()/CdcAcmDevice::open() repeatedly *during* the
 * window right after usb_host_install() appears to contend with the host
 * stack's own hub/port enumeration and can suppress it entirely -- a device
 * plugged in during that initial busy-polling window was never seen, but the
 * exact same device plugged in later (after the polling had stopped) enumerated
 * cleanly every time. So: try once per loop iteration, and if that attempt
 * fails, back off and let the bus sit idle for a bit before trying again,
 * rather than hammering it. This also means the hand can be plugged in (or
 * unplugged and replugged) at any time after boot, not just in the first
 * few seconds -- matching the existing fire-and-forget design (nothing here
 * blocks app_main or crashes if no adapter ever shows up).
 */
void usb_open_task(void *arg)
{
    const int baud = (int)(intptr_t)arg;

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 5000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = NULL,
    };

    while (true) {
        if (s_vcp != nullptr) {
            /* Already have one (or handle_event's disconnect handler hasn't
             * cleared it yet) -- nothing to do this round. */
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "opening any VCP device (5s timeout)...");
        s_vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));
        if (s_vcp == nullptr) {
            /* Not a vendor-specific FTDI/CP210x/CH34x-style adapter (or none
             * was found). ORCA's own orca_core docs say their adapter shows
             * up as /dev/cu.usbmodemXXXX on macOS, not /dev/cu.usbserialXXXX
             * -- that naming usually means a standards-compliant CDC-ACM
             * device (native USB on the adapter's own MCU), which
             * VCP::open() does NOT cover (it only matches the
             * non-compliant vendor chips above). Fall back to the plain
             * compliant-CDC open path before giving up this round. */
            ESP_LOGW(TAG, "no VCP (FTDI/CP210x/CH34x) adapter found -- retrying as a "
                          "standards-compliant CDC-ACM device instead");
            auto plain = std::unique_ptr<CdcAcmDevice>(new CdcAcmDevice());
            esp_err_t open_err = plain->open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0, &dev_config);
            if (open_err == ESP_OK) {
                s_vcp = std::move(plain);
            }
        }

        if (s_vcp == nullptr) {
            ESP_LOGW(TAG, "no USB serial adapter found this round -- will keep retrying "
                          "(plug in the hand any time)");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = (uint32_t)baud,
            .bCharFormat = 0, /* 1 stop bit */
            .bParityType = 0, /* none */
            .bDataBits = 8,
        };
        esp_err_t err = s_vcp->line_coding_set(&line_coding);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "line_coding_set failed: %s -- dropping this device and retrying",
                     esp_err_to_name(err));
            s_vcp.reset();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        s_ready = true;
        ESP_LOGI(TAG, "USB VCP adapter ready at %d baud", baud);
        /* Keep the task alive (rather than deleting it) so a later
         * disconnect -- which clears s_vcp from handle_event() -- gets
         * picked back up automatically instead of needing a reboot. */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool transport_init(const transport_uart_cfg_t *cfg)
{
    ESP_LOGI(TAG, "installing USB host (ignoring uart/gpio config -- baud=%d only)", cfg->baud);

    /* Waveshare ESP32-P4-Module-DEV-KIT: the native USB D+/D- pins go through
     * an FSUSB42UMX 2:1 analog switch (U15) before reaching either a Type-C
     * connector or the onboard CH334F hub (whose upstream feeds the 4 USB-A
     * host ports). The switch's SEL pin is tied to GPIO23, *not* just to the
     * physical HOST/DEVICE jumper. Both GPIO23 levels were tried on real
     * hardware and made no observable difference either way -- the real bug
     * turned out to be the polling loop below contending with enumeration,
     * not this pin -- but it's left driven (rather than floating) since an
     * undriven mux select is worse, not better. */
    gpio_reset_pin(GPIO_NUM_23);
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_23, 0);

    if (!s_rx_q) {
        s_rx_q = xQueueCreate(256, sizeof(uint8_t));
    }

    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
        return false;
    }

    BaseType_t created = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    if (created != pdTRUE) {
        ESP_LOGE(TAG, "failed to start usb_lib_task");
        return false;
    }

    err = cdc_acm_host_install(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cdc_acm_host_install failed: %s", esp_err_to_name(err));
        return false;
    }

    if (!s_drivers_registered) {
        /* We don't need to know which chip the adapter actually uses --
         * whichever of these matches is the one that loads. */
        VCP::register_driver<FT23x>();
        VCP::register_driver<CP210x>();
        VCP::register_driver<CH34x>();
        s_drivers_registered = true;
    }

    /* Device acquisition happens in the background from here on (see
     * usb_open_task above) -- this returns as soon as the host stack itself
     * is up, regardless of whether anything is plugged in yet. */
    created = xTaskCreate(usb_open_task, "usb_open", 4096, (void *)(intptr_t)cfg->baud, 5, NULL);
    if (created != pdTRUE) {
        ESP_LOGE(TAG, "failed to start usb_open_task");
        return false;
    }

    return true;
}

bool transport_write(const uint8_t *data, size_t len)
{
    if (!s_vcp) {
        return false;
    }
    esp_err_t err = s_vcp->tx_blocking(const_cast<uint8_t *>(data), len);
    return err == ESP_OK;
}

bool transport_is_ready(void)
{
    return s_ready;
}

void transport_flush_rx(void)
{
    if (!s_rx_q) {
        return;
    }
    uint8_t discard;
    while (xQueueReceive(s_rx_q, &discard, 0) == pdTRUE) {
        /* drain whatever's queued */
    }
}

size_t transport_read(uint8_t *out, size_t maxlen, int timeout_ms)
{
    if (!s_rx_q || maxlen == 0) {
        return 0;
    }
    /* Block for the *first* byte only, then take whatever else is already
     * queued without waiting. The previous version kept blocking until it had
     * `maxlen` bytes, so every read burned its whole timeout waiting for
     * bytes that were never coming -- an 8-byte servo reply that landed in
     * under a millisecond still cost the full 20ms. Returning as soon as
     * something is here is what lets ft_read_bytes() parse as it goes.
     *
     * A servo reply arrives as one USB bulk transfer, so handle_rx() queues
     * all of its bytes together and the drain loop below picks up the whole
     * packet in this single call. */
    if (xQueueReceive(s_rx_q, &out[0], pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return 0;
    }
    size_t n = 1;
    while (n < maxlen && xQueueReceive(s_rx_q, &out[n], 0) == pdTRUE) {
        n++;
    }
    return n;
}
