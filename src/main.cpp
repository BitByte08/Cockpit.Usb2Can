// Serial <-> CAN (TWAI) bridge for ESP32-C3 using ESP-IDF

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "usb2can";

#define USB_BUF_SIZE 1024

// TWAI settings
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Simple ASCII protocol:
// Host -> ESP: "t<ID_HEX><DATA_HEX>\n" (eg: t10023A0B) sends a frame
// ESP -> Host: "t<ID_HEX><DATA_HEX>\n" when a CAN frame is received

static void init_usb_serial()
{
	usb_serial_jtag_driver_config_t usb_cfg;
	usb_cfg.rx_buffer_size = USB_BUF_SIZE * 2;
	usb_cfg.tx_buffer_size = USB_BUF_SIZE * 2;
	usb_serial_jtag_driver_install(&usb_cfg);
}

static void init_twai()
{
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);

	esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "TWAI driver installed (TX=4, RX=5)");
	} else if (err == ESP_ERR_INVALID_STATE) {
		ESP_LOGW(TAG, "TWAI driver already installed");
	} else {
		ESP_LOGE(TAG, "Failed to install TWAI driver: %s (0x%x)", esp_err_to_name(err), err);
	}

	err = twai_start();
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "TWAI started");
	} else {
		ESP_LOGE(TAG, "Failed to start TWAI: %s (0x%x)", esp_err_to_name(err), err);
	}

	twai_status_info_t status;
	twai_get_status_info(&status);
	ESP_LOGI(TAG, "TWAI state=%d, arb_lost=%d, rx_msg=%lu, tx_msg=%lu",
		status.state, status.arb_lost_count, (unsigned long)status.msgs_to_rx, (unsigned long)status.msgs_to_tx);
}

/* Read exactly n hex characters from s. Stops early on non-hex or NUL. */
static uint32_t hex_to_u32_n(const char *s, int n)
{
	uint32_t v = 0;
	for (int i = 0; i < n; i++) {
		char c = s[i];
		if (!c) break;
		uint8_t d = 0;
		if (c >= '0' && c <= '9') d = c - '0';
		else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
		else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
		else break;
		v = (v << 4) | d;
	}
	return v;
}

static void usb_rx_task(void *arg)
{
	uint8_t *data = (uint8_t *)malloc(USB_BUF_SIZE + 1);
	size_t idx = 0;

	while (1) {
		int len = usb_serial_jtag_read_bytes(data + idx, 1, pdMS_TO_TICKS(100));
		if (len > 0) {
			if (data[idx] == '\r') continue;
			if (data[idx] == '\n') {
				data[idx] = 0;
				// parse line
				if (idx >= 4 && (data[0] == 'T' || data[0] == 't')) {
					// SLCAN format: t{ID:3hex}{DLC:1hex}{DATA:2*DLChex}
					// Example: t105200FF → ID=0x105, DLC=2, Data=[0x00,0xFF]
					char *p = (char *)data + 1;
					/* Read exactly 3 hex chars for 11-bit standard CAN ID.
					 * hex_to_u32 was greedy and consumed "1052FF3F" entirely → wrong 29-bit ID. */
					uint32_t id = hex_to_u32_n(p, 3);
					p += 3;

					// Read DLC from position 4 (1 hex digit)
					uint8_t dlc = 0;
					if (*p >= '0' && *p <= '9') dlc = *p - '0';
					else if (*p >= 'A' && *p <= 'F') dlc = *p - 'A' + 10;
					else if (*p >= 'a' && *p <= 'f') dlc = *p - 'a' + 10;
					if (dlc > 8) dlc = 8;
					p++;  // skip DLC byte

					twai_message_t msg;
					memset(&msg, 0, sizeof(msg));
					msg.identifier = id;
					msg.extd = 0;  // always standard frame from SLCAN 't' command
					msg.data_length_code = dlc;

					// Read exactly dlc data bytes (exactly 2 hex chars each)
					for (int i = 0; i < (int)dlc && *p; i++) {
						msg.data[i] = (uint8_t)hex_to_u32_n(p, 2);
						p += 2;
					}

					esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(1000));
					/* Always report TWAI bus status so we can detect bus-off / no-ACK situations.
					 * OK = frame queued in TX buffer, NOT confirmed sent on bus.
					 * If tx_err > 0 or state != 1 (RUNNING): STM32 is not ACKing → physical bus broken. */
					{
						twai_status_info_t st;
						twai_get_status_info(&st);
						char resp[128];
						int rlen;
						if (res == ESP_OK) {
							rlen = snprintf(resp, sizeof(resp),
								"OK id=%03lX state=%d tx_err=%lu rx_err=%lu\n",
								(unsigned long)msg.identifier,
								(int)st.state,
								(unsigned long)st.tx_error_counter,
								(unsigned long)st.rx_error_counter);
						} else {
							rlen = snprintf(resp, sizeof(resp),
								"ERR id=%03lX state=%d tx_err=%lu rx_err=%lu arb=%lu res=%d\n",
								(unsigned long)msg.identifier,
								(int)st.state,
								(unsigned long)st.tx_error_counter,
								(unsigned long)st.rx_error_counter,
								(unsigned long)st.arb_lost_count, res);
						}
						usb_serial_jtag_write_bytes(resp, rlen, pdMS_TO_TICKS(100));
					}
				}
				idx = 0;
			} else {
				if (idx < USB_BUF_SIZE) idx += len;
				else idx = 0; // overflow, reset
			}
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	free(data);
	vTaskDelete(NULL);
}

static void twai_rx_task(void *arg)
{
	while (1) {
		twai_message_t msg;
		esp_err_t ret = twai_receive(&msg, pdMS_TO_TICKS(100));
		if (ret == ESP_OK) {
			char out[128];
			int pos = 0;
			pos += snprintf(out + pos, sizeof(out) - pos, "t%03lX%01X", (unsigned long)(msg.identifier & 0x1FFFFFFF), msg.data_length_code);
			for (int i = 0; i < msg.data_length_code; ++i) {
				pos += snprintf(out + pos, sizeof(out) - pos, "%02X", msg.data[i]);
				if (pos >= (int)sizeof(out) - 10) break;
			}
			pos += snprintf(out + pos, sizeof(out) - pos, "\n");
			usb_serial_jtag_write_bytes(out, pos, pdMS_TO_TICKS(100));
		} else if (ret == ESP_ERR_TIMEOUT) {
			vTaskDelay(pdMS_TO_TICKS(10));
		} else {
			twai_status_info_t status;
			twai_get_status_info(&status);
			if (status.state == TWAI_STATE_BUS_OFF) {
				ESP_LOGW(TAG, "TWAI bus-off, restarting...");
				twai_stop();
				vTaskDelay(pdMS_TO_TICKS(100));
				twai_start();
				ESP_LOGI(TAG, "TWAI restarted");
			} else if (status.state == TWAI_STATE_RECOVERING) {
				vTaskDelay(pdMS_TO_TICKS(100));
			} else {
				vTaskDelay(pdMS_TO_TICKS(10));
			}
		}
	}
	vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
	esp_log_level_set(TAG, ESP_LOG_INFO);
	ESP_LOGI(TAG, "Starting usb2can (USB<->TWAI) bridge");

	init_usb_serial();
	init_twai();

	xTaskCreate(usb_rx_task, "usb_rx", 4096, NULL, 5, NULL);
	xTaskCreate(twai_rx_task, "twai_rx", 4096, NULL, 5, NULL);

	// app_main returns, tasks keep running
}
