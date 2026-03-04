// Serial <-> CAN (TWAI) bridge for ESP32-C3 using ESP-IDF

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "usb2can";

// UART settings (use the USB-serial bridge on devkit)
#define UART_PORT UART_NUM_0
#define UART_BAUD 115200
#define UART_BUF_SIZE 1024

// TWAI settings
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Simple ASCII protocol:
// Host -> ESP: "T <ID_HEX> <LEN> <BYTE_HEX> ...\n" (eg: T 1FF 8 DE AD BE EF 00 11 22 33) sends a frame
// ESP -> Host: "F <ID_HEX> <LEN> <BYTE_HEX> ...\n" when a CAN frame is received

static void init_uart()
{
	const uart_config_t uart_config = {
		.baud_rate = UART_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_param_config(UART_PORT, &uart_config);
	uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void init_twai()
{
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_NC, GPIO_NUM_NC, TWAI_MODE_NORMAL);
	// For ESP32-C3 devkit pins, use default NC (internal) and let driver map to hw

	esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "TWAI driver installed");
	} else if (err == ESP_ERR_INVALID_STATE) {
		ESP_LOGW(TAG, "TWAI driver already installed");
	} else {
		ESP_LOGE(TAG, "Failed to install TWAI driver: %d", err);
	}

	if (twai_start() == ESP_OK) {
		ESP_LOGI(TAG, "TWAI started");
	} else {
		ESP_LOGE(TAG, "Failed to start TWAI");
	}
}

static uint32_t hex_to_u32(const char *s)
{
	uint32_t v = 0;
	while (*s && *s != ' ' && *s != '\t' && *s != '\n' && *s != '\r') {
		char c = *s++;
		uint8_t d = 0;
		if (c >= '0' && c <= '9') d = c - '0';
		else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
		else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
		else break;
		v = (v << 4) | d;
	}
	return v;
}

static void uart_rx_task(void *arg)
{
	uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE + 1);
	size_t idx = 0;

	while (1) {
		int len = uart_read_bytes(UART_PORT, data + idx, 1, pdMS_TO_TICKS(100));
		if (len > 0) {
			if (data[idx] == '\r') continue;
			if (data[idx] == '\n') {
				data[idx] = 0;
				// parse line
				if (idx > 0 && (data[0] == 'T' || data[0] == 't')) {
					// Expected: T ID LEN BYTES...
					char *p = (char *)data + 1;
					// skip spaces
					while (*p == ' ' || *p == '\t') p++;
					uint32_t id = hex_to_u32(p);
					// move p past id
					while (*p && *p != ' ' && *p != '\t') p++;
					while (*p == ' ' || *p == '\t') p++;
					int dlc = atoi(p);
					// move p past dlc
					while (*p && *p != ' ' && *p != '\t') p++;
					while (*p == ' ' || *p == '\t') p++;

					twai_message_t msg;
					memset(&msg, 0, sizeof(msg));
					msg.identifier = id;
					msg.extd = (id > 0x7FF) ? 1 : 0;
					msg.data_length_code = dlc > 8 ? 8 : dlc;

					for (int i = 0; i < msg.data_length_code; ++i) {
						// parse next hex byte
						if (!*p) break;
						uint32_t b = hex_to_u32(p);
						msg.data[i] = (uint8_t)b;
						// advance p past this byte token
						while (*p && *p != ' ' && *p != '\t') p++;
						while (*p == ' ' || *p == '\t') p++;
					}

					esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(1000));
					if (res == ESP_OK) {
						const char *ok = "OK\n";
						uart_write_bytes(UART_PORT, ok, strlen(ok));
					} else {
						const char *err = "ERR\n";
						uart_write_bytes(UART_PORT, err, strlen(err));
					}
				}
				idx = 0;
			} else {
				if (idx < UART_BUF_SIZE) idx += len;
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
		esp_err_t ret = twai_receive(&msg, pdMS_TO_TICKS(1000));
		if (ret == ESP_OK) {
			// format: F ID LEN BYTES...\n
			char out[128];
			int pos = 0;
			pos += snprintf(out + pos, sizeof(out) - pos, "F %03X %d", msg.identifier & 0x1FFFFFFF, msg.data_length_code);
			for (int i = 0; i < msg.data_length_code; ++i) {
				pos += snprintf(out + pos, sizeof(out) - pos, " %02X", msg.data[i]);
				if (pos >= (int)sizeof(out) - 10) break;
			}
			pos += snprintf(out + pos, sizeof(out) - pos, "\n");
			uart_write_bytes(UART_PORT, out, pos);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
	esp_log_level_set(TAG, ESP_LOG_INFO);
	ESP_LOGI(TAG, "Starting usb2can (UART<->TWAI) bridge");

	init_uart();
	init_twai();

	xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL);
	xTaskCreate(twai_rx_task, "twai_rx", 4096, NULL, 5, NULL);

	// app_main returns, tasks keep running
}