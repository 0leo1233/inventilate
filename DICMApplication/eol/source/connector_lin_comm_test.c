/*
 * connector_lin_comm_test.h
 *
 *  Created on: 11 Aug 2022
 *      Author: Sundaramoorthy Jeyakodi LTTS
 */

#include "configuration.h"

#ifdef CONNECTOR_LIN_COMM_TEST

#include "driver/gpio.h"
#include "connector.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "connector_lin_comm_test.h"
#include <string.h>
#include <stdint.h>

/** Defines */
#define LIN_FRAME_START_LEN (3)				//!< Size of LIN frame start (0x00, 0x55, identifier)
#define LIN_FRAME_DATA_LEN (8)				//!< Size of LIN frame data
#define LIN_FRAME_LEN (LIN_FRAME_START_LEN + LIN_FRAME_DATA_LEN + 1)
#define UART_PIN_RTS2				UART_PIN_NO_CHANGE
#define UART_PIN_CTS2				UART_PIN_NO_CHANGE
#define BIT_LOCAL(data,shift) ((data&(1<<shift))>>shift)

static int initialize_lin_comm_test(void);
static void init_uart_lin(void);
static void lin_transmit(const uint8_t *data, size_t len);
static uint8_t calc_parity(uint8_t ident);
static uint8_t calculate_chksum(const uint8_t *data, size_t len);

CONNECTOR connector_lin_comm_test=
{
	.name="LIN Comm Test",
	.initialize=initialize_lin_comm_test,
};

static void init_uart_lin(void)
{
    uart_config_t uart_config =
	{
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    ZERO_CHECK(uart_param_config(CONNECTOR_LIN_UART_NUM, &uart_config));
	ZERO_CHECK(uart_set_pin(CONNECTOR_LIN_UART_NUM, CONNECTOR_LIN_UART_TX, CONNECTOR_LIN_UART_RX, UART_PIN_RTS2, UART_PIN_CTS2));
	ZERO_CHECK(uart_driver_install(CONNECTOR_LIN_UART_NUM, CONNECTOR_LIN_UART_BUF_SIZE, CONNECTOR_LIN_UART_BUF_SIZE, 8, 0, 0));
}

static int initialize_lin_comm_test(void)
{
	int i = 0;

	init_uart_lin();

	LIN_SLEEP(1);

	return i;
}

static uint8_t calc_parity(uint8_t ident)
{
	/* Create the Lin ID parity */
	uint8_t p0 = BIT_LOCAL(ident,0) ^ BIT_LOCAL(ident,1) ^ BIT_LOCAL(ident,2) ^ BIT_LOCAL(ident,4);
	uint8_t p1 = ~(BIT_LOCAL(ident,1) ^ BIT_LOCAL(ident,3) ^ BIT_LOCAL(ident,4) ^ BIT_LOCAL(ident,5));
	return (p0 | (p1<<1))<<6;
}

static uint8_t calculate_chksum(const uint8_t *data, size_t len)
{
	uint32_t checksum = 0;
	uint8_t result;
	uint8_t start = 0;

	if (((data[0] & 0x3f) == 0x3c) || ((data[0] & 0x3f) == 0x3d))
	{
		start = 1; // Classic checksum without the PID
	}

	for (size_t i = start; i < len; i++)
	{
		checksum = checksum + data[i];
		if (checksum >= 256)
		{
			checksum -= 255;
		}
	}

	result = (uint8_t)(((uint8_t)checksum) ^ 0xFF);

	return result;
}

void lin_tx_data(LIN_TX_FRAME* ptr_lin_tx)
{
    uint8_t data_len;
	uint8_t lin_data[20];
    uint8_t parity  = calc_parity(ptr_lin_tx->device_id);;
    uint8_t protected_id = ptr_lin_tx->device_id | parity;

    parity = calc_parity(ptr_lin_tx->device_id);
	protected_id = ptr_lin_tx->device_id | parity;

    //LOG(I, "protected_id = 0x%x", protected_id);
    
    lin_data[0] = protected_id;
	memcpy(&lin_data[1], ptr_lin_tx->data, ptr_lin_tx->size);
	lin_data[ptr_lin_tx->size + 1] = calculate_chksum(lin_data, ptr_lin_tx->size + 1); /* Enhanced chksum */
	data_len = ptr_lin_tx->size + 2; /* + 2 for device and checksum */

	lin_transmit(lin_data, data_len);
}

static void lin_transmit(const uint8_t *data, size_t len)
{
    size_t index;
	const uint8_t syncb = 0x55;

    uart_set_line_inverse(CONNECTOR_LIN_UART_NUM, UART_SIGNAL_TXD_INV);
	ets_delay_us(678); // break length, at least 13 bit length
	uart_set_line_inverse(CONNECTOR_LIN_UART_NUM, UART_SIGNAL_INV_DISABLE );
	ets_delay_us(53);  // Break delimiter delay, at least one bit length

    WRITE_PERI_REG(UART_FIFO_AHB_REG(CONNECTOR_LIN_UART_NUM), syncb);

    for ( index = 0; index < len; index++, data++ )
	{
		WRITE_PERI_REG(UART_FIFO_AHB_REG(CONNECTOR_LIN_UART_NUM), *data);
	}

    ESP_LOG_BUFFER_HEXDUMP("LINTX  ", (uint8_t *)data, len, ESP_LOG_INFO);
}


#endif
