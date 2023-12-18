/*
 * connector_rvc_eol.c
 *
 *  Created on: 4 Sep 2019
 *      Author: Stefan.Henningsohn
 */

#include "configuration.h"

#ifdef CONNECTOR_RVC_EOL
#include "connector_rvc_eol.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "connector.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ddm2.h"
#include "esp_log.h"

/** Defines */
#define PIN_CAN_RX				CONNECTOR_RVC_CAN_RX
#define PIN_CAN_TX				CONNECTOR_RVC_CAN_TX

// PG: fixme. Why DDMP2_FRAME_SIZE_MAX not defined?
#define DDMP2_FRAME_SIZE_MAX    5

#define INCOMING_QUEUE_DEPTH    10

static int initialize_can(void);

CONNECTOR connector_rvc_eol =
{
	.name = "RV/C EOL connector",
	.initialize = initialize_can,
};

static int initialize_can_driver(void);
static void initialize_can_gpio(void);
#ifndef CONNECTOR_EOL_SERVICE
static void can_process_task(void* Parameter);
#endif
static void can_rx_task(void* Parameter);

//! \~ Initialize CAN
static int initialize_can_driver(void)
{
	int ret = 0;
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	//Install CAN driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
	{
		LOG(I, "Driver installed");

		//Start CAN driver
		if (twai_start() == ESP_OK)
		{
			LOG(I, "Driver started\n");
		}
		else
		{
			LOG(I, "Failed to start driver\n");
			ret = 1;
		}
	}
	else
	{
		ret = 1;
		LOG(I, "Failed to install driver\n");
	}

	return ret;
}

static void initialize_can_gpio(void)
{
	/* Pin is connected to CAN sleep, i.e. 1 for sleep. 0 Disable sleep. */
    CAN1_EN(0);
}

static int initialize_can(void)
{
	int ret;

  	if ((ret = initialize_can_driver()) == 0)
	{
		initialize_can_gpio();

		TRUE_CHECK(xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 3, NULL));
	}

	return !ret;
}

static void can_rx_task(void* Parameter)
{
	twai_message_t msg;

	while (1)
	{
		if (twai_receive(&msg, pdMS_TO_TICKS(10000)) == ESP_OK)
		{
			LOG(I, "CAN Message received");

			if (msg.flags & TWAI_MSG_FLAG_EXTD)
			{
				LOG(I, "Message is in Extended Format\n");
			}
			else
			{
				LOG(I, "Message is in Standard Format\n");
			}

			LOG(I, "ID is 0x%x\n", msg.identifier);

			if (!(msg.flags & TWAI_MSG_FLAG_RTR))
			{
				ESP_LOG_BUFFER_HEXDUMP("CAN->Data  ", msg.data, msg.data_length_code, ESP_LOG_INFO);
			}

			// Retransmit msg (ID + 1)
			msg.identifier += 1;
			for (int i=0;i<msg.data_length_code;i++)
			{
				msg.data[i] = msg.data[i] + 1;
			}
			twai_transmit(&msg, 0);
		}
		else
		{
			LOG(I, "Failed to receive message\n");
		}
	}
}

#endif
