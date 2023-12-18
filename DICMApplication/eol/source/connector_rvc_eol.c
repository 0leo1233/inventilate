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
#include "driver/can.h"
#include "connector.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ddm2.h"
#include "esp_log.h"

/** Defines */
// ?? #define CAN_ENABLE				GPIO_NUM_23
#define PIN_CAN_RX				CONNECTOR_RVC_CAN_RX
#define PIN_CAN_TX				CONNECTOR_RVC_CAN_TX

// PG: fixme. Why DDMP2_FRAME_SIZE_MAX not defined?
#define DDMP2_FRAME_SIZE_MAX    5

#define INCOMING_QUEUE_DEPTH    10

static int initialize_can(void);

CONNECTOR connector_rvc_eol=
{
	.name="RV/C EOL connector",
	.initialize=initialize_can,
};

/* Queue for CAN DDMP */
//static QueueHandle_t xQueueCan;
static StaticQueue_t xStaticQueue;							//!< \~ Static queue metadata storage for incoming queue
static uint8_t ucQueueStorageArea[INCOMING_QUEUE_DEPTH * DDMP2_FRAME_SIZE_MAX];	//!< \~ Static queue data storage for CAN DDMP

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
	can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, CAN_MODE_NORMAL);
	can_timing_config_t t_config = CAN_TIMING_CONFIG_250KBITS();
	can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

	//Install CAN driver
	if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
	{
		LOG(I, "Driver installed");

		//Start CAN driver
		if (can_start() == ESP_OK)
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

	TRUE_CHECK(connector_rvc.to_connector=xQueueCreateStatic(INCOMING_QUEUE_DEPTH, DDMP2_FRAME_SIZE_MAX, ucQueueStorageArea, &xStaticQueue));

  	if ((ret = initialize_can_driver()) == 0)
	{
		initialize_can_gpio();

#ifndef CONNECTOR_EOL_SERVICE
		TRUE_CHECK(xTaskCreate(can_process_task, "can_ddmp", 4096, NULL, 3, NULL));
#endif

		TRUE_CHECK(xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 3, NULL));
	}

	return !ret;
}

#ifndef CONNECTOR_EOL_SERVICE
static void can_process_task(void* Parameter)
{
	DDMP2_FRAME ddmp_msg;

	while (1)
	{
		TRUE_CHECK(xQueueReceive(connector_rvc.to_connector, (void *)&ddmp_msg, portMAX_DELAY));

		ESP_LOG_BUFFER_HEXDUMP("DDMP->CAN  ", (uint8_t *)&ddmp_msg, ddmp_msg.frame_size + DDMP2_METADATA_SIZE, ESP_LOG_INFO);

		LOG(I, "CAN action 0x%x\n", ddmp_msg.frame.control);

		if (ddmp_msg.frame.control == DDMP2_CONTROL_SET)
		{
			LOG(I, "CAN PUT\n");
		}
	}
}
#endif

static void can_rx_task(void* Parameter)
{
	can_message_t msg;

	while (1)
	{
		if (can_receive(&msg, pdMS_TO_TICKS(10000)) == ESP_OK)
		{
			LOG(I, "CAN Message received");

			if (msg.flags & CAN_MSG_FLAG_EXTD)
			{
				LOG(I, "Message is in Extended Format\n");
			}
			else
			{
				LOG(I, "Message is in Standard Format\n");
			}

			LOG(I, "ID is 0x%x\n", msg.identifier);

			if (!(msg.flags & CAN_MSG_FLAG_RTR))
			{
				ESP_LOG_BUFFER_HEXDUMP("CAN->Data  ", msg.data, msg.data_length_code, ESP_LOG_INFO);
			}

			// Retransmit msg (ID + 1)
			msg.identifier += 1;
			for (int i=0;i<msg.data_length_code;i++)
			{
				msg.data[i] = msg.data[i] + 1;
			}
			can_transmit(&msg, 0);
		}
		else
		{
			LOG(I, "Failed to receive message\n");
		}
	}
}

#endif
