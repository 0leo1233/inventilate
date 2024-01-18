/*
 * connector_lin_comm_test.h
 *
 *  Created on: 11 Aug 2022
 *      Author: Sundaramoorthy Jeyakodi LTTS
 */

#ifndef CONNECTOR_LIN_COMM_TEST_H_
#define CONNECTOR_LIN_COMM_TEST_H_

#include "configuration.h"

#ifdef CONNECTOR_LIN_COMM_TEST

#include "connector.h"

typedef struct __lin_tx_frame
{
    uint8_t device_id;
    uint8_t data[8];
    uint8_t size;
}LIN_TX_FRAME;

extern CONNECTOR connector_lin_comm_test;

void lin_tx_data(LIN_TX_FRAME* ptr_lin_tx);

#endif /* CONNECTOR_LIN_COMM_TEST */

#endif /* CONNECTOR_LIN_COMM_TEST_H_ */
