/*
 * XBEE.h
 *
 *  Created on: May 24, 2025
 *      Author: Loochis
 */

#ifndef XBEE_DRIVER_INC_XBEE_H_
#define XBEE_DRIVER_INC_XBEE_H_

#include "stm32h7xx_hal.h"

    // DEFINITIONS
    // ------------------------------------------------------------------------------------
    #define PKT_RAWSIZE 68
	#define PKT_DATASIZE 64
	#define PKT_DELIMETER 0b10101010

	// STRUCTS
    // ------------------------------------------------------------------------------------
	// For this driver to work, you must set some variables before init
	// pktRx_max, pktTx_max

	typedef struct
	{
		UART_HandleTypeDef 	*uart_handle;	// ptr to UART_HandleTypeDef which interfaces with the XBEE

		uint8_t				state;			// State of operation; 0=IdleRecieve, 1=BusyTransmit

		uint8_t				*pkt_bufDMA;	// memory for the circular DMA buffer

		uint8_t				*pkt_bufPart;	// memory for the partially completed packet being collected

		uint8_t				pkt_DMAHead;	// index of the starting point of the packet WITHIN the circular DMA buffer

		// RX variables

		uint8_t				pktRx_state;	// State of RX; 0=Idle, 1=Busy

		uint8_t 			*pktRx_mem;		// Memory reserved for contents of RX'd packets

		uint8_t 			pktRx_max;		// Maximum number of receptions that can be stored before overflow

		uint8_t				pktRx_idxPush;	// Push position of RX circular buffer (FIFO)

		uint8_t				pktRx_idxPop;	// Pop position of RX circular buffer (FIFO)

		// TX variables

		uint8_t				pktTx_state;	// State of TX; 0x=Idle, 1=Busy

		uint8_t				*pktTx_mem;		// Memory reserved for contents of queued TX packets

		uint8_t				pktTx_max;		// Maximum number of transmissions that can be queued

		uint8_t				pktTx_idxPush;	// Push position of TX circular buffer

		uint8_t				pktTx_idxPop;	// Push position of TX circular buffer

	} XBEE_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

	uint8_t XBEE_Init(XBEE_HandleTypeDef *hxbee);
	uint8_t XBEE_RXPacket(XBEE_HandleTypeDef *hxbee, uint8_t **pRxBuffer, uint16_t *pkt_num);
	uint8_t XBEE_TXPacket(XBEE_HandleTypeDef *hxbee, uint8_t *pTxBuffer, uint16_t pkt_num);

	uint8_t XBEE_RX_DMACallback(XBEE_HandleTypeDef *hxbee);
	uint8_t XBEE_TX_DMACallback(XBEE_HandleTypeDef *hxbee);


#endif /* XBEE_DRIVER_INC_XBEE_H_ */
