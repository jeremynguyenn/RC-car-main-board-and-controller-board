/*
 * XBEE.c
 *
 *  Created on: May 24, 2025
 *      Author: Loochis
 */

#include "XBEE.h"

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

uint8_t XBEE_Init(XBEE_HandleTypeDef *hxbee) {
	// Set up the variables
	hxbee->pkt_DMAHead   = 0;
	hxbee->pktRx_state   = 0;
	hxbee->pktTx_state   = 0;
	hxbee->pktRx_idxPush = 0;
	hxbee->pktRx_idxPop  = 0;
	hxbee->pktTx_idxPush = 0;
	hxbee->pktTx_idxPop  = 0;

	// Allocate mem
	hxbee->pkt_bufDMA  = malloc(PKT_RAWSIZE);
	hxbee->pkt_bufPart = malloc(PKT_RAWSIZE);
	hxbee->pktRx_mem   = malloc(hxbee->pktRx_max*PKT_RAWSIZE);
	hxbee->pktTx_mem   = malloc(hxbee->pktTx_max*PKT_RAWSIZE);

	// Begin the cyclic UART capture
	// Make sure the DMA is circular
	return HAL_UART_Receive_DMA(hxbee->uart_handle, hxbee->pkt_bufDMA, PKT_RAWSIZE);
}

// Gets a packet if there's one buffered
// 0 - Success
// 1 - Nothing to Get
uint8_t XBEE_RXPacket(XBEE_HandleTypeDef *hxbee, uint8_t **pRxBuffer, uint16_t *pkt_num) {
	// Check if there's an available packet
	if (hxbee->pktRx_idxPop == hxbee->pktRx_idxPush) return 1;

	// There's a packet up for grabs, get a pointer to the raw pkt incl. Header
	uint8_t *pRxInternal = hxbee->pktRx_mem + hxbee->pktRx_idxPop*PKT_RAWSIZE;

	hxbee->pktRx_idxPop += 1;
	hxbee->pktRx_idxPop %= hxbee->pktRx_max;

	// Compute the checksum
	uint8_t checksum = 0x00;
	for (uint8_t i = 0; i < PKT_RAWSIZE; i++) {
		if (i == 3) continue;
		checksum ^= pRxInternal[i];
	}

	// Packet corrupted, mismatch checksum
	if (pRxInternal[3] != checksum) return 1;

	// Get the 16 bit packet number field
	*pkt_num = 0;
	*pkt_num += pRxInternal[1];
	*pkt_num *= 256;
	*pkt_num += pRxInternal[2];

	*pRxBuffer = pRxInternal + 4;

	return 0;
}

uint8_t XBEE_TXPacket(XBEE_HandleTypeDef *hxbee, uint8_t *pTxBuffer, uint16_t pkt_num) {
	// Attempt to increment the packet buffer index
	// Note: We don't actually increment the index until the end of the function in case something goes wrong
	uint8_t pkt_idx = hxbee->pktTx_idxPush + 1;
	pkt_idx %= hxbee->pktTx_max;

	// No space left in the buffer, we have to drop this packet
	if (pkt_idx == hxbee->pktTx_idxPop) return 1;

	// Get a reference to this memory location
	uint8_t *newPkt = hxbee->pktTx_mem + hxbee->pktTx_idxPush*PKT_RAWSIZE;

	// Construct the packet
	newPkt[0] = PKT_DELIMETER;
	newPkt[1] = pkt_num >> 8;
	newPkt[2] = pkt_num & 0x00FF;

	memcpy(newPkt + 4, pTxBuffer, PKT_DATASIZE);

	// Compute the checksum
	newPkt[3] = 0x00;
	for (uint8_t i = 0; i < PKT_RAWSIZE; i++) {
		if (i == 3) continue;
		newPkt[3] ^= newPkt[i];
	}

	// Try and immediately send the packet
	// TODO: Set the bust state, interrupts
	return HAL_UART_Transmit(hxbee->uart_handle, newPkt, PKT_RAWSIZE, 30);	// Transmit the buffer
}

// DMA CALLBACKS
// ------------------------------------------------------------------------------------

// Process incoming packets on DMA callback
// 0 - Success
// 1 - Buffer full
// 2 - Packet malformed
// 3 - Checksum mismatch
uint8_t XBEE_RX_DMACallback(XBEE_HandleTypeDef *hxbee) {
	// Attempt to increment the packet buffer index
	// Note: We don't actually increment the index until the end of the function in case something goes wrong
	uint8_t pkt_idx = hxbee->pktRx_idxPush + 1;
	pkt_idx %= hxbee->pktRx_max;

	// No space left in the buffer, we have to drop this packet
	if (pkt_idx == hxbee->pktRx_idxPop) return 1;

	// Create a temporary buffer for the complete packet
	uint8_t pkt_cplt[PKT_RAWSIZE];

	// Store the last readHead, we need this for packet reconstruction if bytes got lost
	uint8_t old_head = hxbee->pkt_DMAHead;

	// Find the delimeter
	uint8_t found_delim = 0;
	for (uint8_t circular_ptr = 0; circular_ptr < PKT_RAWSIZE; circular_ptr++) {
		uint8_t packet_ptr = (circular_ptr + hxbee->pkt_DMAHead) % PKT_RAWSIZE;
		if (hxbee->pkt_bufDMA[packet_ptr] == PKT_DELIMETER) {
			found_delim = 1;					// Update flag
			hxbee->pkt_DMAHead = packet_ptr;	// Move the readHead
			break;
		}
	}

	// Couldn't find the delimeter, this packet is FUBAR, discard the whole thing
	if (!found_delim) return 2;

	// Copy the partial packet contents into the completed packet buffer
	memcpy(pkt_cplt, hxbee->pkt_bufPart, PKT_RAWSIZE);

	// Copy the new packet contents into the partial packet buffer
	memcpy(hxbee->pkt_bufPart, hxbee->pkt_bufDMA + hxbee->pkt_DMAHead, PKT_RAWSIZE - hxbee->pkt_DMAHead);

	// finish the old packet
	// Account for dropped byte underflow
	// TODO: We can actually correct this instead of just giving up
	if (old_head < hxbee->pkt_DMAHead) {
		return 2;
	}

	uint16_t head_slip = old_head - hxbee->pkt_DMAHead;	// How many bytes were dropped
	memset(pkt_cplt + (PKT_RAWSIZE - old_head), 0x00, head_slip); 									 // Zero dropped bytes
	memcpy(pkt_cplt + (PKT_RAWSIZE - old_head) + head_slip, hxbee->pkt_bufDMA, hxbee->pkt_DMAHead); // Fill in missing bytes
	// Note that the above method attempts to reconstruct packets when bytes are dropped
	// What this looks like in memory:
	// B0 B1 B2 B3 B4 XX XX B7
	// In the event of a single dropped byte, this is accurate, if more than one gets dropped this may become inaccurate

	// TODO: CHECKSUM

	// If everything went well, increment the index and move the data into the RX buffer
	hxbee->pktRx_idxPush = pkt_idx;
	memcpy(hxbee->pktRx_mem + hxbee->pktRx_idxPush*PKT_RAWSIZE, pkt_cplt, PKT_RAWSIZE);

	return 0;
}
