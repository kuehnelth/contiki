/**
 * \file
 *         AES encryption/decryption functions for ATmega128rfa1.
 * 	   See also reference: ATmega128rfa1 datasheet (9.8.8 Security Module (AES), page 94ff)
 * \author
 *         Stefan Lorenz <s69401@informatik.htw-dresden.de>
 */

#include "dev/atmega128rfa1-aes.h"
#include <avr/io.h>
#include <stdio.h>

#define KEYLEN 16
#define MAX_DATALEN 16
#define MIN(a,b) ((a) < (b)? (a): (b))
//AES_STATUS_RY = 0 ?!?
#define AES_STATUS_RY 0
/*---------------------------------------------------------------------------*/
/**
 * Step 1: Key Setup:
 * Write encryption or decryption key to KEY
 * buffer(16 consecutive byte writes to AES_KEY)
 *
 * ATmega128rfa1 only supports AES-128 standard(128 Bit/16 Byte key-length)
 */
void
atmega128rfa1_aes_set_key(uint8_t *key)
{
	int i;
	//write aes-key into buffer 
	for(i = 0; i < 16; i++) {
		AES_KEY = key[i];
	}
}
/*---------------------------------------------------------------------------*/
/**
 * Step 2: AES configuration:
 * Select AES mode: ECB or CBC
 * Select encryption or decryption
 * Enable the AES Encryption Ready Interrupt AES_READY
 */
void
atmega128rfa1_aes_set_mode(int index)
{
	// index = 0 -> ECB / index = 1 -> CBC
	//AES_MODE = index;
	switch(index)
	{	
		case 0:
			AES_CTRL &= ~(1 << AES_MODE);
			break;
		case 1:	
			AES_CTRL |= ~(1 << AES_MODE);
			break;
	}
}
/*---------------------------------------------------------------------------*/
void
atmega128rfa1_aes_set_direction(int index)
{
	// index = 0 -> encryption / index = 1 -> decryption
	//AES_DIR = index;
	switch(index)
	{
		case 0:
			AES_CTRL &= ~(1 << AES_DIR);
		case 1:
			AES_CTRL |= ~(1 << AES_DIR);
	}
}
/*---------------------------------------------------------------------------*/
/**
 * Step 3: Write Data:
 * Write plain text or cipher text to DATA buffer
 * (16 consecutive byte writes to AES_STATE)
 */
void
atmega128rfa1_aes_write_data16(uint8_t *data, int len)
{
	int i;
	len = MIN(len, MAX_DATALEN);
	//write string to encrypt/decrypt into buffer 
   	for (i = 0; i < len; i++) { 
      		AES_STATE = data[i];
   	} 
}
/*---------------------------------------------------------------------------*/
/**
 * Step 4: Start operation:
 * Start AES operation
 */
void
atmega128rfa1_aes_start_operation()
{
	//The encryption or decryption is initiated with bit AES_REQUEST = 1
	//AES_REQUEST = 1;
	AES_CTRL |= (1 << AES_REQUEST);
}
/*---------------------------------------------------------------------------*/
/**
 * Step 5: Wait for AES finished:
 * 5.1. AES_READY IRQ or
 * 5.2. polling AES_DONE bit(register AES_STATUS) or
 * 5.3. wait for 24 μs
 * 
 * Wait until AES encryption/decryption is finished
 * successfully
 */
void
atmega128rfa1_aes_wait()
{
	while (0 == (AES_STATUS & (1 << AES_STATUS_RY)));
}
/*---------------------------------------------------------------------------*/
/**
 * Step 6: Read Data:
 * Read cipher text or plain text from DATA buffer
 * (16 consecutive byte reads from AES_STATE)
 */
void
atmega128rfa1_aes_read_data16(uint8_t *data, int len)
{
	int i;
	len = MIN(AES_STATE, MAX_DATALEN);
	for(i = 0; i < len; i++) { 
      		data[i] = AES_STATE; 
   	}
}
/*---------------------------------------------------------------------------*/
/**
 * encrypt/decrypt full data:
 * step by step encryption/decryption in blocks of 16 byte
 */
void
atmega128rfa1_aes_cipher(uint8_t *data, int len, int direction)
{
	int i;

	//direction: encrypt
	atmega128rfa1_aes_set_direction(direction);

	//write data
	for(i = 0; i < len; i = i + MAX_DATALEN) {
		atmega128rfa1_aes_write_data16(data + i, MIN(len - i, MAX_DATALEN));
		atmega128rfa1_aes_start_operation();
		atmega128rfa1_aes_wait();
		atmega128rfa1_aes_read_data16(data + i, MIN(len - i, MAX_DATALEN));
	}
}
