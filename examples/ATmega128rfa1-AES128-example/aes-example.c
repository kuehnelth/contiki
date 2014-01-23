#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include "atmega128rfa1-aes.h"
/*---------------------------------------------------------------------------*/
PROCESS(aes_example_process, "AES example process");
AUTOSTART_PROCESSES(&aes_example_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(aes_example_process, ev, data)
{
	//uint8_t key[16] = 	"0123456789abcdef";
	uint8_t key[16] = 	{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
	uint8_t *pkey = key;
	
	//uint8_t packet_data[32] = 	"Testdaten: Hallo, Welt.";
	//uint8_t packet_data[32] = 	"Testdaten: Hallo, Welt._________";	//genau 32 Byte
	uint8_t packet_data[128] = 	{0x54, 0x65, 0x73, 0x74, 0x64, 0x61, 0x74, 0x65,
				 	 0x6e, 0x3a, 0x20, 0x48, 0x61, 0x6c, 0x6c, 0x6f,
				 	 0x2c, 0x20, 0x57, 0x65, 0x6c, 0x74, 0x2e, 0x00,
				 	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t *pdata = packet_data;

	int len = strlen((const char *)packet_data);

	PROCESS_BEGIN();

	//write key to AES_KEY register
	atmega128rfa1_aes_set_key(pkey);
	//the mode index: either 0 = ECB or 1 = CBC.
	atmega128rfa1_aes_set_mode(0);

	printf("Plaintext before Encryption: %s\n", pdata);

	printf("Encryption...\n");

	atmega128rfa1_aes_cipher(pdata, len, 0);
	printf("Ciphertext after Encryption: %s\n", pdata);

	printf("Decryption...\n");
	atmega128rfa1_aes_cipher(pdata, len, 1);
	printf("Plaintext after Decryption: %s\n", pdata);

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
