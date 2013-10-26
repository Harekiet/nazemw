#include "board.h"
#include "naze.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage

//Header written to the start of the flash followed by actual config data
typedef struct {
	uint16_t crc;
	uint16_t version;
	uint16_t size;
} ConfigHeader_t;

//Size the config header takes up when 4byte aligned
#define FLASH_HEADERSIZE ( 4 * ( ( 3 + sizeof( ConfigHeader_t ) ) / 4) )
//Remaining size of the flash for config data
#define FLASH_AVAILABLE  ( FLASH_PAGE_SIZE - FLASH_HEADERSIZE )

//Simple slow crc16 calculation without lookup table
static uint16_t makeCRC16( uint16_t remainder, uint32_t size, const void* data ) {
	enum {
		WIDTH = 16,
		TOPBIT = (1 << WIDTH) - 1,
		POLYNOMIAL = 0x8810
	};

	const uint8_t* input = (const uint8_t*) data;
	uint32_t i;
	uint8_t bit;

	for ( i = 0; i < size; i++) {
		//Feed new byte into the value
		remainder ^= input[i] << (WIDTH - 8);

		/*
		 * Perform modulo-2 division, a bit at a time.
		 */
		for ( bit = 8; bit > 0; --bit) {
			/*
			 * Try to divide the current data bit.
			 */
			if (remainder & TOPBIT) {
				remainder = (remainder << 1) ^ POLYNOMIAL;
			} else {
				remainder = (remainder << 1);
			}
		}
	}
	return remainder;
}


bool boardSaveSettings( uint16_t version, uint16_t size, const void* data ) {
	if ( size > FLASH_AVAILABLE )
		return false;
	uint16_t i;
	ConfigHeader_t header;
	header.version = version;
	header.size = size;
	header.crc = makeCRC16( 0, size, data );
#if 0
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if (FLASH_ErasePage(FLASH_WRITE_ADDR) == FLASH_COMPLETE) {
    	//Place the header
    	for ( i = 0; i < sizeof( header ); i +=4 ) {

    	}

        for (i = 0; i < size; i += 4) {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *) ((char *) &cfg + i));
            if (status != FLASH_COMPLETE)
                break;          // TODO: fail
        }
    }
    FLASH_Lock();
#endif
	return true;
}

bool boardLoadSettings( uint16_t version, uint16_t size, void* data ) {
	if ( size > FLASH_AVAILABLE )
		return false;
	const ConfigHeader_t* header = ( const ConfigHeader_t* ) ( FLASH_WRITE_ADDR );
	const void* source = ( const void* ) ( FLASH_WRITE_ADDR + FLASH_HEADERSIZE );
	if ( header->version != version )
		return false;
	if ( header->size != size )
		return false;
	if ( header->crc != makeCRC16( 0, size, source ) )
		return false;
	//Copy the config data from flash
	memcpy( data, source, size );
	return true;
}

//Beep beep
void boardBeepOn() {
	BEEP_ON;
}

void boardBeepOff() {
	BEEP_OFF;
}

void boardBeepToggle() {
	BEEP_TOGGLE;
}

//Blink Blinl

uint8_t boardLedCount() {
	return 2;
}

void boardLedToggle( uint8_t index ) {
	if ( !index ) {
		LED0_TOGGLE;
	} else {
		LED1_TOGGLE;
	}
}

void boardLedOn( uint8_t index ) {
	if ( !index ) {
		LED0_ON;
	} else {
		LED1_ON;
	}
}

void boardLedOff( uint8_t index ) {
	if ( !index ) {
		LED0_OFF;
	} else {
		LED1_OFF;
	}
}

void boardInit( BoardConfig_t* config ) {
	//Try to detect sensors
//	sensorsAutodetect();
}

//Trigger a reset of the board
void boardReset() {
    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

void boardBootLoader() {
	//Set the flag to indicate a bootloader
	// 1FFFF000 -> 20000200 -> SP
   // 1FFFF004 -> 1FFFF021 -> PC
   *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
   boardReset();
}

void boardFault( uint32_t code ) {
    LED1_ON;
    LED0_OFF;
    while (1) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delayMilli(475 * code - 2);
        BEEP_ON
        delayMilli(25);
        BEEP_OFF;
    }
}

