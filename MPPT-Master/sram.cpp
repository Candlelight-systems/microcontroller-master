#include <Arduino.h>
#include <SPI.h>
#include "MataHari.h"
#include "sram.h"
#include "parameters.h"

bool SRAM_READ_OPEN = false;


void SRAM_STORE_IV( uint8_t chanId, uint16_t& address, int32_t voltage, int32_t current, int8_t pgaValue ) {
	
	const uint32_t SRAM_ADDRESS = SRAM_NUMBER_BYTES_PER_CHANNEL * ( chanId - 1 ) + address * 7;
	
	SPI.beginTransaction( SPISettings( 1000000, MSBFIRST, SPI_MODE0 ) ); // Ok get ready for SPI
	digitalWrite( SRAM_STORAGE_CS_PIN, LOW );
	
	// WRITE MODE
	SPI.transfer( 0x02 );
	
	// Followed by the 24-bit address. The first 7 will be ignored.
	SPI.transfer( SRAM_ADDRESS >> 16 );
	SPI.transfer( SRAM_ADDRESS >> 8 );
	SPI.transfer( SRAM_ADDRESS );
	
	SPI.transfer( voltage >> 24 );
	SPI.transfer( voltage >> 16 );
	SPI.transfer( voltage >> 8 );
	
	SPI.transfer( current >> 24 );
	SPI.transfer( current >> 16 );
	SPI.transfer( current >> 8 );
	
	SPI.transfer( pgaValue );
	
	address = address + 1; 

	digitalWrite( SRAM_STORAGE_CS_PIN, HIGH );
	SPI.endTransaction();
}

void SRAM_INIT() {
	
	pinMode( SRAM_STORAGE_CS_PIN, OUTPUT );
	pinMode( SRAM_STORAGE_HOLD_PIN, OUTPUT );
	
	
	digitalWrite( SRAM_STORAGE_CS_PIN, HIGH );
	digitalWrite( SRAM_STORAGE_HOLD_PIN, HIGH );



	SPI.beginTransaction( SPISettings( 1000000, MSBFIRST, SPI_MODE0 ) );
	digitalWrite( SRAM_STORAGE_CS_PIN, LOW );

	SPI.transfer( 0x01 ); // Write the mode register
	SPI.transfer( 0b01000000 ); // Ensure sequential operation (not limited by the page limit)
	

	digitalWrite( SRAM_STORAGE_CS_PIN, HIGH );
	SPI.endTransaction();
}


void SRAM_READ_INIT( uint8_t chanId, uint16_t address ) {
	
	const uint32_t SRAM_ADDRESS = SRAM_NUMBER_BYTES_PER_CHANNEL * ( chanId - 1 );
	
	if( SRAM_READ_OPEN ) {
		SRAM_READ_TERMINATE();
	}
	
	SRAM_READ_OPEN = true;
	SPI.beginTransaction( SPISettings( 100000, MSBFIRST, SPI_MODE0 ) );
	
	digitalWrite( SRAM_STORAGE_HOLD_PIN, HIGH );
	digitalWrite( SRAM_STORAGE_CS_PIN, LOW );

	delayMicroseconds( 10 );
	SPI.transfer( 0x03 );
	SPI.transfer( SRAM_ADDRESS >> 16 );
	SPI.transfer( SRAM_ADDRESS >> 8 );
	SPI.transfer( SRAM_ADDRESS >> 0 );
	delayMicroseconds( 10 );
	digitalWrite( SRAM_STORAGE_HOLD_PIN, LOW );	
}

void SRAM_READ_TERMINATE() {
	

	digitalWrite( SRAM_STORAGE_HOLD_PIN, HIGH );
	digitalWrite( SRAM_STORAGE_CS_PIN, HIGH );
	SPI.endTransaction();
	SRAM_READ_OPEN = false;
}



int32_t SRAM_READ( uint8_t numBytes ) {
	
	digitalWrite( SRAM_STORAGE_HOLD_PIN, HIGH );

	uint8_t shifter = 0;	
	int32_t v = 0;
	uint8_t b;
	
	while( shifter < numBytes ) {
		v = v | ( SPI.transfer( 0x00 ) << ( 24 - shifter * 8 ) );
		shifter++;
	}

	digitalWrite( SRAM_STORAGE_HOLD_PIN, LOW );
	
	return v;
}
