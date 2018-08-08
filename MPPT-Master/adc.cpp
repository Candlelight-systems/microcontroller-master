
#include <Arduino.h>
#include <SPI.h>
#include "MataHari.h"
#include "adc.h"

#include "mux.h"

byte ADC_CFG0 = 0x00;
byte ADC_CFG1 = 0x00;
byte ADC_CFG2 = 0x00;
byte ADC_OFC0 = 0x00;
byte ADC_OFC1 = 0x00;
byte ADC_OFC2 = 0x00;
byte ADC_FSC0 = 0x00;
byte ADC_FSC1 = 0x00;
byte ADC_FSC2 = 0x00;


volatile bool adcState = false;
volatile bool adcError = false;
volatile unsigned long readytime;

void sendPGA( byte toSend ) {
	SPI.transfer( toSend );
}

byte readPGA() {
	byte response = SPI.transfer( 0x00 );
	return response;
}

void PGA_SET_GAIN( byte gainValue ) {
	PGA_WRITE_REGISTER( 0x00, gainValue << 3 );
}

void PGA_MUX_PD( ) {
	PGA_WRITE_REGISTER( 0x06, B00011000 );
}

void PGA_MUX_MAIN(  ) {
	PGA_WRITE_REGISTER( 0x06, B01100000 );
}

void PGA_WRITE_REGISTER( byte registerNumber, byte newValue ) {
SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI

	PGA_ENABLE();
	byte commandByte = ( B01000000 | ( registerNumber & 0x0F ) );
	byte checkSum = 0x9B + commandByte + newValue;
	sendPGA( commandByte );
	sendPGA( newValue );
//	sendPGA( checkSum ); // Will only send the lowest 8 bits. Anyway it's a byte, so default carry is enabled
	PGA_DISABLE();
SPI.endTransaction();
}

byte PGA_READ_REGISTER( byte registerNumber ) {
SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI

	PGA_ENABLE();
	byte commandByte = ( B10000000 | ( registerNumber & 0x0F ) );
	byte checkSum = 0x9B + registerNumber;
	sendPGA( commandByte );
	
	//sendPGA( checkSum );
	byte b = readPGA( );
	PGA_DISABLE();
SPI.endTransaction();
	return b;
}

byte PGA_CS_CONTROL( ) {

	PGA_ENABLE();
	byte commandByte = ( B11000010 );
	byte checkSum = 0x9B + commandByte;
	sendPGA( commandByte );
//	sendPGA( checkSum );
}

void PGA_DISABLE() {
	digitalWrite( PIN_PGA_CS, HIGH );
}

void PGA_ENABLE() {
	digitalWrite( PIN_PGA_CS, LOW );
}

void ADC_WAKEUP() {
	ADC_ENABLE();
	sendADC( 0x02 );
	ADC_DISABLE();
}

void ADC_SLEEP() {
	sendADC( 0x04 );
}

void ADC_RESET() {
	sendADC( 0x06 );
}

byte OOR;

int32_t ADC_READ_ONCE() {

	int32_t val = 0;
	SPI.beginTransaction( SPISettings(4000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI
	
	ADC_ENABLE();
	sendADC( 0x12 );
	byte v1 = readADC( 0xFF ); // Send NOP
	byte v2 = readADC( 0xFF ); // Send NOP
	byte v3 = readADC( 0xFF ); // Send NOP
	// Checksum byte
	byte v4 = readADC( 0xFF ); // Send NOP
	val = ( v1 << 24 & 0xFF000000 ) | ( v2 << 16 & 0x00FF0000 ) | ( v3 << 8 & 0x0000FF00 );
	ADC_DISABLE();
	
	//OOR = v4 >> 7;
	
	SPI.endTransaction();
	return val;
}

byte ADC_OUT_OF_RANGE() {
	return OOR;
}


void sendADC( byte toSend ) {
	SPI.transfer( toSend );
}

byte readADC() {
	byte response = SPI.transfer( 0x00 );
	return response;
}

byte readADC( byte value ) {
	byte response = SPI.transfer( value );
	return response;
}

void ADC_SET_SPEED( byte speedByte ) {
	ADC_CFG2 |= ( speedByte & 0b00000111 );
	ADC_WRITE_CONFIG();
}

void ADC_WRITE_REGISTER( byte registerNumber, byte newValue ) { // Up to 15 registers. Details on DS page 47
	SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI
	
	ADC_ENABLE();
	sendADC( ( B01000000 | ( registerNumber ) ) );
	sendADC( 0x00 ); // Only 1 register at the time
	sendADC( newValue );
	ADC_DISABLE();
	SPI.endTransaction();
}



void ADC_WRITE_CONFIG( ) { // Up to 15 registers. Details on DS page 47
	SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI

	ADC_ENABLE();
	sendADC( ( B01000000 ) );
	sendADC( 0x02 ); // Only 1 register at the time
	sendADC( ADC_CFG0 );
	sendADC( ADC_CFG1 );
	sendADC( ADC_CFG2 );
	ADC_DISABLE();
	SPI.endTransaction();
}



void ADC_WRITE_COMMAND( byte command ) { // Up to 15 registers. Details on DS page 47
	SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI
	
	ADC_ENABLE();
	sendADC( command );
	ADC_DISABLE();
SPI.endTransaction();
}


byte ADC_READ_REGISTER( byte registerNumber ) {
	
	SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) ); // Ok get ready for SPI
	
	byte b;
	ADC_ENABLE();
	sendADC( ( B00100000 | ( registerNumber & 0x0F ) ) );
	readADC( 0x00 ); // Only 1 register at the time
	b = readADC();
	ADC_DISABLE();
	
	SPI.endTransaction();
	return b;
}

void ADC_ENABLE() {
	PGA_CS_CONTROL();
}

void ADC_DISABLE() {
	PGA_DISABLE();
}

void ADC_PINREADY_CHANGE() {
	adcState = true;
}

void ADC_MONITOR_PINREADY() {

	readytime = millis();
	
	while ( !adcState ) {
		if(  millis() - readytime > 100 ) {
			adcState = false;
			adcError = true;
			digitalWrite( PIN_LED_INDICATOR, LOW );
			return;
		}
	}

	adcState = false;
}


void ADCSetup() {

	// Setting up the PGA
	PGA_WRITE_REGISTER( 0x01, B00000001 ); // Soft reset
	PGA_WRITE_REGISTER( 0x0B, B00000000 ); // Enable checksum
	PGA_WRITE_REGISTER( 0x02, B00000100 ); // Write the SPI mode for the ADC on pin 2
	PGA_WRITE_REGISTER( 0x03, B00000101); // Current buffer delay: 5 * 4us = 20us approx.
	PGA_WRITE_REGISTER( 0x08, B00101100); // Output direction for GPIOs. Out for GPIO5 (BUFAOut), GPIO3 (EFOut) and GPIO2 (ECS for the ADC)
	PGA_WRITE_REGISTER( 0x09, B00000100); // Enable the ECS on pin 2
	PGA_WRITE_REGISTER( 0x0A, B11101000); // Disable the MUX feature. Inverse BUFOut polarity. Disable errors when current buffer is on
	PGA_WRITE_REGISTER( 0x0C, B01111000); // Enable SYNCIn on GPIO6. Enable BUFAOut on GPIO5. Enable BUFTIn on GPIO4. ENable EFOut on GPIO3
	
	ADC_CFG0 = B00000101; // Power up reference
	ADC_CFG1 = B00000000; // Mux internal reference in
	ADC_CFG1 |= B00000101; // Add some delay to the conversion
	ADC_CFG1 |= B01000000; // Checksum enable
	ADC_CFG1 |= B10000000; // OFR (Flag) enable. In this mode the flag will appear as the bit 7 (first bit shifted out) of the checksum
	ADC_CFG2 = B00110100; // Pulse mode (no gating)
	
	ADC_WRITE_COMMAND( 0x06 );
	ADC_WRITE_COMMAND( 0x11 );
	ADC_WRITE_COMMAND( 0x02 );
//	ADC_READ_REGISTER( ADC_CFG0_ADDR );
	ADC_WRITE_CONFIG(  );
	
	attachInterrupt( digitalPinToInterrupt( 13 ), ADC_PINREADY_CHANGE, FALLING );
}



// Reading voltage code
int32_t ADCReadVoltageCode( int chanId ) {
	return ADCReadVoltageCode( chanId, 0b00000011 );
}


// Reading voltage code
int32_t ADCReadVoltageCode( int chanId, byte pgaValue ) {

	int32_t code;
	do {
		adcError = false;
		digitalWrite( PIN_PGA_CURRENT_BUFFER, HIGH ); // Enable the buffer
		PGA_SET_GAIN( pgaValue ); // Gain of 1 for the voltage
		muxVoltage( chanId ); // Mux in the proper signal
		digitalWrite( PIN_PGA_CURRENT_BUFFER, LOW ); // Disable the buffer. As soon as it happens, conversion will start (after proper delays)
		delayMicroseconds( 80 );
		adcState = false;
		ADC_MONITOR_PINREADY(); // Wait until we get it
	} while ( adcError );
	
	return ADC_READ_ONCE();
}


// Reading current code
int32_t ADCReadCurrentCode( int chanId, byte pgaValue ) {
	int32_t code;
	
	do {
		adcError = false;
		
		digitalWrite( PIN_PGA_CURRENT_BUFFER, HIGH ); // Enable the buffer
		PGA_SET_GAIN( pgaValue ); // Gain of 1 for the voltage
		muxCurrent( chanId ); // Mux in the proper signal
		digitalWrite( PIN_PGA_CURRENT_BUFFER, LOW ); // Disable the buffer. As soon as it happens, conversion will start (after proper delays)
		delayMicroseconds( 80 );
		adcState = false;
		ADC_MONITOR_PINREADY(); // Wait until we get it
		
	} while ( adcError );

	return ADC_READ_ONCE();
}


// Reading PD code
int32_t ADCReadPDCode( ) {
	
	int32_t code;
	
	do {
		
		adcError = false;
		digitalWrite( PIN_PGA_CURRENT_BUFFER, HIGH ); // Enable the buffer
		PGA_MUX_PD();
		PGA_SET_GAIN( 0b00000011 ); // Gain of 1 for the voltage
		digitalWrite( PIN_PGA_CURRENT_BUFFER, LOW ); // Disable the buffer. As soon as it happens, conversion will start (after proper delays)
	
		delayMicroseconds( 80 );
		adcState = false;
		ADC_MONITOR_PINREADY(); // Wait until we get it
		PGA_MUX_MAIN();
	} while ( adcError );
		
	return ADC_READ_ONCE();
}


