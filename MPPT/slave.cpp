#include <Arduino.h>
#include "MPPT.h"
#include "slave.h"
#include <Wire.h>
#include <SPI.h>
#include "scpiparser.h"

void muxI2C( byte id ) {
	muxI2C( id, WireSlave );
}


void muxI2C( byte id, TwoWire &TheWire ) {

	TheWire.beginTransmission( 0b1110000 );
  if( id == 1 ) {
	  TheWire.write( 0b00000001 );
  } else {
	  TheWire.write( 0b00000010 );
  }
  TheWire.endTransmission();
}


void demuxI2C(  TwoWire &Wire ) {

  Wire.beginTransmission( 0b1110000 );
  Wire.write( 0b00000000 );
  Wire.endTransmission();
}


int16_t slave_readADS1015( byte address, byte channel ) {

  byte b1, b2;
  byte channelCfg;

  switch( channel ) {

    case 0:
      channelCfg = 0b01000000;
    break;

    case 1:
      channelCfg = 0b01010000;
    break;

    case 2:
      channelCfg = 0b01100000;
    break;

    case 3:
      channelCfg = 0b01110000;
    break;
  }

  WireSlave.beginTransmission( address );
  WireSlave.write( 0x00 | 0b00000001 ); // Write to the configuration register
  WireSlave.write( 0b10000101 | channelCfg ); // PGA at 2.048V, single shot, at a specific channel. Executes a conversion
  WireSlave.write( 0b10000111 ); // 1.6kSPS, disable comparator
  WireSlave.endTransmission();
  delay( 2 ); // Wait for the conversion to complete
  WireSlave.beginTransmission( address );
  WireSlave.write( 0x00 | 0b00000000 ); // Points to the data register
  WireSlave.endTransmission();

  if( WireSlave.requestFrom( address, 2 ) == 0 ) {
	  return 0x0000;
  }

  b1 = WireSlave.read();
  b2 = WireSlave.read();
  return ( ( b1 << 8 | b2 ) );   // The data is in the first 12 bits. We don't shift them to the right because we want to keep the signing. The software should divide by 32 (2^4) to get the real code value
}



/**
 * UV light sensor VEML6070
 * I2C address fixed
 * Write address: 0x38
 * Read address: 0x38, 0x39, 1 byte each
 */
/*
float slave_readVEML6070( ) {

  byte b1, b2;
  byte channelCfg;

  // Configuration
  WireSlave.beginTransmission( 0x38 );
  WireSlave.write( 0b00001111 );
  WireSlave.endTransmission();

  // LSB
  WireSlave.requestFrom( 0x38, 1 );
  b1 = WireSlave.read();

  // MSB
  WireSlave.requestFrom( 0x39, 1 );
  b2 = WireSlave.read();

  float sensitivity = 5e-3; // Sensitivity in mW cm-2
  return ( float ) ( ( b2 << 8 ) | b1 ) * sensitivity;
}*/


void aquireHumiditySensor( byte address ) {

    WireSlave.beginTransmission( address );
    WireSlave.write( 0x02 ); // Config register
    WireSlave.write( B00110000 );
    WireSlave.write( 0 );
    WireSlave.endTransmission();

    WireSlave.beginTransmission( address );
    WireSlave.write( 0x00 ); // Temperature register
    WireSlave.endTransmission();
    delay( 20 );

    WireSlave.requestFrom( address, 4 );
    int temperature1 = WireSlave.read();
    int temperature2 = WireSlave.read();

    int temperature = ( temperature1 << 8 | temperature2 );

    printDouble( ( (float) temperature ) / ( 65543 ) * 165.0 - 40.0, 6 );
    SerialUSB.println("");

    int humidity1 = WireSlave.read();
    int humidity2 = WireSlave.read();

    int humidity = ( humidity1 << 8 | humidity2 );

    printDouble( ( (float)  ( humidity ) / ( 65543 ) ), 4 );
    SerialUSB.println("");
}
