#include <Arduino.h>
#include <SPI.h>
#include "MataHari.h"
#include "parameters.h"
#include "dac.h"


byte DAC1 = 0x00;
byte DAC2 = 0x00;
byte DAC3 = 0x00;

void DACUpdate( int dacID ) {

  int dacPin;

  if( dacID == 1 ) {
    dacPin = PIN_DAC_CS_1;
  } else if( dacID == 2 ) {
    dacPin = PIN_DAC_CS_2;  
  }
  
  SPI.beginTransaction( SPISettings( 1000000, MSBFIRST, SPI_MODE1 ) );
  digitalWrite( dacPin, LOW );
  SPI.transfer( DAC1 );
  SPI.transfer( DAC2 );
  SPI.transfer( DAC3 );

  digitalWrite( dacPin, HIGH );
   SPI.endTransaction();
  DAC1 = 0;
  DAC2 = 0;
  DAC3 = 0;
}

void configureDAC() {

  DAC1 = B00000000;
  DAC2 = B00000001;
  DAC3 = B11000000;
  DACUpdate( 1 );

  DAC1 = B00000000;
  DAC2 = B00000001;
  DAC3 = B11000000;
  DACUpdate( 2 );

  // Offset A
  DAC1 = B00000011;
  DAC2 = B10101010;
  DAC3 = B10110000;
  DACUpdate( 1 );

  // Offset A
  DAC1 = B00000011;
  DAC2 = B10101010;
  DAC3 = B10110000;
  DACUpdate( 2 );

  // Offset B
  DAC1 = B00000100;
  DAC2 = B10101010;
  DAC3 = B10110000;
  DACUpdate( 1 );

  // Offset B
  DAC1 = B00000100;
  DAC2 = B10101010;
  DAC3 = B10110000;
  DACUpdate( 2 );
 
}

void setDAC( int channel, uint16_t value ) {

  byte dacId = 1;
  byte address = B00000000;
  
  switch( channel % 8 ) {
    case 1: address |= B00001000; break;
    case 2: address |= B00001001; break;
    case 3: address |= B00001010; break;
    case 4: address |= B00001011; break;
    case 5: address |= B00001100; break;
    case 6: address |= B00001101; break;
    case 7: address |= B00001110; break;
    case 0: address |= B00001111; break;
  }

  if( channel > 8 ) {
    dacId = 1;
  } else {
    dacId = 2;
  }
  
  // value from 0 to 4096
  DAC1 = address;
  DAC2 = 0;
  DAC3 = 0;
  #if DAC_RESOLUTION == 12
  DAC2 = ( value << 4 ) >> 8; // Getting the first 8 bits
  DAC3 = ( value << 4 ) & B11110000; // Getting the last 4 bits
  # elif  DAC_RESOLUTION == 14
  DAC2 = ( value << 2 ) >> 8; // Getting the first 8 bits
  DAC3 = ( value << 2 ) & B11111100; // Getting the last 4 bits
  # else 
  DAC2 = ( value << 0 ) >> 8; // Getting the first 8 bits
  DAC3 = ( value << 0 ) & B11111111; // Getting the last 4 bits
  # endif
  
  DACUpdate( dacId );
}

