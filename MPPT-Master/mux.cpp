#include <Arduino.h>
#include <SPI.h>
#include "MataHari.h"
#include "mux.h"



byte mux0 = 0;
byte mux1 = 0;
byte mux2 = 0;
byte mux3 = 0;

void muxVoltage( int chanId ) {

  switch( chanId % 8 ) {

    case 1: // 1 qnd 9
      mux0 = 1;
      mux1 = 0;
      mux2 = 0;
      mux3 = 0;
    break;
    
    case 2: // 2 and 10
      mux0 = 1;
      mux1 = 1;
      mux2 = 0;
      mux3 = 0;
    break;
    
    case 3: // 3 and 11
      mux0 = 1;
      mux1 = 0;
      mux2 = 1;
      mux3 = 0;
    break;
    
    case 4: // 4 and 12
      mux0 = 1;
      mux1 = 1;
      mux2 = 1;
      mux3 = 0;
    break;
    
    case 5: // 5 and 13
      mux0 = 0;
      mux1 = 1;
      mux2 = 1;
      mux3 = 1;
    break;
    
    case 6: // 6 and 14
      mux0 = 0;
      mux1 = 0;
      mux2 = 1;
      mux3 = 1;
    break;
    
    case 7: // 7 and 15
      mux0 = 0;
      mux1 = 1;
      mux2 = 0;
      mux3 = 1;
    break;
    
    case 0: // 8 and 16
      mux0 = 0;
      mux1 = 0;
      mux2 = 0;
      mux3 = 1;
    break;
  }
  
  mux( chanId );

}



void muxCurrent( int chanId ) {

  switch( chanId % 8 ) {

    case 1: // 1 qnd 9
      mux0 = 0;
      mux1 = 0;
      mux2 = 0;
      mux3 = 0;
    break;
    
    case 2: // 2 and 10
      mux0 = 0;
      mux1 = 1;
      mux2 = 0;
      mux3 = 0;
    break;
    
    case 3: // 3 and 11
      mux0 = 0;
      mux1 = 0;
      mux2 = 1;
      mux3 = 0;
    break;
    
    case 4: // 4 and 12
      mux0 = 0;
      mux1 = 1;
      mux2 = 1;
      mux3 = 0;
    break;
    
    case 5: // 5 and 13
      mux0 = 1;
      mux1 = 1;
      mux2 = 1;
      mux3 = 1;
    break;
    
    case 6: // 6 and 14
      mux0 = 1;
      mux1 = 0;
      mux2 = 1;
      mux3 = 1;
    break;
    
    case 7: // 7 and 15
      mux0 = 1;
      mux1 = 1;
      mux2 = 0;
      mux3 = 1;
    break;
    
    case 0: // 8 and 16
      mux0 = 1;
      mux1 = 0;
      mux2 = 0;
      mux3 = 1;
    break;
  }

  mux( chanId );
}


void mux( int chanId ) {

  if( chanId > 8 ) {
    digitalWrite( PIN_MUXEN2, LOW );
    digitalWrite( PIN_MUXEN1, HIGH );
  } else {
    digitalWrite( PIN_MUXEN1, LOW );
    digitalWrite( PIN_MUXEN2, HIGH );
  }

  digitalWrite( PIN_MUX0, mux0 );
  digitalWrite( PIN_MUX1, mux1 );
  digitalWrite( PIN_MUX2, mux2 );
  digitalWrite( PIN_MUX3, mux3 );
}



