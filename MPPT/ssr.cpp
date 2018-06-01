
#include <Arduino.h>
#include "MPPT.h"
#include "pid.h"

struct ssr {
	unsigned char enabled = 0;
};

struct ssr* ssrs[ 3 ] = {
  NULL,
  new ssr,
  new ssr
};

void SSRSetValue( unsigned char chanId, uint32_t newValue ) {

	if( ssrs[ chanId ]->enabled != 1 ) {
		return;
	}

	if( chanId == 1 ) {
		REG_TCC0_CC1 = newValue;         // TCC0 CC3 - on D7
		while (TCC0->SYNCBUSY.bit.CC1 );                // Wait for synchronization
	} else if( chanId == 2 ) {
		REG_TCC0_CC2 = newValue;
		while( TCC0->SYNCBUSY.bit.CC2 );
	}
}

void SSRSetValue( unsigned char chanId, float value ) {

if( value > 1 ) {
	value = 1;
}

if( value < 0 ) {
	value = 0;
}

	uint32_t newValue = (uint32_t) ( value * ( uint32_t ) ( 0xFFFFFF ) );

	SSRSetValue( chanId, newValue );
}

void SSRDisable( unsigned char chanId ) {

	SSRSetValue( chanId, (uint32_t) 0 );
	ssrs[ chanId ]->enabled = 0;
}

void SSREnable( unsigned char chanId ) {

	SSRSetValue( chanId, (uint32_t) 0 );
	ssrs[ chanId ]->enabled = 1;
}
