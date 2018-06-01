#include <Arduino.h>
#include "MPPT.h"
#include "scpiparser.h"

struct light {
  unsigned long time_checking = 0;
  unsigned long time_processing = 0;

  float pd_scaling_ma_to_sun = -6.93;
  float setpoint = 1;
  uint8_t master = 0;
  uint32_t pwm = 255;
};

struct light* lights[ 3 ] = {
  NULL,
  new light,
  new light
};

#if CLIEND_ID == 10023
lights[ 1 ]->master = 2;
#endif

bool light_isenabled( byte chanId ) {
  if ( chanId == 1 ) {
    return digitalRead( PIN_LIGHT_1_ONOFF_READ ) == HIGH;
  } else if ( chanId == 2 ) {
    return digitalRead( PIN_LIGHT_2_ONOFF_READ ) == HIGH;
  }

  return false;
}


bool light_isautomatic( byte chanId ) {
  if ( chanId == 1 ) {
    return digitalRead( PIN_LIGHT1_MODE_DEC ) == HIGH;
  } else if ( chanId == 2 ) {
    return digitalRead( PIN_LIGHT2_MODE_DEC ) == HIGH;
  }

  return false;
}


void lightSetSetpoint( unsigned char chanId, float setPoint ) {
  if ( chanId == 1 ) {
	lights[ 1 ] -> setpoint = setPoint;
  } else if ( chanId == 2 ) {
	lights[ 2 ] -> setpoint = setPoint;
  }
}

void lightSetScaling( unsigned char chanId, float scaling ) {
  if ( chanId == 1 ) {
	lights[ 1 ] -> pd_scaling_ma_to_sun = scaling;
  } else if ( chanId == 2 ) {
	lights[ 2 ] -> pd_scaling_ma_to_sun = scaling;
  }
}

void light_setPWM( byte chanId, uint32_t pwm ) {
  if ( chanId == 1 ) {
	REG_TCC1_CC0 = pwm;         // TCC1 CC0
	while (TCC1->SYNCBUSY.bit.CC0 ); // Wait for synchronization
  } else if ( chanId == 2 ) {
	 REG_TCC1_CC1 = pwm;         // TCC1 CC1
	 while (TCC1->SYNCBUSY.bit.CC1 ); // Wait for synchronization
  }

  if( lights[ chanId ]->master > 0 ) {
	 light_setPWM( lights[ chanId ]->master, pwm );
  }
}

float measurePDSun( byte pdId ) {
  float f;
  if ( pdId == 1 ) {
    slaveCommand( COMMAND_REQ_PD1 );

    readFromWire( 4 );

    f = readFloatFromWire();
  } else if ( pdId == 2 ) {
    slaveCommand( COMMAND_REQ_PD2 );

    readFromWire( 4 );

    f = readFloatFromWire();
  } else {
    return -1.0;
  }

  return f * lights[ pdId ]->pd_scaling_ma_to_sun;
}


float measurePD( byte pdId ) {
  float f;

  if ( pdId == 1 ) {
    slaveCommand( COMMAND_REQ_PD1 );
    readFromWire( 4 );
    f = readFloatFromWire();
  } else if ( pdId == 2 ) {
    slaveCommand( COMMAND_REQ_PD2 );
    readFromWire( 4 );
    f = readFloatFromWire();
  } else {
    return -1.0;
  }

  return f;
}


int32_t measurePDCode( byte pdId ) {
  float f;
  int32_t intv;

  if ( pdId == 1 ) {
	slaveCommand( COMMAND_REQ_PD1_CODE );
	readFromWire( 4 );
	intv = readIntFromWire( );

  } else if ( pdId == 2 ) {
	readFromWire( 4 );
	slaveCommand( COMMAND_REQ_PD2_CODE );
	intv = readIntFromWire( );

  } else {
	return 0;
  }

  return intv;
}

void light_check( byte chanId, bool force ) {

  unsigned long time_ms = millis();
  float damping = 0.8;

  int i = 0;
  // Last steady state was more than 10seconds ago or last update
  if ( time_ms - lights[ chanId ] -> time_checking > 10000 || force ) {

     lights[ chanId ]->time_checking = millis();

     if( time_ms - lights[ chanId ] -> time_processing > 1000 || force ) {

      lights[ chanId ] -> time_processing = millis();
      if ( ! light_isenabled( chanId ) ) { // Off. Nobody cares
        SerialUSB.println("");
        return;
      }

      if ( ! light_isautomatic( chanId ) ) { // Manual mode, nobody cares
    	    SerialUSB.println("");
        return;
      }

      float sunValue;// = measurePDSun( chanId );

//      slaveCommand( COMMAND_PAUSE, -1, (byte) 1 );


       do {

           sunValue = measurePDSun( chanId );
           SerialUSB.print( lights[ chanId ]->pwm );

          if ( fabs( sunValue - lights[ chanId ]->setpoint ) < 0.01 ) { // Light and setpoint have above 1% deviation
            break;
          }


          // We need to pause the tracking while the light is bein updated.


         if ( lights[ chanId ]->pwm == 1023 && sunValue < lights[ chanId ]->setpoint ) {
            light_setPWM( chanId, 1023 );
            break;
          }

          if ( lights[ chanId ]->pwm == 0 && sunValue > lights[ chanId ]->setpoint ) {
            light_setPWM( chanId, 0 );
            break;
          }

            //SerialUSB.println(lights[ chanId ]->pwm);
          if( sunValue > lights[ chanId ]->setpoint ) {
            lights[ chanId ]->pwm -= 1;
          } else {
            lights[ chanId ]->pwm += 1;
          }

          SerialUSB.print( lights[ chanId ]->pwm );
          SerialUSB.print( "," );

          if ( lights[ chanId ]->pwm > 1023 ) {
            lights[ chanId ]->pwm = 1023;
          }

          if ( lights[ chanId ]->pwm < 0 ) {
            lights[ chanId ]->pwm = 0;
          }


          light_setPWM( chanId, lights[ chanId ]->pwm );

          if( lights[ chanId ]->pwm == 1 || lights[ chanId ]->pwm == 1023 ) {
            break;
          }

          delay(10);
          i++;
       } while( true && i < 1023 );

  //      slaveCommand( COMMAND_PAUSE, -1, (byte) 0 );

     }
    }
    SerialUSB.println("");

  }




