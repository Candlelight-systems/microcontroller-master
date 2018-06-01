
struct light {
  unsigned long time_checking = 0;
  unsigned long time_processing = 0;

  float pd_scaling_ma_to_sun = -6.93;
  float setpoint = 1;
  uint32_t pwm = 255;
};


struct light* lights[ 3 ] = {
  NULL,
  new light,
  new light
};




scpi_error_t light_isautomatic(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  SerialUSB.println( light_isautomatic( chanId ) );
  return SCPI_SUCCESS;
}


scpi_error_t light_isenabled(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  SerialUSB.println( light_isenabled( chanId ) );
  return SCPI_SUCCESS;
}

bool light_isenabled( byte chanId ) {
  if ( chanId == 1 ) {
    return digitalRead( PIN_LIGHT_1_ONOFF_READ );
  } else if ( chanId == 2 ) {
    return digitalRead( PIN_LIGHT_2_ONOFF_READ );
  }
}


bool light_isautomatic( byte chanId ) {
  if ( chanId == 1 ) {
    return digitalRead( PIN_LIGHT1_MODE_DEC );
  } else if ( chanId == 2 ) {
    return digitalRead( PIN_LIGHT2_MODE_DEC );
  }
}

void light_setPWM( byte chanId, uint32_t pwm ) {
  if ( chanId == 1 ) {
    analogWrite( PIN_LIGHT_PWM_1, pwm );
  } else if ( chanId == 2 ) {
    analogWrite( PIN_LIGHT_PWM_2, pwm );
  }
}

scpi_error_t light_set_setpoint(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  struct scpi_numeric setpoint = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 2 );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if ( chanId == 1 ) {
    lights[ 1 ] -> setpoint = ( float ) setpoint.value;
  } else if ( chanId == 2 ) {
    lights[ 2 ] -> setpoint = ( float ) setpoint.value;
  }

  return SCPI_SUCCESS;
}



scpi_error_t light_set_scaling(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  struct scpi_numeric scaling = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 100 );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if ( chanId == 1 ) {
    lights[ 1 ] -> pd_scaling_ma_to_sun = ( float ) scaling.value;
  } else if ( chanId == 2 ) {
    lights[ 2 ] -> pd_scaling_ma_to_sun = ( float ) scaling.value;
  }

  return SCPI_SUCCESS;
}



scpi_error_t light_check(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  light_check( chanId, false );
  //light_check( 2 );

  return SCPI_SUCCESS;
}


scpi_error_t light_force_check(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  light_check( chanId, true );
  //light_check( 2 );

  return SCPI_SUCCESS;
}


scpi_error_t light_setPWM(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  struct scpi_numeric value = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 255 );
  scpi_free_tokens(args);
  light_setPWM( chanId, value.value );
  //light_check( 2 );

  return SCPI_SUCCESS;
}




double measurePDSun( byte pdId ) {
  float f;
  if ( pdId == 1 ) {
    slaveCommand( COMMAND_REQ_PD1 );
    SerialUSB.println('a');
    delay( 1 );
    readFromWire( 4 );
    SerialUSB.println('b');
    delay( 1 );
    f = readFloatFromWire();
    SerialUSB.println('c');
    delay( 1 );
  } else if ( pdId == 2 ) {
    slaveCommand( COMMAND_REQ_PD2 );
    readFromWire( 4 );
    f = readFloatFromWire();
  } else {
    return -1.0;
  }

  return f * lights[ pdId ]->pd_scaling_ma_to_sun;
}


double measurePD( byte pdId ) {
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
  
      slaveCommand( COMMAND_PAUSE, -1, (byte) 1 );

      
       do {
        
           sunValue = measurePDSun( chanId );
           
        
          if ( fabs( sunValue - lights[ chanId ]->setpoint ) < 0.01 ) { // Light and setpoint have above 1% deviation
            break;
          }
  
          
          // We need to pause the tracking while the light is bein updated.
          

         if ( lights[ chanId ]->pwm == 255 && sunValue > lights[ chanId ]->setpoint ) {
            light_setPWM( chanId, 255 );
            break;
          }
    
          if ( lights[ chanId ]->pwm == 0 && sunValue < lights[ chanId ]->setpoint ) {
            light_setPWM( chanId, 0 );
            break;
          }

  
          if( sunValue < lights[ chanId ]->setpoint ) {
            lights[ chanId ]->pwm -= 1;
          } else {
            lights[ chanId ]->pwm += 1;
          }
    
          if ( lights[ chanId ]->pwm > 255 ) {
            lights[ chanId ]->pwm = 255;
          }
    
          if ( lights[ chanId ]->pwm < 0 ) {
            lights[ chanId ]->pwm = 0;
          }
  
      
          light_setPWM( chanId, lights[ chanId ]->pwm );
      
          if( lights[ chanId ]->pwm == 2 || lights[ chanId ]->pwm == 255 ) {
            break;
          }

          delay(10);
          i++;
       } while( true && i < 255 );

        slaveCommand( COMMAND_PAUSE, -1, (byte) 0 );
  
     }
     
    }

    SerialUSB.println("");

  }





