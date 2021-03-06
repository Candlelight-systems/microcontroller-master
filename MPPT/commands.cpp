#include "scpiparser.h"
#include <Arduino.h>
#include "MPPT.h"
#include <Wire.h>
#include "commands.h"
#include "relays.h"
#include "slave.h"
#include "light.h"
#include "pid.h"
#include "ssr.h"
#include "light_expander.h"
#include "_parameters.c"

struct scpi_parser_context ctx;

scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens( args );
  SerialUSB.println( "" );
  return SCPI_SUCCESS;
}

scpi_error_t reset(struct scpi_parser_context* context, struct scpi_token* args) {
  slaveCommand( COMMAND_RESET );
  NVIC_SystemReset();
  return SCPI_SUCCESS;
}

scpi_error_t measure_voc(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  pause_wait();
  slaveCommand( COMMAND_VOC_TRIGGER, chanId );
  pause_resume();
  return SCPI_SUCCESS;
}

scpi_error_t measure_voc_status(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_VOC_DOING, chanId );
  SerialUSB.println( readByteFromWire() );
  return SCPI_SUCCESS;
}

scpi_error_t measure_voc_data(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_VOC, chanId );
  Wire.requestFrom( 8, 4 );
  printDouble( readFloatFromWire(), 7 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}

// Trigger
scpi_error_t measure_jsc(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  pause_wait();
  slaveCommand( COMMAND_JSC_TRIGGER, chanId );
  pause_resume();
  return SCPI_SUCCESS;
}

scpi_error_t measure_jsc_status(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_JSC_DOING, chanId );
  SerialUSB.println( readByteFromWire() );
  return SCPI_SUCCESS;
}

scpi_error_t measure_jsc_data(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  slaveCommand( COMMAND_REQ_JSC, chanId );
  Wire.requestFrom( 8, 4 );
  printDouble( readFloatFromWire(), 7 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}


scpi_error_t set_dac_voltage(struct scpi_parser_context* context, struct scpi_token* args) {
  struct scpi_numeric dac_voltage = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 4095 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_SET_VOLTAGE_CODE, chanId, (int32_t) dac_voltage.value );
  delay( 20 );
  return SCPI_SUCCESS;
}


scpi_error_t set_voltage(struct scpi_parser_context* context, struct scpi_token* args) {
  struct scpi_numeric dac_voltage = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -10, 10 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_SET_VOLTAGE, chanId, (float) dac_voltage.value );
  delay( 20 );
  return SCPI_SUCCESS;
}

scpi_error_t set_gain(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric gainValue = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 1, -1, 10 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  int gval = ( int ) gainValue.value;
  if ( gval == -1 ) {
    slaveCommand( COMMAND_AUTOGAIN, chanId, (byte) 1 );
  } else {
    slaveCommand( COMMAND_AUTOGAIN, chanId, (byte) 0 );
    slaveCommand( COMMAND_GAIN, chanId, (byte) gval );
  }
  return SCPI_SUCCESS;
}



scpi_error_t set_device_photodiode(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric pd = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 1, -1, 10 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  slaveCommand( COMMAND_SET_DEVICE_PHOTODIODE, chanId, (byte) pd.value );
  return SCPI_SUCCESS;
}

scpi_error_t enable_reference(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  slaveCommand( COMMAND_REFERENCE_ENABLE );
  return SCPI_SUCCESS;
}



scpi_error_t disable_reference(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  slaveCommand( COMMAND_REFERENCE_DISABLE );
  return SCPI_SUCCESS;
}


scpi_error_t reset_slave(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  reset_slave();
  return SCPI_SUCCESS;
}

scpi_error_t get_dac_voltage(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_VOLTAGE_DAC_CODE, chanId );
  Wire.requestFrom( 8, 4 ); // Max 64
  SerialUSB.println( readIntFromWire() );

  return SCPI_SUCCESS;
}


scpi_error_t measure_voltage(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  slaveCommand( COMMAND_REQ_VOLTAGE, chanId );
  Wire.requestFrom( 8, 4 );
  printDouble( readFloatFromWire(), 6 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}


scpi_error_t measure_current(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_CURRENT, chanId );
  Wire.requestFrom( 8, 4 );
  printDouble( readFloatFromWire(), 8 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}




scpi_error_t measure_voltage_code(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_VOLTAGE_CODE, chanId );
  Wire.requestFrom( 8, 4 ); // Max 64
  SerialUSB.println( readIntFromWire() );
  return SCPI_SUCCESS;
}


scpi_error_t measure_current_code(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_REQ_CURRENT_CODE, chanId );
  Wire.requestFrom( 8, 4 ); // Max 64
  SerialUSB.println( readIntFromWire() );
  return SCPI_SUCCESS;
}

scpi_error_t iv_execute(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_IV_TRIGGER, chanId );
  return SCPI_SUCCESS;
}

scpi_error_t iv_status(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  slaveCommand( COMMAND_REQ_IV_DOING, chanId );
  byte b = readByteFromWire();
  SerialUSB.println( b );
  return SCPI_SUCCESS;
}

scpi_error_t iv_data(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  slaveCommand( COMMAND_REQ_IV_LENGTH, chanId );
  readFromWire( 2 );
  uint16_t ivlength = readUInt16FromWire();

  SerialUSB.print( ivlength );
  SerialUSB.print(",");


  uint16_t iterator = 0;
  bool output = true;


  pause_wait();

  int j = 0;

  while( output ) {
	uint8_t r = 56;

	slaveCommand( 0x31, chanId );


    Wire.requestFrom( 8, r );

    for ( j = 0; j < 14; j ++ ) {

      if( iterator >= ivlength ) {

        output = false;
        break;
      }


      if( output ) {

          printDouble( readFloatFromWire(), 6 );
          SerialUSB.print(",");
       } else {
        readFloatFromWire();
       }

       if(j % 2 == 1 ) {
        iterator++;
       }
    }
  }

  slaveCommand( COMMAND_PAUSE, -1, (byte) 0 );


  SerialUSB.println("");

  return SCPI_SUCCESS;
}



scpi_error_t set_iv_autostart(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  bool autostart = scpi_parse_bool( args->next->next->next->value, args->next->next->next->length );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( (byte) COMMAND_IV_AUTOSTART, chanId, (unsigned char) autostart );
  return SCPI_SUCCESS;
}


scpi_error_t set_iv_start(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric start = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 1, -10, 10 );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_IV_START, chanId, (float) start.value );
  return SCPI_SUCCESS;
}

scpi_error_t set_iv_stop(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric stop = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -10, 10 );
  scpi_free_tokens( args );

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_IV_STOP, chanId, (float) stop.value );
  return SCPI_SUCCESS;
}

scpi_error_t set_iv_hysteresis(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  bool hysteresis = scpi_parse_bool( args->next->next->next->value, args->next->next->next->length );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_IV_HYSTERESIS, chanId, (unsigned char) hysteresis );
  return SCPI_SUCCESS;
}

scpi_error_t set_iv_rate(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric rate = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0.02, 0, 1 );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_IV_RATE, chanId, (float) rate.value );
  return SCPI_SUCCESS;
}


// ENABLE
scpi_error_t set_output_enable(struct scpi_parser_context* context, struct scpi_token* args) {


  byte chanId = getChannel( args );
  bool enable = scpi_parse_bool( args->next->next->next->value, args->next->next->next->length );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if ( enable ) {
    enableChannel( chanId );
    slaveCommand( COMMAND_ENABLE_CHANNEL, chanId );
  } else {
    disableChannel( chanId );
    slaveCommand( COMMAND_DISABLE_CHANNEL, chanId );
  }

  return SCPI_SUCCESS;
}


// External relay
scpi_error_t switch_external_relay(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  bool enable = scpi_parse_bool( args->next->next->next->value, args->next->next->next->length );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if ( enable ) {
    enableChannelExternal( chanId );
  } else {
    disableChannelExternal( chanId );
  }

  return SCPI_SUCCESS;
}



// External relay
scpi_error_t switch_general_relay(struct scpi_parser_context* context, struct scpi_token* args) {
  byte relayId = getChannel( args );
  bool enable = scpi_parse_bool( args->next->next->next->value, args->next->next->next->length );
  scpi_free_tokens(args);
  if ( relayId == -1 ) {
    return SCPI_SUCCESS;
  }

  if ( enable ) {
    enableChannelGeneral( relayId );
  } else {
    disableChannelGeneral( relayId );
  }

  return SCPI_SUCCESS;
}




// TRACKER
scpi_error_t set_tracking_mode(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric mode = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 4 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_MODE, chanId, (byte) mode.value );
  return SCPI_SUCCESS;
}


// TRACKER
scpi_error_t set_tracking_voltage(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric mode = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -10, 10 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_VOLTAGE, chanId, (float) mode.value );
  return SCPI_SUCCESS;
}


scpi_error_t set_tracking_interval(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric interval = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 1, 0, 10 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_INTERVAL, chanId, (int32_t) interval.value );
  return SCPI_SUCCESS;
}

scpi_error_t set_tracking_fwbw(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric fwbw = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0.0, 0.0, 1.0 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_FWBW, chanId, (float) fwbw.value );
  return SCPI_SUCCESS;
}

scpi_error_t set_tracking_bwfw(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric bwfw = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0.0, 0.0, 1.0 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_BWFW, chanId, (float) bwfw.value );
  return SCPI_SUCCESS;
}


scpi_error_t set_tracking_step(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric step = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 10 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_STEP, chanId, (uint8_t) step.value );
  return SCPI_SUCCESS;
}



scpi_error_t trackin_reset_channel(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_RESET, chanId );
  return SCPI_SUCCESS;
}




scpi_error_t set_tracking_switchdelay(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric switchdelay = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 1000 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_SWITCHDELAY, chanId, (int32_t) switchdelay.value );
  return SCPI_SUCCESS;
}



scpi_error_t set_tracking_speed(struct scpi_parser_context* context, struct scpi_token* args) {
  struct scpi_numeric speed = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 1000 );
  scpi_free_tokens(args);
  slaveCommand( COMMAND_TRACKER_SPEED, -1, (int32_t) speed.value );
  return SCPI_SUCCESS;
}


scpi_error_t data_tracker(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  getTrackData( chanId );
  return SCPI_SUCCESS;
}

scpi_error_t enable_debug(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  debug = true;
  return SCPI_SUCCESS;
}


scpi_error_t disable_debug(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  debug = false;
  return SCPI_SUCCESS;
}

scpi_error_t pause_hardware(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  slaveCommand( COMMAND_PAUSE, -1, (byte) 1 );
  return SCPI_SUCCESS;
}

scpi_error_t resume_hardware(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  slaveCommand( COMMAND_PAUSE, -1, (byte) 0 );
  return SCPI_SUCCESS;
}


scpi_error_t setupSlave(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  slaveCommand( COMMAND_SETUP );
  return SCPI_SUCCESS;
}


scpi_error_t configured(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens(args);
  statusByte |= 0b00000001;

  return SCPI_SUCCESS;
}


scpi_error_t autoZero(struct scpi_parser_context* context, struct scpi_token* args) {
	//struct scpi_numeric value;
	  int chanId = getChannel( args );
	  scpi_free_tokens(args);

	  if ( chanId == -1 ) {
		return SCPI_SUCCESS;
	  }

	  uint8_t chanEnabled = channelEnabled( chanId );

	  if( chanEnabled ) {
		  disableChannel( chanId );
	  }

	  pause_wait();	// Force pausing the whole system
	  slaveCommand( COMMAND_AUTOZERO, chanId ); // Autozero
	  delay( 50 );
	  slaveCommand( COMMAND_PAUSE, -1, (byte) 0 ); // Stops the pause

	  if( chanEnabled ) {
		 enableChannel( chanId );
	  }

//	slaveCommand( COMMAND_AUTOZERO );
	//delay( 100 );
	  return SCPI_SUCCESS;
}

scpi_error_t light_enable(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  lightEnable( chanId );
  return SCPI_SUCCESS;
}

scpi_error_t light_disable(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  lightDisable( chanId );

  return SCPI_SUCCESS;
}




scpi_error_t get_humidity_box(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric I2C_SLAVE = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 127 );


  scpi_free_tokens(args);
  muxI2C( (int) ( args->next->next->value[ 5 ] - '0' ) );

  aquireHumiditySensor( I2C_SLAVE.value );

  return SCPI_SUCCESS;
}
/*

// ENVIRONMENT:UVINTENSITY? 1
//       0         1        2
scpi_error_t get_uv_intenity(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric slaveNumber = scpi_parse_numeric( args->next->next->value, args->next->next->length, 0, 0, 127 );
  byte channel = getChannel( args );

  scpi_free_tokens(args);
  muxI2C( (int) slaveNumber.value );
  float val = slave_readVEML6070( );

  printDouble( val, 4 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}
*/

// ENVIRONMENT:TEMPBASE?:CH1:SLAVE1 72
//       0         1      2    3   4
scpi_error_t get_temperature_base(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric I2C_SLAVE = scpi_parse_numeric( args->next->next->next->next->value, args->next->next->next->next->length, 0, 0, 127 );
  byte channel = getChannel( args );

  scpi_free_tokens(args);
  muxI2C( (int) ( args->next->next->next->value[ 5 ] - '0' ) );
  int16_t val = slave_readADS1015( (byte) I2C_SLAVE.value, (byte) channel );

  SerialUSB.write( val >> 8 );
  SerialUSB.write( val );
  SerialUSB.println("");



  return SCPI_SUCCESS;
}

scpi_error_t get_temperature_ir(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric I2C_SLAVE = scpi_parse_numeric( args->next->next->next->next->value, args->next->next->next->next->length, 0, 0, 127 );
  byte channel = getChannel( args );
  scpi_free_tokens(args);
  muxI2C( (int) ( args->next->next->next->value[ 5 ] - '0' ) );
  int16_t val = slave_readADS1015( (byte) I2C_SLAVE.value, (byte) channel );

  //SerialUSB.println( (int) ( args->next->next->next->value[ 5 ] - '0' ) );
  //SerialUSB.println( (byte) I2C_SLAVE.value );
  SerialUSB.write( val >> 8 );
  SerialUSB.write( val );
  SerialUSB.println("");

  demuxI2C( WireSlave );

  return SCPI_SUCCESS;
}


scpi_error_t measure_pd(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  printDouble( measurePD( chanId ), 5 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}



scpi_error_t measure_pd_sun(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  printDouble( measurePDSun( chanId ), 5 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}



scpi_error_t measure_pd_code(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  SerialUSB.println( measurePDCode( chanId ) );
  return SCPI_SUCCESS;
}


scpi_error_t ssr_mode_heating(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 ) {
	  //enableChannelGeneral( 1 ); // Triggered externally
	  PID_SSR1.mode_heating = true;

  } else if( chanId == 2 ) {
	//  enableChannelGeneral( 2 ); // Triggered externally
	  PID_SSR2.mode_heating = true;
  }

  return SCPI_SUCCESS;
}



scpi_error_t ssr_mode_cooling(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 ) {
	  //enableChannelGeneral( 1 ); // Triggered externally
	  PID_SSR1.mode_heating = false;

  } else if( chanId == 2 ) {
	//  enableChannelGeneral( 2 ); // Triggered externally
	  PID_SSR2.mode_heating = false;
  }
  return SCPI_SUCCESS;
}


scpi_error_t ssr_disable(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 || chanId == 2 ) {
	//  disableChannelGeneral( 1 ); // Triggered externally
	  SSRDisable( chanId );
  }

  return SCPI_SUCCESS;
}



scpi_error_t ssr_enable(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 || chanId == 2 ) {
	//  disableChannelGeneral( 1 ); // Triggered externally
	  SSREnable( chanId );
  }

  return SCPI_SUCCESS;
}

scpi_error_t ssr_value(struct scpi_parser_context* context, struct scpi_token* args) {

 struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 || chanId == 2 ) {
	//  disableChannelGeneral( 1 ); // Triggered externally
	  SSRSetValue( chanId, (float) val.value );
  }

  return SCPI_SUCCESS;
}


scpi_error_t ssr_pid_param_kp_h(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.Kp_h = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.Kp_h = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_kp_c(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.Kp_c = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.Kp_c = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_kd_h(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.Kd_h = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.Kd_h = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_kd_c(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.Kd_c = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.Kd_c = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_ki_h(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.Ki_h = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.Ki_h = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_ki_c(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.Ki_c = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.Ki_c = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_bi_h(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.bias_h = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.bias_h = (float) val.value;
}

  return SCPI_SUCCESS;
}

scpi_error_t ssr_pid_param_bi_c(struct scpi_parser_context* context, struct scpi_token* args) {

struct scpi_numeric val = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
byte chanId = getChannel( args );
scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
if( chanId == 1) {
	PID_SSR1.bias_c = (float) val.value;
} else if( chanId == 2 ) {
	PID_SSR2.bias_c = (float) val.value;
}

  return SCPI_SUCCESS;
}




scpi_error_t ssr_set_target(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric target = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 85 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 ) {
	  PID_SSR1.target = (float) target.value;
  } else if( chanId == 2 ) {
	  PID_SSR2.target = (float) target.value;
  }

  return SCPI_SUCCESS;
}


scpi_error_t ssr_set_feedback(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric value = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 85 );
  byte chanId = getChannel( args );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 ) {
	  SSRSetValue( chanId, PID_SSR1.next( (float) value.value ) );
  } else if( chanId == 2 ) {
	  SSRSetValue( chanId, PID_SSR2.next( (float) value.value ) );
  }

  return SCPI_SUCCESS;
}

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


scpi_error_t light_set_setpoint(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  struct scpi_numeric setpoint = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 2 );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  lightSetSetpoint( chanId, (float) setpoint.value );
  return SCPI_SUCCESS;
}

scpi_error_t light_set_scaling(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  struct scpi_numeric scaling = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 100 );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  lightSetScaling( chanId, ( float ) scaling.value );

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



scpi_error_t read_4_20_mA_sensor(struct scpi_parser_context* context, struct scpi_token* args) {

  byte address = 0b1001000;
  byte chanId = getChannel( args );
  byte b1, b2;

  scpi_free_tokens(args);

  Wire.beginTransmission( 0b1110000	 );
  if( chanId == 1 ) {
    Wire.write( 0b00000001 );
  } else {
    Wire.write( 0b00000010 );
  }


  Wire.endTransmission();

  Wire.beginTransmission( address );
  Wire.write( 0x00 | 0b00000001 ); // Write to the configuration register
  Wire.write( 0b00000100 ); // PGA at 2.048V, single shot, at a specific channel. Executes a conversion
  Wire.write( 0b10000111 ); // 1.6kSPS, disable comparator
  Wire.endTransmission();
  delay( 2 ); // Wait for the conversion to complete
  Wire.beginTransmission( address );
  Wire.write( 0x00 | 0b00000000 ); // Points to the data register
  Wire.endTransmission();
delayMicroseconds( 50 );
  if( Wire.requestFrom( address, 2 ) == 0 ) {
	  return SCPI_SUCCESS;
  }

  b1 = Wire.read();
  b2 = Wire.read();
  int16_t val = ( ( b1 << 8 | b2 ) / 32 );

  printDouble( ( float ) ( val * CALIBRATION_4_20MA_GAIN ) + CALIBRATION_4_20MA_OFFSET, 5 );
  SerialUSB.println("");

  Wire.beginTransmission( 0b1110000 );
  if( chanId == 1 ) {
    Wire.write( 0b00000000 );
  } else {
    Wire.write( 0b00000000 );
  }


  Wire.endTransmission();


  //light_check( 2 );

  return SCPI_SUCCESS;
}


void registerSCPICommands() {

	scpi_init(&ctx);


  struct scpi_command* source;
  struct scpi_command* measure;
  struct scpi_command* iv;
  struct scpi_command* tracker;
  struct scpi_command* output;
  struct scpi_command* reserved;
  struct scpi_command* environment;
  struct scpi_command* light;

  struct scpi_command* ssr;
  struct scpi_command* slave;
  struct scpi_command* expander;

  struct scpi_command* data;
  // Setting up SCPI commands
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*RST", 4, "*RST", 4, reset);
  source = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "SOURCE", 6, "SOUR", 4, NULL);
  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE", 7, "VOLT", 4, set_voltage);
  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE?", 7, "VOLT?", 4, get_dac_voltage);

  measure = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "MEASURE", 7, "MEAS", 4, NULL);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE", 7, "VOLT", 4, measure_voltage);
  scpi_register_command(measure, SCPI_CL_CHILD, "CURRENT", 7, "CURR", 4, measure_current);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOC", 3, "VOC", 3, measure_voc);
  scpi_register_command(measure, SCPI_CL_CHILD, "JSC", 3, "JSC", 3, measure_jsc);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOCSTATUS", 9, "VOCS", 4, measure_voc_status);
  scpi_register_command(measure, SCPI_CL_CHILD, "JSCSTATUS", 9, "JSCS", 4, measure_jsc_status);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOCDATA", 7, "VOCD", 4, measure_voc_data);
  scpi_register_command(measure, SCPI_CL_CHILD, "JSCDATA", 7, "JSCD", 4, measure_jsc_data);


  data = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "DATA", 4, "DATA", 4, NULL);
  scpi_register_command(data, SCPI_CL_CHILD, "TRACKER", 7, "TRAC", 4, data_tracker);

  iv = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "IV", 2, "IV", 2, NULL);
  scpi_register_command(iv, SCPI_CL_CHILD, "START", 5, "STAR", 4, set_iv_start);
  scpi_register_command(iv, SCPI_CL_CHILD, "STOP", 4, "STOP", 4, set_iv_stop);
  scpi_register_command(iv, SCPI_CL_CHILD, "AUTOSTART", 9, "AUTO", 4, set_iv_autostart);
  scpi_register_command(iv, SCPI_CL_CHILD, "HYSTERESIS", 10, "HYST", 4, set_iv_hysteresis);
  scpi_register_command(iv, SCPI_CL_CHILD, "RATE", 4, "RATE", 4, set_iv_rate);
  scpi_register_command(iv, SCPI_CL_CHILD, "EXECUTE", 7, "EXEC", 4, iv_execute);
  scpi_register_command(iv, SCPI_CL_CHILD, "STATUS", 6, "STAT", 4, iv_status);
  scpi_register_command(iv, SCPI_CL_CHILD, "DATA", 4, "DATA", 4, iv_data);

  output = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "RELAY", 5, "RELA", 4, NULL);
  scpi_register_command(output, SCPI_CL_CHILD, "CHANNEL", 7, "CHAN", 4, set_output_enable);
  scpi_register_command(output, SCPI_CL_CHILD, "EXTERNAL", 8, "EXTE", 4, switch_external_relay);
  scpi_register_command(output, SCPI_CL_CHILD, "GENERAL", 7, "GENE", 4, switch_general_relay);

  ssr = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "SSR", 3, "SSR", 3, NULL);
  scpi_register_command(ssr, SCPI_CL_CHILD, "TARGET", 6, "TARG", 4, ssr_set_target);
  scpi_register_command(ssr, SCPI_CL_CHILD, "FEEDBACK", 8, "FEED", 4, ssr_set_feedback);
  scpi_register_command(ssr, SCPI_CL_CHILD, "HEATING", 7, "HEAT", 4, ssr_mode_heating);
  scpi_register_command(ssr, SCPI_CL_CHILD, "COOLING", 7, "COOL", 4, ssr_mode_cooling);
  scpi_register_command(ssr, SCPI_CL_CHILD, "DISABLE", 7, "DISA", 4, ssr_disable);
  scpi_register_command(ssr, SCPI_CL_CHILD, "ENABLE", 6, "ENAB", 4, ssr_enable);
  scpi_register_command(ssr, SCPI_CL_CHILD, "VALUE", 5, "VALU", 4, ssr_value);

  scpi_register_command(ssr, SCPI_CL_CHILD, "KP_HEATING", 10, "KP_H", 4, ssr_pid_param_kp_h);
  scpi_register_command(ssr, SCPI_CL_CHILD, "KP_COOLING", 10, "KP_C", 4, ssr_pid_param_kp_c);
  scpi_register_command(ssr, SCPI_CL_CHILD, "KD_HEATING", 10, "KD_H", 4, ssr_pid_param_kd_h);
  scpi_register_command(ssr, SCPI_CL_CHILD, "KD_COOLING", 10, "KD_C", 4, ssr_pid_param_kd_c);
  scpi_register_command(ssr, SCPI_CL_CHILD, "KI_HEATING", 10, "KI_H", 4, ssr_pid_param_ki_h);
  scpi_register_command(ssr, SCPI_CL_CHILD, "KI_COOLING", 10, "KI_C", 4, ssr_pid_param_ki_c);

  scpi_register_command(ssr, SCPI_CL_CHILD, "BIAS_HEATING", 12, "BI_H", 4, ssr_pid_param_bi_h);
  scpi_register_command(ssr, SCPI_CL_CHILD, "BIAS_COOLING", 12, "BI_C", 4, ssr_pid_param_bi_c);

  tracker = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "TRACKING", 8, "TRAC", 4, NULL);
  scpi_register_command(tracker, SCPI_CL_CHILD, "MODE", 4, "MODE", 4, set_tracking_mode);
  scpi_register_command(tracker, SCPI_CL_CHILD, "VOLTAGE", 7, "VOLT", 4, set_tracking_voltage );
  scpi_register_command(tracker, SCPI_CL_CHILD, "INTERVAL", 8, "INTE", 4, set_tracking_interval);
  scpi_register_command(tracker, SCPI_CL_CHILD, "FWBWTHRESHOLD", 13, "FWBW", 4, set_tracking_fwbw);
  scpi_register_command(tracker, SCPI_CL_CHILD, "BWFWTHRESHOLD", 13, "BWFW", 4, set_tracking_bwfw);
  scpi_register_command(tracker, SCPI_CL_CHILD, "STEP", 4, "STEP", 4, set_tracking_step);
  scpi_register_command(tracker, SCPI_CL_CHILD, "SWITCHDELAY", 11, "SWIT", 4, set_tracking_switchdelay);
  scpi_register_command(tracker, SCPI_CL_CHILD, "GAIN", 4, "GAIN", 4, set_gain);
  scpi_register_command(tracker, SCPI_CL_CHILD, "PHOTODIODE", 10, "PD", 2, set_device_photodiode);
  scpi_register_command(tracker, SCPI_CL_CHILD, "RESET", 5, "RESE", 4, trackin_reset_channel);

  scpi_register_command(tracker, SCPI_CL_CHILD, "SPEED", 5, "SPEE", 4, set_tracking_speed);

  environment = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "ENVIRONMENT", 11, "ENVI", 4, NULL);

  //scpi_register_command(environment, SCPI_CL_CHILD, "TEMPBOX?", 8, "TBOX?", 5, get_temperature_box);
 // scpi_register_command(environment, SCPI_CL_CHILD, "UVINTENSITY?", 12, "UV?", 3, get_uv_intensity);
  scpi_register_command(environment, SCPI_CL_CHILD, "TEMPBASE?", 9, "TBASE?", 6, get_temperature_base);
  scpi_register_command(environment, SCPI_CL_CHILD, "TEMPIR?", 7, "TIR?", 4, get_temperature_ir);
  scpi_register_command(environment, SCPI_CL_CHILD, "HUMIDITY?", 9, "HUMI?", 5, get_humidity_box);
  scpi_register_command(environment, SCPI_CL_CHILD, "PHOTODIODE", 10, "PD", 2, measure_pd);
  scpi_register_command(environment, SCPI_CL_CHILD, "SUNPHOTODIODE", 13, "SPD", 3, measure_pd_sun);
  scpi_register_command(environment, SCPI_CL_CHILD, "CODEPHOTODIODE", 14, "CPD", 3, measure_pd_code);
  //  scpi_register_command(environment, SCPI_CL_CHILD, "LEDON", 5, "LEDON", 5, led_on);
  //  scpi_register_command(environment, SCPI_CL_CHILD, "LEDOFF", 6, "LEDOFF", 6, led_off);


  slave = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "I2C", 3, "I2C", 3, NULL);

  //scpi_register_command(environment, SCPI_CL_CHILD, "TEMPBOX?", 8, "TBOX?", 5, get_temperature_box);
  scpi_register_command(slave, SCPI_CL_CHILD, "4_20MA", 6, "4_20MA", 6, read_4_20_mA_sensor);


  // I2C:LIGHTEXPANDER:SETPWM:CH1 0.3
#if LIGHT_EXPANDER

  expander = scpi_register_command(slave, SCPI_CL_CHILD, "LIGHTEXPANDER", 13, "LIGH", 4, NULL);

  scpi_register_command(expander, SCPI_CL_CHILD, "SETPWM", 6, "SETP", 6, light_expander_setPWM);
  scpi_register_command(expander, SCPI_CL_CHILD, "ENABLE", 6, "ENAB", 6, light_expander_enable);
  scpi_register_command(expander, SCPI_CL_CHILD, "DISABLE", 7, "DISA", 6, light_expander_disable);
  scpi_register_command(expander, SCPI_CL_CHILD, "ISENABLED", 9, "ISEN", 4, light_expander_isEnabled);

#endif



  //  scpi_register_command(environment, SCPI_CL_CHILD, "LEDON", 5, "LEDON", 5, led_on);
  //  scpi_register_command(environment, SCPI_CL_CHILD, "LEDOFF", 6, "LEDOFF", 6, led_off);


  reserved = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "RESERVED", 8, "RESE", 4, NULL);
  scpi_register_command(reserved, SCPI_CL_CHILD, "DEBUG", 5, "DEBU", 4, enable_debug);
  scpi_register_command(reserved, SCPI_CL_CHILD, "PRODUCTION", 10, "PROD", 4, disable_debug);
  scpi_register_command(reserved, SCPI_CL_CHILD, "ADCVOLTAGE", 10, "ADCV", 4, measure_voltage_code);
  scpi_register_command(reserved, SCPI_CL_CHILD, "ADCCURRENT", 10, "ADCI", 4, measure_current_code);
  scpi_register_command(reserved, SCPI_CL_CHILD, "DACVOLTAGE", 10, "DACV", 4, set_dac_voltage);
  scpi_register_command(reserved, SCPI_CL_CHILD, "REFENABLE", 9, "REFE", 4, enable_reference);
  scpi_register_command(reserved, SCPI_CL_CHILD, "REFDISABLE", 10, "REFD", 4, disable_reference);
  scpi_register_command(reserved, SCPI_CL_CHILD, "RESETSLAVE", 10, "RSTSLV", 6, reset_slave);
  scpi_register_command(reserved, SCPI_CL_CHILD, "PAUSE", 5, "PAUS", 4, pause_hardware);
  scpi_register_command(reserved, SCPI_CL_CHILD, "RESUME", 6, "RESU", 4, resume_hardware);
  scpi_register_command(reserved, SCPI_CL_CHILD, "SETUP", 5, "SETU", 4, setupSlave);
  scpi_register_command(reserved, SCPI_CL_CHILD, "CONFIGURED", 10, "CONF", 4, configured);
  scpi_register_command(reserved, SCPI_CL_CHILD, "AUTOZERO", 8, "ZERO", 4, autoZero);




  light = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "LIGHT", 5, "LIGH", 4, NULL);
  scpi_register_command(light, SCPI_CL_CHILD, "ENABLE", 6, "ENAB", 4, light_enable);
  scpi_register_command(light, SCPI_CL_CHILD, "DISABLE", 7, "DISA", 4, light_disable);
  scpi_register_command(light, SCPI_CL_CHILD, "ENABLED?", 8, "ENA?", 4, light_isenabled);
  scpi_register_command(light, SCPI_CL_CHILD, "AUTOMATIC?", 10, "AUTO?", 5, light_isautomatic);
  scpi_register_command(light, SCPI_CL_CHILD, "SETPOINT", 8, "SETP", 4, light_set_setpoint);
  scpi_register_command(light, SCPI_CL_CHILD, "SCALING  ", 7, "SCAL", 4, light_set_scaling);
  scpi_register_command(light, SCPI_CL_CHILD, "CHECK", 5, "CHEC", 4, light_check );
  scpi_register_command(light, SCPI_CL_CHILD, "FORCECHECK", 10, "FORCE", 5, light_force_check );
  scpi_register_command(light, SCPI_CL_CHILD, "PWM", 3, "PWM", 4, light_setPWM );


}
