#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <scpiparser.h>
#include <wiring_private.h>
#include "MataHari.h"
#include <float.h>


// We removed the 0x00 command. The byte 0x00 means no command received from now on.RE
#define COMMAND_TRACKER_MODE 0x43
#define COMMAND_TRACKER_INTERVAL 0x01
#define COMMAND_TRACKER_STEP 0x02
#define COMMAND_TRACKER_FWBW 0x03
#define COMMAND_TRACKER_BWFW 0x04
#define COMMAND_TRACKER_SWITCHDELAY 0x05
#define COMMAND_TRACKER_RESET 0x18
#define COMMAND_IV_START 0x06
#define COMMAND_IV_STOP 0x07
#define COMMAND_IV_RATE 0x08
#define COMMAND_IV_HYSTERESIS 0x09
#define COMMAND_IV_AUTOSTART 0x0A
#define COMMAND_IV_TRIGGER 0x0B
#define COMMAND_AUTOGAIN 0x0C
#define COMMAND_GAIN 0x0D
#define COMMAND_PAUSE 0x0E
#define COMMAND_VOC_TRIGGER 0x0F
#define COMMAND_JSC_TRIGGER 0x10
#define COMMAND_SET_VOLTAGE 0x11
#define COMMAND_REFERENCE_ENABLE 0x12
#define COMMAND_REFERENCE_DISABLE 0x13
#define COMMAND_SETUP 0x14
#define COMMAND_SET_VOLTAGE_CODE 0x15
#define COMMAND_TRACKER_SPEED 0x16
#define COMMAND_SET_DEVICE_PHOTODIODE 0x17
#define COMMAND_RESET 0x18
#define COMMAND_ENABLE_CHANNEL 0x41
#define COMMAND_DISABLE_CHANNEL 0x42

#define COMMAND_REQ_TRACKDATA 0x20
#define COMMAND_REQ_VOC 0x21
#define COMMAND_REQ_JSC 0x22
#define COMMAND_REQ_IV_DOING 0x23
#define COMMAND_REQ_VOC_DOING 0x25
#define COMMAND_REQ_JSC_DOING 0x26
#define COMMAND_REQ_CURRENT 0x27
#define COMMAND_REQ_VOLTAGE 0x28
#define COMMAND_REQ_CURRENT_CODE 0x29
#define COMMAND_REQ_VOLTAGE_CODE 0x2A
#define COMMAND_REQ_VOLTAGE_DAC_CODE 0x2B
#define COMMAND_REQ_PD2 0x2C
#define COMMAND_REQ_PD1 0x2D
#define COMMAND_REQ_PD2_CURRENT 0x2E
#define COMMAND_REQ_PD1_CURRENT 0x2F
#define COMMAND_REQ_IV_LENGTH 0x30

bool swdec1State = false;
bool swdec2State = false;
struct scpi_parser_context ctx;
bool debug = false;
byte statusByte = 0x00;;


// Create a 2-Wire instance for the slave
TwoWire WireSlave( &sercom1, 11, 13 );


void loop() {
  char line_buffer[256];
  int read_length;
  int i;

  while (1)
  {
    /* Read in a line and execute it. */
    // Before treating any channel, check for serial communication
    if ( SerialUSB.available() ) {

      read_length = SerialUSB.readBytesUntil('\n', line_buffer, 256);

      if (read_length > 0)
      {

        
 //slaveCommand( COMMAND_LOCK );
  
        scpi_execute_command(&ctx, line_buffer, read_length);
        read_length = 0;
        SerialUSB.println( statusByte ); // Always send a statusbyte at the end of the command
        //slaveCommand( COMMAND_UNLOCK );
      }
    }
  }
}


void slaveCommand( byte commandByte ) {
  Wire.beginTransmission( 8 );
  Wire.write( commandByte );
  Wire.endTransmission();
}


void slaveCommand( byte commandByte, byte chanId ) {
  Wire.beginTransmission( 8 );
  Wire.write( commandByte );
  Wire.write( chanId );
  Wire.endTransmission();
}



union {
  float f;
  unsigned char bytes[4];
} u;

void slaveCommand( byte commandByte, byte channelId, float value ) {

  // Overite bytes of union with float variable
  float f = value;
  u.f = f;

  Wire.beginTransmission( 8 );
  Wire.write( commandByte );
  Wire.write( channelId );

  int ii;
  for (ii=3; ii>=0; ii--) {
//    SerialUSB.println( u.bytes[ ii ], BIN );
    Wire.write( u.bytes[ ii ] );
  }

  Wire.endTransmission();
}

void slaveCommand( byte commandByte, byte chanId, int32_t value ) {
  Wire.beginTransmission( 8 );
  Wire.write( commandByte );
  Wire.write( chanId );
  Wire.write( value >> 24 );
  Wire.write( value >> 16 );
  Wire.write( value >> 8 );
  Wire.write( value >> 0 );

  
  Wire.endTransmission();
}


void slaveCommand( byte commandByte, byte chanId, byte value ) {
  Wire.beginTransmission( 8 );
  Wire.write( commandByte );
  Wire.write( chanId );
  Wire.write( value );
  Wire.endTransmission();
}

void readFromWire( int howMany ) {
  Wire.requestFrom( 8, howMany ); // Max 64
}

byte readByteFromWire( ) {
  Wire.requestFrom( 8, 1 ); // Max 64
  return Wire.read();
}

int32_t readIntFromWire( ) {
  
  byte b1 = Wire.read();
  byte b2 = Wire.read();
  byte b3 = Wire.read();
  byte b4 = Wire.read();
  return (int32_t) ( ( ( int32_t ) b1  << 24 & 0xFF000000 ) | ( ( int32_t ) b2  << 16 & 0x00FF0000 )  | ( ( int32_t ) b3 << 8 & 0x0000FF00 )  | ( ( int32_t ) b4 << 0 & 0x000000FF ) );
}



int32_t readUInt16FromWire( ) {
  
  byte b1 = Wire.read();
  byte b2 = Wire.read();
  return (uint16_t) ( ( ( uint16_t ) b1 << 8 & 0xFF00 )  | ( ( uint16_t ) b2 << 0 & 0x00FF ) );
}


float readFloatFromWire() {

  byte b1 = Wire.read();
  byte b2 = Wire.read();
  byte b3 = Wire.read();
  byte b4 = Wire.read();

  u.bytes[ 0 ] = b1;
  u.bytes[ 1 ] = b2;
  u.bytes[ 2 ] = b3;
  u.bytes[ 3 ] = b4;
   return u.f;
}

void getTrackData( int channel ) {


  char comma = ',';

  float f1, f2, f3, f4, f5, f6, f7, f8, f9;
  uint32_t i1, i2;
  
  // Tell the slave we wish the retrieve the tracking data
  slaveCommand( COMMAND_REQ_TRACKDATA, channel );
  // Request the proper number of bytes

  readFromWire( 44 ); // 44 bytes in total (11 x 4 bytes / float32)


  
  f1 = readFloatFromWire();
  f2 = readFloatFromWire();
  f3 = readFloatFromWire();
  f4 = readFloatFromWire();
  f5 = readFloatFromWire();
  f6 = readFloatFromWire();
  f7 = readFloatFromWire();
  f8 = readFloatFromWire();
  f9 = readFloatFromWire();

  i1 = readIntFromWire();
  i2 = readIntFromWire();
  
  // Request and print
  printDouble2( f1 );
  printDouble2( f2 );
  printDouble2( f3 );
  printDouble2( f4 );
  printDouble2( f5 );
  printDouble2( f6 );
  printDouble2( f7 );
  printDouble2( f8 );
  printDouble2( f9 );
  
  SerialUSB.write( i1 );
  SerialUSB.write( i2 );
  SerialUSB.println("");
}

void printDouble2( float val ) {
  
  union {
    float f;
    unsigned char v[ 4 ];
  } conversion;  

  conversion.f = val;
  SerialUSB.write( conversion.v[ 0 ] );
  SerialUSB.write( conversion.v[ 1 ] );
  SerialUSB.write( conversion.v[ 2 ] );
  SerialUSB.write( conversion.v[ 3 ] );
}

byte getChannel( struct scpi_token* args ) {

  int a = (int) ( args->next->next->value[2] - '0' );
  if ( args->next->next->length == 4 ) {
    a = a * 10 + (int) ( args->next->next->value[3] - '0' );
  }
  if ( a < 0 || a > 16 ) {
    scpi_error error;
    error.id = 101;
    error.description = "Command error;Invalid channel";
    error.length = 29;
    scpi_queue_error(&ctx, error);
    return -1;
  }

  return a;
}

scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* args) {
  scpi_free_tokens( args );
  SerialUSB.println( "" );
  return SCPI_SUCCESS;
}

scpi_error_t measure_voc(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_VOC_TRIGGER, chanId );
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
  slaveCommand( COMMAND_JSC_TRIGGER, chanId );
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

void reset_slave() {
  digitalWrite( PIN_RESET_SLAVE, HIGH );
  delay( 1000 );
  digitalWrite( PIN_RESET_SLAVE, LOW );
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

  slaveCommand( COMMAND_PAUSE, -1, (byte) 1 );
   
  int j = 0;
  while( output ) {

    slaveCommand( 0x31, chanId );

    Wire.requestFrom( 8, 60 );
    for ( j = 0; j < 15; j ++ ) {

      if ( j == 14 ) {
//        iterator++;
        readFloatFromWire();
        continue;
      }
      
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
  struct scpi_numeric start = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 1, -2.5, 2.5 );
  scpi_free_tokens( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_IV_START, chanId, (float) start.value );
  return SCPI_SUCCESS;
}

scpi_error_t set_iv_stop(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  struct scpi_numeric stop = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
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
  struct scpi_numeric mode = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0.02, 0, 1 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_MODE, chanId, (byte) mode.value );
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
  struct scpi_numeric step = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 1 );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  slaveCommand( COMMAND_TRACKER_STEP, chanId, (float) step.value );
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



void printDouble( double val, byte precision) {
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)
  if ( val < 0 && val > -1 ) {
    SerialUSB.print("-");
  }

  SerialUSB.print (int(val));  //prints the int part
  if ( precision > 0) {
    SerialUSB.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;
    while ( frac1 /= 10 )
      padding--;
    while (  padding--)
      SerialUSB.print("0");
    SerialUSB.print(frac, DEC) ;
  }
}

void swdec2Rising() {
  swdec2State = true;
}

void swdec2Falling() {
  swdec2State = false;
}

void swdec1Rising() {
  swdec1State = true;
}

void swdec1Falling() {
  swdec1State = false;
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


void setup() {


  Wire.begin();
  WireSlave.begin();
  
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  
  // I2CBus.begin(); // Other master
  // I2CBus.setClock(10L); // Let's make the I2C very slow

  // put your setup code here, to run once:
  scpi_init(&ctx);


  pinMode( PIN_POWER, OUTPUT );
  pinMode( PIN_DCDC_CS_1, OUTPUT );
  pinMode( PIN_DCDC_CS_2, OUTPUT );
  pinMode( PIN_DCDC_EN_1, OUTPUT );
  pinMode( PIN_DCDC_EN_2, OUTPUT );
  pinMode( PIN_LIGHT_PWM_1, OUTPUT );
  pinMode( PIN_LIGHT_PWM_2, OUTPUT );
  pinMode( PIN_OE, INPUT );
  pinMode( PIN_RELAY_CS, OUTPUT );
  pinMode( PIN_RELAY_RCK, OUTPUT );

  pinMode( PIN_LIGHT_1_ONOFF_READ, INPUT );
  pinMode( PIN_LIGHT_2_ONOFF_READ, INPUT );

  pinMode( PIN_RESET_SLAVE, OUTPUT );

  pinMode( PIN_LIGHT1_MODE_DEC, INPUT_PULLUP );
  pinMode( PIN_LIGHT2_MODE_DEC, INPUT_PULLUP );

  pinMode( PIN_SSR_1, OUTPUT );
  pinMode( PIN_SSR_2, OUTPUT );

  digitalWrite( PIN_SSR_1, LOW );
  digitalWrite( PIN_SSR_2, LOW );

  
  digitalWrite( PIN_POWER, LOW );
  digitalWrite( PIN_DCDC_CS_1, HIGH );
  digitalWrite( PIN_DCDC_CS_2, HIGH );
  digitalWrite( PIN_DCDC_EN_1, LOW );
  digitalWrite( PIN_DCDC_EN_2, LOW );
  digitalWrite( PIN_LIGHT_PWM_1, LOW );
  digitalWrite( PIN_LIGHT_PWM_2, LOW );

  digitalWrite( PIN_RELAY_CS, LOW );
  digitalWrite( PIN_RELAY_RCK, LOW );


  digitalWrite( PIN_RESET_SLAVE, LOW );


  SerialUSB.begin( 115200 );

  struct scpi_command* source;
  struct scpi_command* measure;
  struct scpi_command* iv;
  struct scpi_command* tracker;
  struct scpi_command* output;
  struct scpi_command* reserved;
  struct scpi_command* environment;
  struct scpi_command* light;
  struct scpi_command* dcdc;
  struct scpi_command* ssr;


  struct scpi_command* data;
  // Setting up SCPI commands
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);
  source = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "SOURCE", 6, "SOUR", 4, NULL);
  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE", 7, "VOLT", 4, set_dac_voltage);
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
  scpi_register_command(ssr, SCPI_CL_CHILD, "ON", 2, "ON", 2, ssr_on);
  scpi_register_command(ssr, SCPI_CL_CHILD, "OFF", 3, "OFF", 3, ssr_off);


  tracker = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "TRACKING", 8, "TRAC", 4, NULL);
  scpi_register_command(tracker, SCPI_CL_CHILD, "MODE", 4, "MODE", 4, set_tracking_mode);
  scpi_register_command(tracker, SCPI_CL_CHILD, "INTERVAL", 8, "INTE", 4, set_tracking_interval);
  scpi_register_command(tracker, SCPI_CL_CHILD, "FWBWTHRESHOLD", 13, "FWBW", 4, set_tracking_fwbw);
  scpi_register_command(tracker, SCPI_CL_CHILD, "BWFWTHRESHOLD", 13, "BWFW", 4, set_tracking_bwfw);
  scpi_register_command(tracker, SCPI_CL_CHILD, "STEP", 4, "STEP", 4, set_tracking_step);
  scpi_register_command(tracker, SCPI_CL_CHILD, "SWITCHDELAY", 11, "SWIT", 4, set_tracking_switchdelay);
  scpi_register_command(tracker, SCPI_CL_CHILD, "GAIN", 4, "GAIN", 4, set_gain);
  scpi_register_command(tracker, SCPI_CL_CHILD, "PHOTODIODE", 10, "PD", 2, set_device_photodiode);
  scpi_register_command(tracker, SCPI_CL_CHILD, "RESET", 5, "RESE", 5, trackin_reset_channel);

  
  scpi_register_command(tracker, SCPI_CL_CHILD, "SPEED", 5, "SPEE", 4, set_tracking_speed);


  environment = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "ENVIRONMENT", 11, "ENVI", 4, NULL);

  //scpi_register_command(environment, SCPI_CL_CHILD, "TEMPBOX?", 8, "TBOX?", 5, get_temperature_box);
  scpi_register_command(environment, SCPI_CL_CHILD, "TEMPBASE?", 9, "TBASE?", 6, get_temperature_base);
  scpi_register_command(environment, SCPI_CL_CHILD, "TEMPIR?", 7, "TIR?", 4, get_temperature_ir);
  scpi_register_command(environment, SCPI_CL_CHILD, "HUMIDITY?", 9, "HUMI?", 5, get_humidity_box);
  scpi_register_command(environment, SCPI_CL_CHILD, "PHOTODIODE", 10, "PD", 2, measure_pd);
  scpi_register_command(environment, SCPI_CL_CHILD, "SUNPHOTODIODE", 13, "SPD", 3, measure_pd_sun);
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


  dcdc = scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "DCDC", 4, "DCDC", 4, NULL );
  scpi_register_command(dcdc, SCPI_CL_CHILD, "ENABLE", 6, "ENAB", 4, scpi_enabledcdc );
  scpi_register_command(dcdc, SCPI_CL_CHILD, "ENABLED?", 8, "ENAB?", 5, scpi_isenableddcdc );
  scpi_register_command(dcdc, SCPI_CL_CHILD, "DISABLE", 7, "DISA", 4, scpi_disabledcdc );
  scpi_register_command(dcdc, SCPI_CL_CHILD, "VALUE", 5, "VALU", 4, scpi_valuedcdc );
  scpi_register_command(dcdc, SCPI_CL_CHILD, "CURRENT", 7, "CURR", 4, scpi_currentdcdc );
  scpi_register_command(dcdc, SCPI_CL_CHILD, "VOLTAGE", 7, "VOLT", 4, scpi_voltagedcdc );

  
  resetChannels();
  dcdc_setup();
  
  //reset_slave();
  delay( 1500 ); // Wait for 1 second
  //
  digitalWrite( PIN_POWER, HIGH ); // Turn on analog power
  slaveCommand( COMMAND_SETUP ); // Setup the aquisition ATSAMD chip and all peripherals (DAC/ADC/PGA)

}



