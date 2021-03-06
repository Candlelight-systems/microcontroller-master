#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <wiring_private.h>
#include <float.h>

#include "MPPT.h"

#include "commands.h"
#include "scpiparser.h"
#include "light.h"
#include "light_expander.h"
#include "ssr.h"
#include "_parameters.c"

bool swdec1State = false;
bool swdec2State = false;

bool debug = false;
unsigned char statusByte = 0x00;;


PID PID_SSR1;
PID PID_SSR2;

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
      if (read_length > 0) {

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

void slaveCommand( byte commandByte, byte chanId, float value ) {

  // Overite bytes of union with float variable
  float f = value;
  u.f = f;

  Wire.beginTransmission( 8 );
  Wire.write( commandByte );
  Wire.write( chanId );

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

void pause_wait() {
	byte paused = false;

	slaveCommand( COMMAND_PAUSE, -1, (byte) 1 );
	do {
	  delay( 10 );
	  slaveCommand( COMMAND_REQ_PAUSE );
	  paused = readByteFromWire( );
	} while( paused == 0 );

}

void pause_resume() {
	slaveCommand( COMMAND_PAUSE, -1, (byte) 0 );
}
int32_t readIntFromWire( ) {

  byte b1 = Wire.read();
  byte b2 = Wire.read();
  byte b3 = Wire.read();
  byte b4 = Wire.read();
  return (int32_t) ( ( ( int32_t ) b1  << 24 & 0xFF000000 ) | ( ( int32_t ) b2  << 16 & 0x00FF0000 )  | ( ( int32_t ) b3 << 8 & 0x0000FF00 )  | ( ( int32_t ) b4 << 0 & 0x000000FF ) );
}



uint16_t readUInt16FromWire( ) {

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

void getTrackData( unsigned char channel ) {


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

byte getChannelFromSource( struct scpi_token* args ) {

  int a = (int) ( args->value[2] - '0' );
  if ( args->length == 4 ) {
    a = a * 10 + (int) ( args->value[3] - '0' );
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


byte getChannel( struct scpi_token* args ) {
	return getChannelFromSource( args->next->next );
}

void reset_slave() {
//  digitalWrite( PIN_RESET_SLAVE, HIGH );
//  delay( 1000 );
//  digitalWrite( PIN_RESET_SLAVE, LOW );
}




void printDouble( float val, byte precision) {
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




void setup() {



  Wire.begin();
  WireSlave.begin();

  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  // I2CBus.begin(); // Other master
  // I2CBus.setClock(10L); // Let's make the I2C very slow

  // put your setup code here, to run once:


  pinMode( PIN_POWER, OUTPUT );
  /*pinMode( PIN_DCDC_CS_1, OUTPUT );
  pinMode( PIN_DCDC_CS_2, OUTPUT );
  pinMode( PIN_DCDC_EN_1, OUTPUT );
  pinMode( PIN_DCDC_EN_2, OUTPUT );*/
  pinMode( PIN_LIGHT_PWM_1, OUTPUT );
  pinMode( PIN_LIGHT_PWM_2, OUTPUT );
  pinMode( PIN_OE, INPUT );
  pinMode( PIN_RELAY_CS, OUTPUT );
  pinMode( PIN_RELAY_RCK, OUTPUT );
  pinMode( PIN_RESET_I2C, OUTPUT );
  pinMode( 40 , OUTPUT );

  pinMode( PIN_LIGHT_1_ONOFF_READ, INPUT_PULLDOWN );
  pinMode( PIN_LIGHT_2_ONOFF_READ, INPUT_PULLDOWN );

  pinMode( PIN_RESET_SLAVE, OUTPUT );

  pinMode( PIN_LIGHT1_MODE_DEC, INPUT_PULLUP );
  pinMode( PIN_LIGHT2_MODE_DEC, INPUT_PULLUP );

  pinMode( PIN_SSR_1, OUTPUT );
  pinMode( PIN_SSR_2, OUTPUT );

  	  pinMode( MUX_CURRENT_SWITCH, OUTPUT );
  digitalWrite( PIN_SSR_1, LOW );
  digitalWrite( PIN_SSR_2, LOW );

  digitalWrite( PIN_RESET_I2C, HIGH );


  digitalWrite( PIN_POWER, LOW );
  /*digitalWrite( PIN_DCDC_CS_1, HIGH );
  digitalWrite( PIN_DCDC_CS_2, HIGH );
  digitalWrite( PIN_DCDC_EN_1, LOW );
  digitalWrite( PIN_DCDC_EN_2, LOW );*/
  digitalWrite( PIN_LIGHT_PWM_1, LOW );
  digitalWrite( PIN_LIGHT_PWM_2, LOW );

  digitalWrite( PIN_RELAY_CS, LOW );
  digitalWrite( PIN_RELAY_RCK, LOW );
  digitalWrite( PIN_RESET_SLAVE, HIGH );


  digitalWrite( MUX_CURRENT_SWITCH, HIGH );

  	  pinMode( 10, OUTPUT );
  	  digitalWrite( 10, HIGH );

  SerialUSB.begin( 115200 );

  SPI.begin();
  registerSCPICommands();

  resetChannels();
 // dcdc_setup();


  reset_slave();
  digitalWrite( PIN_POWER, HIGH ); // Turn on analog power
  delay( 1500 ); // Wait for 1 second
  resetChannels();
  //

  slaveCommand( COMMAND_SETUP ); // Setup the aquisition ATSAMD chip and all peripherals (DAC/ADC/PGA)



  // PA09: SSR Channel 1 (slow PWM)
  PORT->Group[ PORTA ].PINCFG[ 9 ].bit.PMUXEN = 1;
  PORT->Group[ PORTA ].PMUX[ 9 >> 1 ].reg = PORT_PMUX_PMUXO_E;

  // PA18: SSR Channel 2 (slow PWM)
  PORT->Group[ PORTA ].PINCFG[ 18 ].bit.PMUXEN = 1;
  PORT->Group[ PORTA ].PMUX[ 18 >> 1 ].reg = PORT_PMUX_PMUXE_F;


  // PA07: PWM Channel 2 (light control)
  PORT->Group[ PORTA ].PINCFG[ 7 ].bit.PMUXEN = 1;
  PORT->Group[ PORTA ].PMUX[ 7 >> 1 ].reg = PORT_PMUX_PMUXO_E;


  // PA06: PWM Channel 1 (light control)
  PORT->Group[ PORTA ].PINCFG[ 6 ].bit.PMUXEN = 1;
  PORT->Group[ PORTA ].PMUX[ 6 >> 1 ].reg |= PORT_PMUX_PMUXE_E;
//  PORT->Group[ PORTA ].PMUX[ 6 >> 1 ].reg = PORT_PMUX_PMUXO_E;



  REG_GCLK_GENDIV = GCLK_GENDIV_DIV( 0x01 ) |     // Divide the 32kHz clock source by a 1 divisor
                      GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                       GCLK_GENCTRL_GENEN |         // Enable GCLK4
					   GCLK_GENCTRL_SRC_DFLL48M |// Use the 48MHz as the source frequency
                       GCLK_GENCTRL_ID(4);          // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Feed GCLK4 to TCC0 and TCC1
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                       GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
    REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                      TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

    REG_TCC1_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                      TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
    while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization


    // Each timer counts up to a maximum or TOP value set by the PER register,
    // this determines the frequency of the PWM operation:
    REG_TCC0_PER = 0xFFFFFF;  // 2^14 iterations max
    while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization

    REG_TCC1_PER = 0x3FF;  // 2^14 iterations max
    while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization


    // We can give another prescaler value to the TCC, but it's not needed here
    REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 1024 for TCC0 (SSR control)
                        TCC_CTRLA_ENABLE;             // Enable the TCC0 output
    while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

    REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    // Divide GCLK4 by 1 for TCC1 (light control)
                        TCC_CTRLA_ENABLE;             // Enable the TCC0 output
    while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization



    REG_TCC0_CC1 = 0x0;         // TCC0 CC3 - on D7
    while (TCC0->SYNCBUSY.bit.CC1 );                // Wait for synchronization

    REG_TCC0_CC2 = 0xFFFFFF;         // TCC0 CC3 - on D7
    while (TCC0->SYNCBUSY.bit.CC2 );                // Wait for synchronization


    REG_TCC1_CC1 = 500;         // TCC0 CC3 - on D7
    while (TCC1->SYNCBUSY.bit.CC1 );                // Wait for synchronization

    REG_TCC1_CC0 = 700;         // TCC0 CC3 - on D7
    while (TCC1->SYNCBUSY.bit.CC0 );                // Wait for synchronization


    PID_SSR1.target = 30.0; // 30°C
    PID_SSR2.target = 30.0; // 30°C


	#if LIGHT_EXPANDER
		light_expander_init();
	#endif
}

