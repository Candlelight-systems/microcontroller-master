/*
 * light_expander.cpp
 *
 *  Created on: 8 Jun 2018
 *      Author: normanpellet
 */

#define LIGHT_EXPANDER_PWM_ADDR 96
#define LIGHT_EXPANDER_SWITCH_ADDR_WRITE 56
#define LIGHT_EXPANDER_SWITCH_ADDR_READ 56

#include "_parameters.c"

#if LIGHT_EXPANDER

#include <Arduino.h>
#include <Wire.h>
#include "light_expander.h"
#include "scpiparser.h"
#include "MPPT.h"
#include "slave.h"

struct lightExpander {
	byte slave = 1;
  byte enabled;
  float pwm;
};

struct lightExpander* lightExpanders[ 4 ] = {
	new lightExpander,
	new lightExpander,
	new lightExpander,
	new lightExpander
};


// PCA9685
// https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
void light_expander_init() {

	muxI2C( 1, Wire );
	  Wire.beginTransmission( LIGHT_EXPANDER_PWM_ADDR );
	  Wire.write( 0x00 ); // MODE REGISTER
	  Wire.write( 0b00100000 );
	  Wire.endTransmission();

	  Wire.beginTransmission( LIGHT_EXPANDER_PWM_ADDR );
	  Wire.write( 0x01 ); // MODE REGISTER 2
	  Wire.write( 0b00010011 );
	  Wire.endTransmission();

	  demuxI2C( Wire );
}


scpi_error_t light_expander_setPWM( struct scpi_parser_context* context, struct scpi_token* args ) {

	struct scpi_numeric pwm = scpi_parse_numeric( args->next->next->next->next->value, args->next->next->next->next->length, 0, 0, 1 );
	byte channel = getChannelFromSource( args->next->next->next );
	lightExpanders[ channel - 1 ]->pwm = ( float) pwm.value;
	light_expander_updatePWM();
	return SCPI_SUCCESS;
}


void light_expander_updatePWM() {

	light_expander_init();

	muxI2C( 1, Wire );



	  Wire.beginTransmission( LIGHT_EXPANDER_PWM_ADDR );
	  Wire.write( 0x06 ); // ADDRESS OF CH0

	  int on_1, off_1, on_2, off_2, on_3, off_3, on_4, off_4;
	  float duty_1 = lightExpanders[ 0 ]->pwm,
			duty_2 = lightExpanders[ 1 ]->pwm,
			duty_3 = lightExpanders[ 2 ]->pwm,
			duty_4 = lightExpanders[ 3 ]->pwm;

	  on_1 = 0;
	  off_1 = (duty_1) * 4095;

	  on_2 = 0;
	  off_2 = (duty_2) * 4095;

	  on_3 = 0;
	  off_3 = (duty_3) * 4095;

	  on_4 = 0;
	  off_4 = (duty_4) * 4095;

	  Wire.write( on_1 ); // LOW
	  Wire.write( on_1 >> 8 );

	  Wire.write( off_1 ); // LOW
	  Wire.write( off_1 >> 8 );

	  Wire.write( on_2 ); // LOW
	  Wire.write( on_2 >> 8 );

	  Wire.write( off_2 ); // LOW
	  Wire.write( off_2 >> 8 );

	  Wire.write( on_3 ); // LOW
	  Wire.write( on_3 >> 8 );

	  Wire.write( off_3 ); // LOW
	  Wire.write( off_3 >> 8 );

	  Wire.write( on_4 ); // LOW
	  Wire.write( on_4 >> 8 );

	  Wire.write( off_4 ); // LOW
	  Wire.write( off_4 >> 8 );

	  Wire.endTransmission();
}






void light_expander_updateSwitch() {

	byte wrByte = 0b10101010;

	muxI2C( 1, Wire );

	Wire.beginTransmission( LIGHT_EXPANDER_SWITCH_ADDR_WRITE );
	if( lightExpanders[ 0 ]->enabled ) { wrByte |= 0b00000001; }
	if( lightExpanders[ 1 ]->enabled ) { wrByte |= 0b00000100; }
	if( lightExpanders[ 2 ]->enabled ) { wrByte |= 0b00010000; }
	if( lightExpanders[ 3 ]->enabled ) { wrByte |= 0b01000000; }

	Wire.write( wrByte );

	Wire.endTransmission();
}

scpi_error_t light_expander_enable( struct scpi_parser_context* context, struct scpi_token* args ) {
	byte chanId = getChannelFromSource( args->next->next->next );
	  lightExpanders[ chanId ]->enabled = 1;
	  light_expander_updateSwitch();
	  return SCPI_SUCCESS;
}

scpi_error_t light_expander_disable( struct scpi_parser_context* context, struct scpi_token* args ) {
	byte chanId = getChannelFromSource( args->next->next->next );
	  lightExpanders[ chanId ]->enabled = 0;
	  light_expander_updateSwitch();
	  return SCPI_SUCCESS;
}

scpi_error_t light_expander_isEnabled( struct scpi_parser_context* context, struct scpi_token* args ) {
	muxI2C( 1, Wire );

	byte chanId = getChannelFromSource( args->next->next->next );
	  Wire.requestFrom( LIGHT_EXPANDER_SWITCH_ADDR_WRITE, 1 );

	  byte b = Wire.read();
		if( chanId == 1 ) { b &= 0b01000000; }
		if( chanId == 2  ) { b &= 0b00010000; }
		if( chanId == 3 ) { b &= 0b00000100; }
		if( chanId == 4 ) { b &= 0b00000001; }

	SerialUSB.println( b );
	 return SCPI_SUCCESS;

  }


#endif
