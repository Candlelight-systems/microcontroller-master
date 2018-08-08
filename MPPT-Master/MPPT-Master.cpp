
#include <Wire.h>
#include <SPI.h>

#include "parameters.h"
#include "MataHari.h"
#include "adc.h"
#include "dac.h"
#include "calibration.h"
#include "mux.h"
#include "sram.h"
#include "Arduino.h"


#include <float.h>


#if DAC_RESOLUTION == 12
	#define DAC_MAX_CODE 0x0FFF
	#define DAC_ZERO_VALUE 0x07FF
#elif DAC_RESOLUTION == 14
	#define DAC_MAX_CODE 0x3FFF
	#define DAC_ZERO_VALUE 0x1FFF
#elif DAC_RESOLUTION == 16
	#define DAC_MAX_CODE 0xFFFF
	#define DAC_ZERO_VALUE 0x7FFF
#endif

unsigned long timing = 0;
float iv_start;
float iv_stop;

bool i2cLease = false;
bool pause = false;
byte systemLock = false;


uint8_t	triggerIV = 0;
bool triggerPause = false;
uint8_t triggerAutozero = 0;
int i = 0;
bool debug = false;

byte channelByte;

byte valueByte = 0x00;
int32_t valueInt = 0x0000;
byte requestByte;

union {
	float f;
	long l;
	int32_t i;
	unsigned char bytes[4];
} u2;


struct channel {

	// ID of the channel, attributed in order
	byte			_chanId = 0;

	// Tracker mode. 1 == MPP, 2 == Voc, 3 == Jsc
	byte            tracker_mode = 0;

	// Relay enabled or not
	byte			enabled = 0;

	// Interval between two tracker measurement
	unsigned long   tracker_interval = 100;


	// MPP tracking:

	// delay time after ramp direaction switching
	unsigned long   tracker_switchdelay = 0;
	// Tracking step, in V
	uint8_t           tracker_step = 1;
	// Inverting threshold
	float           tracker_fwbw = 0.0;
	// Inverting threshold
	float           tracker_bwfw = 0.0;

	// IV CURVE

	// Start voltage (V)
	float           iv_start = 1.00;
	// Stop voltage (V)
	float           iv_stop = 0.00;
	// j-V rate (V/s)
	float           iv_rate = 0.2;
	// Measure the hysteresis
	bool            iv_hysteresis = 1;
	// Auto voltage start
	bool            iv_autoStart = 0;

	// IV temporary information
	uint16_t		_iv_iterator = 0;
	float           _iv_percentage;
	unsigned long   _iv_startTime = 0;
	byte            _iv_sweep_first;
	byte			_processing_iv = 0;
	float			_iv_start;

	bool            autoGain = true;

	bool            overload = false;

	int             _gainValue = B00000011;
	float           _tracker_last_power = 0.0;
	float           _tracker_last_current;
	float           _tracker_last_voltage;
	int32_t         _tracker_last_current_code;
	int32_t         _tracker_last_voltage_code;

	float           _jsc;
	float           _voc;

	bool            _processing_voc;
	bool            _processing_jsc;

	float           _voltage_mean = 0;
	float           _voltage_min = 0;
	float           _voltage_max = 0;

	float           _current_mean = 0;
	float           _current_min = 0;
	float           _current_max = 0;

	float           _power_mean = 0;
	float           _power_min = 0;
	float           _power_max = 0;

	unsigned long   _tracker_last_switch;
	unsigned long   _tracker_last_time;

	int16_t			_voc_counter;
	int             _tracker_direction = 1;
	int32_t         _tracker_nb;

	uint16_t        _tracker_voltage_code = DAC_ZERO_VALUE;
	uint16_t        _tracker_voltage_code_voc = DAC_ZERO_VALUE;
	uint16_t        _tracker_voltage_code_current = DAC_ZERO_VALUE;

	int8_t			pd = 0;
	bool            _tracker_fixedcurrent_vascending = true;
	int             _tracker_fixedcurrent_vswitches = 0;

	byte			_current_overload = 0;
};

float pd1Val = 0.0;
float pd2Val = 0.0;
int32_t pd1Code;
int32_t pd2Code;

union {
	float f;
	long l;
	int32_t i;
	unsigned char bytes[4];
} u;

bool measuring = false;

//float ivdata[256] = {};
uint16_t SRAM_STORE_INDICES[ 17 ] = { 0 };
uint16_t SRAM_READ_INDEX = 0;
int readcounter;

struct channel* channels[17] = {
	NULL,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel,
	new channel
};

static void   WDTsync() {
	while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

//============= resetWDT ===================================================== resetWDT ============
void resetWDT() {
	// reset the WDT watchdog timer.
	// this must be called before the WDT resets the system
	WDT->CLEAR.reg= 0xA5; // reset the WDT
	WDTsync();
}

//============= systemReset ================================================== systemReset ============
void systemReset() {
	// use the WDT watchdog timer to force a system reset.
	// WDT MUST be running for this to work
	WDT->CLEAR.reg= 0x00; // system reset via WDT
	WDTsync();
}

//============= setupWDT ===================================================== setupWDT ============
void setupWDT( uint8_t period) {
	// initialize the WDT watchdog timer

	WDT->CTRL.reg = 0; // disable watchdog
	WDTsync(); // sync is required

	WDT->CONFIG.reg = min(period,11); // see Table 17-5 Timeout Period (valid values 0-11)

	WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
	WDTsync();
}


bool SafetyCheckMaxCurrent( int8_t chanId ) {
	return false;
	// Current overload condition
	return channels[ chanId ]->_current_overload;
}


void loop() {

	// to see WDT work, comment (//) the next line, or uncomment to see WDT reset work
//	resetWDT();
		int i;

		if( ! pause && ( requestByte > 0 && ( requestByte < 0x20 || requestByte > 0x40 ) ) ) {

			struct channel* chan = channels[ channelByte ];

			switch( requestByte ) {


				case 0x14:  // SETUP
				setupAll();
				break;

				case 0x11:  // Set voltage
				setVoltage( channelByte, u2.f, true );
				break;

				case 0x15:  // Set DAC voltage
				setVoltageCode( channelByte, valueInt );
				break;

				case 0x16:
				ADC_SET_SPEED( valueByte );
				break;

				default:
					return;
				break;
			}

			requestByte = 0;
			i2cLease = false;
		}

		if( i2cLease || systemLock ) {
			return;
		}

		if( triggerPause ) {
			pause = true;
			triggerPause = false;
		}

		if( triggerIV && ! pause ) { // We delegate this action from the wire receive because this takes potentially a while. Wire.onReceive is based on interrupts, so the program will continue in the background. We don't want that. findVoc should be synchronous

			iv( triggerIV );
			triggerIV = 0;
		}
		/*
		if( triggerAutozero ) {
			int32_t	code;
			uint8_t gain = 0;
			unsigned char autoZero_i;

			for( autoZero_i = 1; autoZero_i < 16; autoZero_i ++ ) {
				for( gain = 0; gain <= 10; gain += 1 ) {
					*(float*) getCurrentOffset( autoZero_i, gain ) = ADCReadCurrentCode( autoZero_i, gain );
				}
			}

			triggerAutozero = 0;
		}
		*/

		if( triggerAutozero ) {
			int32_t	code;
			uint8_t gain = 0;
			for( gain = 0; gain <= 10; gain += 1 ) {
				*(float*) getCurrentOffset( triggerAutozero, gain ) = ADCReadCurrentCode( triggerAutozero, gain );
			}

			triggerAutozero = 0;
		}


		# if MONITOR_PD
			if( iv_number() == 0 && !pause ) {
				measuring = true;
				getPD1();
				getPD2();
				measuring = false;
			}
		#endif
		/* Read in a line and execute it. */


		// Infinite loop for the 16 channels
		for ( i = 1; i <= 16; i += 1 ) {

			if( ! channels[ i ]->enabled ) {
				continue;
			}

			// This statement should come prior to the j-V curve measurement.
			// It's a safety issue in case of overcurrent
			// but it is required to make the reading of the j-V curves work (no double SPI at the same time !)
			if( pause ) {
				continue;
			}

		//	do {
				getVoltage( i );
				getCurrent( i );
		//	} while( SafetyCheckMaxCurrent( i ) );


			if( channels[ i ]->_processing_iv ) {
				ivNext( channels[ i ] );
				continue;
			}

			trackChannel( i );
		}

}

// Request some data
void i2cRequest() {

	i2cLease = true;
	struct channel* chan = channels[ channelByte ];
	byte b;

	switch( requestByte ) {

		case 0x20: // Request track data
		getTrackData( channelByte );
		break;

		case 0x21: // Request voc value
		outputDouble( chan->_voc );
		break;

		case 0x22: // Request jsc value
		outputDouble( chan->_jsc );
		break;

		case 0x23: // Is IV being recorded
		digitalWrite( PIN_LED_INDICATOR, HIGH );
		 b = 0x00;
		if( chan->_processing_iv ) {
			b |= 0b00000001;
		}

		if( iv_number() > 0 ) {
			b |= 0b00000010;
		}

		Wire.write( b );

		break;

		case 0x24:  // Is the IV being recorded ?
		break;

		case 0x25:  // Is the voc being measured
		Wire.write( chan->_processing_voc );
		break;

		case 0x26: // Is the jsc being processed ?
		Wire.write( chan->_processing_jsc );
		break;

		case 0x27: // Returns the last current measurement. Only works if channel is enabled
		outputDouble( chan->_tracker_last_current );
		break;

		case 0x28:  // Returns the last current measurement. Only works if channel is enabled
		outputDouble( chan->_tracker_last_voltage );
		break;

		case 0x29: // Returns the last current measurement (24-bit word). Only works if channel is enabled
		outputInt32( chan->_tracker_last_current_code );
		break;

		case 0x2A:  // Returns the last current measurement (24-bit word). Only works if channel is enabled
		outputInt32( chan->_tracker_last_voltage_code );
		break;

		case 0x2B: // Get current dac voltage
		Wire.write( chan->_tracker_voltage_code >> 8 );
		Wire.write( chan->_tracker_voltage_code );
		break;

		case 0x2C: // Immediate PD1 measurement
		outputFloat( pd1Val );
		break;

		case 0x2D: // Immediate PD2 measurement
		outputFloat( pd2Val );
		break;

		case 0x2E: // Immediate voltage measurement
		outputInt32( pd1Code );
		break;

		case 0x2F: // Immediate voltage measurement
		outputInt32( pd2Code );
		break;

		case 0x30: // Length of the j-V buffer
			Wire.write( SRAM_STORE_INDICES[ channelByte ] >> 8 );
			Wire.write( SRAM_STORE_INDICES[ channelByte ] );
		break;

		case 0x31: // Reading open. Delegate that to the main loop


			if( ! SRAM_READ_OPEN ) {

				SRAM_READ_INDEX = 0;
				SRAM_READ_INIT( channelByte, SRAM_READ_INDEX );
			}

			int32_t outputValue;

			readcounter = 0;
			while( readcounter < 7 ) {

				if( SRAM_READ_INDEX >= SRAM_STORE_INDICES[ channelByte ] ) {
					SRAM_READ_TERMINATE();
					SRAM_STORE_INDICES[ channelByte ] = 0;
					break;
				}

				outputFloat( getVoltageFromCode( channelByte,  SRAM_READ( 3 ) ) );
				float f = getCurrentFromCode( channelByte, SRAM_READ( 3 ), ( SRAM_READ( 1 ) >> 24 ) & 0x0000000F );
				outputFloat( f );
				SRAM_READ_INDEX += 1;
				readcounter++;

			}


		break;

		case 0x32:
			Wire.write( pause );
		break;

		case 0x40:
			Wire.write( chan->_iv_iterator );
		break;

	}


	i2cLease = false;
}

void outputFloat( float f ) {

	union {
		float f;
		unsigned char bytes[4];
	} u;

	u.f = f;
	Wire.write( u.bytes, 4 );
}


void i2cReceive( int byteNumber ) {

	if( byteNumber == 0 ) {
		return;
	}

	i2cLease = true;
	int index = 0;
	byte c;
	unsigned char bytes[4];
	byte r;

	while ( index < byteNumber ) { // loop through all but the last

		if( Wire.available() == 0 ) {
			continue;
		}

		c = Wire.read();

		if( index == 0 ) { // First byte, request mode
			r = c;
		}

		if( index == 1 ) { // Second byte, channel mode
			channelByte = c;
		}


		// Then, one value, maximum, possibly comprised of 4-byte

		if( index == 2 ) { // Only 1 byte
			valueByte = c;
		}

		// 4-byte wide

		if( index > 1 ) {
			u2.bytes[ index - 2 ] = c;
			valueInt = valueInt << 8 | c;
		}

		index++;

		if( index > 5 ) {
			break;
		}
	}

	u2.i = valueInt;

	struct channel* chan = channels[ channelByte ];

	// Setter values
	switch( r ) {

		// The following instructions need to be executed in the runtime loop, not in the interrupt function. Otherwise, we have the risk of declaring an SPI instance when one already exists, crashing the whole program.
		// We leave the lease and the requestByte untouched. The next loop will pick it up
		case 0x16:
		case 0x15:  // Set DAC voltage
		case 0x11:  // Set voltage
		case 0x14:  // SETUP
		requestByte = r;
		return;
		break;


		case 0x43: // Change tracking mode
			chan->tracker_mode = valueByte;
		break;


		case 0x01: // Change tracking interval
		chan->tracker_interval = (unsigned long) u2.i;
		break;

		case 0x02: // Step interval (in float value (V))
		chan->tracker_step = valueByte;
		break;

		case 0x03:  // Forward - backward threshold
		chan->tracker_fwbw = u2.f;
		break;

		case 0x04:  // Backward - forward threshold
		chan->tracker_bwfw = u2.f;
		break;

		case 0x05:  // Delay on switch
		chan->tracker_switchdelay = (unsigned long) u2.i;
		break;

		case 0x06:  // IV: Start voltage
		chan->iv_start = (float) u2.f;
		break;

		case 0x07:  // IV: Stop voltage
		chan->iv_stop = (float) u2.f;
		break;

		case 0x08:  // IV: rate
		chan->iv_rate = (float) u2.f;
		break;

		case 0x09:  // IV: hysteresis
		chan->iv_hysteresis = valueByte;
		break;

		case 0x0A:  // IV: auto start
		chan->iv_autoStart = valueByte;
		break;

		case 0x0B:  // IV: auto start
		triggerIV = channelByte;

		break;

		case 0x0C:  // Auto gain
		chan->autoGain = valueByte;
		break;

		case 0x0D:  // Auto gain
		chan->autoGain = false;
		chan->_gainValue = valueByte & 0x0F;
		break;

		case 0x0E:  // Auto gain
		if( valueByte ) {
			triggerPause = true;
		} else {
			pause = false;
		}
		break;

		case 0x0F:  // Auto gain
		findVoc( channelByte );
		break;

		case 0x10:  // Auto gain
		findJsc( channelByte );
		break;

		case 0x12:  // Enable reference
		digitalWrite( PIN_REF_EN, 1 );
		break;

		case 0x13:  // Disable reference
		digitalWrite( PIN_REF_EN, 0 );
		break;

		case 0x17:
		chan->pd = valueInt;
		break;

		case 0x18:
			resetChanData( channelByte );
			chan->_tracker_voltage_code_voc = DAC_ZERO_VALUE; // Reset
			chan->_tracker_voltage_code_current = DAC_ZERO_VALUE; // Reset
		break;

		case 0x41:
			chan->enabled = 1;
		break;

		case 0x42:
			chan->enabled = 0;
		break;

		case 0x44:
			triggerAutozero = 1;
		break;

		case 0x45:
			NVIC_SystemReset();
		break;
	}

	if(  r < 0x20 || r > 0x40 ) {
		//r = 0;
		i2cLease = false;
	}	else {
		requestByte = r;
	}
}

void trackChannel( byte chanId ) {

	struct channel* chan = channels[ chanId ];

	// Current execution time
	timing = millis();

	// timing will overflow every 50 days approximately. We need to reset the _tracker_last_time in this case
	if( timing < chan->_tracker_last_time ) {
		chan->_tracker_last_time = timing;
	}


	// Temporary voc finding algorithm
	// In this mode we don't update the chan->_tracker_last_time. This way this method is executed as much as possible until the proper voltage is found
	if( chan->_processing_voc > 0 ) {

		if( timing - chan->_tracker_last_time > 10 ) {

			chan->_tracker_last_time = timing;
			chan->_voc_counter++;
			findVocSubroutine( chanId, chan->_voc_counter, false );

			if( chan->_processing_voc == 0 ) {
				resetChanData( chanId );
			}
		}
	//
		return;
	}

	// Temporary jsc finding. We set ourselves at 0V. In this case we don't update chan->_tracker_last_time, because it's the time reference we use to determine the jsc delay between the setting and the measurement.
	if( chan->_processing_jsc ) {

		// If we've been waiting long enough
		if( timing - chan->_tracker_last_time > 2000 ) {

			// Stop the temporary jsc finding
			chan->_processing_jsc = 0;
			// Acquire one more time the current
			trackChannelJsc( chanId );

			// Store the current
			chan->_jsc = chan->_current_mean;

			resetChanData( chanId );
			setVoltageCode( chanId, chan->_tracker_voltage_code );
			return;
		}

		return;
	}

	if (  chan->tracker_mode > 0 && timing - chan->_tracker_last_time > chan->tracker_interval ) {

		switch ( chan->tracker_mode ) {

			case 2:
			trackChannelVoc( chanId );
			break;

			case 3:
			trackChannelJsc( chanId );
			break;

			default:
			case 1:
			trackChannelPO( chanId );
			break;

		}
		chan->_tracker_last_time = millis();

	}
}

// Removed autogain values
bool checkAutogain( byte chanId, int32_t code ) {

	struct channel* chan = channels[ chanId ];

	if ( ( code < -2000000000 || code > 2000000000 ) && chan->_gainValue > B00000000 ) {
		chan->_gainValue--;
		return true;
		} else if ( ( code < 1000000000 && code > -1000000000 ) && chan->_gainValue < B00001010 ) {
		chan->_gainValue++;
		return true;
	}
	return false;
}


void trackChannelPO( byte channel ) {

	timing = millis();
	struct channel* chan = channels[ channel ];

	if ( timing - chan->_tracker_last_switch > chan->tracker_switchdelay ) {

		//   if ( checkAutogain( channel, current ) ) {
		//     resetChanData( channel );
		//     return;
		//   }

		float factor;


		// This case is no longer applicable. The JV trigger occurs in the main loop !
		//if( chan->_processing_iv ) { // Maybe the iV command came after the getVoltage and getCurrent method. In this case, no update should be done
		//	return;
		//}

		switch( chan->pd ) {
			case 1:
				factor = pd1Val;
			break;

			case 2:
				factor = pd2Val;
			break;

			default:
				factor = 1;
			break;
		}


		float power = chan->_tracker_last_current * chan->_tracker_last_voltage;

		float difference = power / factor - chan->_tracker_last_power;
		// Increase of power => Proper direction
		if ( difference > 0 ) {

			chan->_tracker_last_power = power;

			} else {

			float ratio = difference / fabs( chan->_tracker_last_power );
			//   Serial.println( ratio * 10000 );
			float regulation;

			if ( chan->_tracker_direction == -1 ) {
				regulation = chan->tracker_bwfw;
				} else {
				regulation = chan->tracker_fwbw;
			}

			if ( ratio < -regulation ) {

				chan->_tracker_direction *= -1;
				chan->_tracker_last_power = power / factor;
				chan->_tracker_last_switch = millis();
			}

		//	chan->_tracker_last_current = current;
		//	chan->_tracker_last_voltage = voltage;
		}

		chan->_voltage_mean =  ( ( ( chan->_voltage_mean ) * chan->_tracker_nb ) + chan->_tracker_last_voltage ) / ( chan->_tracker_nb + 1 );
		chan->_current_mean =  ( ( ( chan->_current_mean ) * chan->_tracker_nb ) + chan->_tracker_last_current ) / ( chan->_tracker_nb + 1 );
		chan->_power_mean =  ( ( ( chan->_power_mean ) * chan->_tracker_nb ) + power ) / ( chan->_tracker_nb + 1 );

		chan->_voltage_min = chan->_voltage_min < chan->_tracker_last_voltage ? chan->_voltage_min : chan->_tracker_last_voltage;
		chan->_voltage_max = chan->_voltage_max > chan->_tracker_last_voltage ? chan->_voltage_max : chan->_tracker_last_voltage;
		chan->_current_min = chan->_current_min < chan->_tracker_last_current ? chan->_current_min : chan->_tracker_last_current;
		chan->_current_max = chan->_current_max > chan->_tracker_last_current ? chan->_current_max : chan->_tracker_last_current;
		chan->_power_min   = chan->_power_min < power ? chan->_power_min : power;
		chan->_power_max   = chan->_power_max > power ? chan->_power_max : power;

		if ( ( chan->_tracker_voltage_code > 1 || chan->_tracker_direction > 0 ) && ( chan->_tracker_voltage_code < DAC_MAX_CODE || chan->_tracker_direction < 0 ) ) {

			int stepIncrement = chan->tracker_step;
			if ( stepIncrement <= 1 ) {
				stepIncrement = 1;
			}
			chan->_tracker_voltage_code += ( chan->_tracker_direction * stepIncrement );
		}

		// Irrelevant in a single threaded program
		//if( iv_doing == channel ) { // Just to be sure, do NOT update the voltage
		//	return;
		//}

		setVoltageCode( channel, chan->_tracker_voltage_code );

		chan->_tracker_nb++;
	}
}



void trackChannelVoc( byte chanId, uint8_t stepSize ) {
	struct channel* chan = channels[ chanId ];
	trackChannelCurrent( chanId, 0.0, stepSize );
	chan->_tracker_voltage_code_voc = chan->_tracker_voltage_code_current;
}

void trackChannelVoc( byte chanId ) {
	return trackChannelVoc( chanId, 1 );
}

void trackChannelCurrent( byte chanId, float targetCurrent ) {
	return trackChannelCurrent( chanId, targetCurrent, 1 );
}

void trackChannelCurrent( byte chanId, float targetCurrent, uint8_t stepSize ) {

	struct channel* chan = channels[ chanId ];

	if( stepSize <= 1 ) {
		stepSize = 1;
	}

	if ( chan->_tracker_last_current > targetCurrent ) {

		if ( ! chan -> _tracker_fixedcurrent_vascending ) {
			chan -> _tracker_fixedcurrent_vswitches ++;
			chan->_tracker_fixedcurrent_vascending = true;
		}

		chan->_tracker_voltage_code_current += stepSize;


	} else {

		if ( chan -> _tracker_fixedcurrent_vascending ) {
			chan -> _tracker_fixedcurrent_vswitches ++;
			chan->_tracker_fixedcurrent_vascending = false;
		}

		chan->_tracker_voltage_code_current -= stepSize;

	}

	// Overflowing error !
	if( chan->_tracker_voltage_code_current >= DAC_MAX_CODE || chan->_tracker_voltage_code_current <= 0 ) {
		chan->_tracker_fixedcurrent_vswitches += 100;
	}

	chan->_voltage_mean = chan->_tracker_last_voltage;
	chan->_current_mean = chan->_tracker_last_current;

	chan->_current_min = 0;
	chan->_voltage_min = 0;
	chan->_power_min = 0;
	chan->_current_max = 0;
	chan->_voltage_max = 0;
	chan->_power_max = 0;
	chan->_power_mean = 0;

	setVoltageCode( chanId, chan->_tracker_voltage_code_current );

	chan->_tracker_nb++;
}




void trackChannelJsc( byte chanId ) {

	struct channel* chan = channels[ chanId ];


	//  if ( checkAutogain( chanId, current ) ) {
	//    resetChanData( chanId );
	//    return;
	//  }

	chan->_voltage_mean = chan->_tracker_last_voltage;
	chan->_current_mean = chan->_tracker_last_current;
	chan->_current_min = 0;
	chan->_voltage_min = 0;
	chan->_power_min = 0;
	chan->_current_max = 0;
	chan->_voltage_max = 0;
	chan->_power_max = 0;
	chan->_power_mean = 0;


	setVoltage( chanId, 0.00 );
	chan->_tracker_nb++;
}


void getTrackData( byte channel ) {

	struct channel* chan = channels[ channel ];

	byte buffer[ 44 ];

	outputDouble( chan->_voltage_mean, buffer, 0 );
	outputDouble( chan->_current_mean, buffer, 4 );
	outputDouble( chan->_power_mean, buffer, 8 );
	outputDouble( chan->_voltage_min, buffer, 12 );
	outputDouble( chan->_current_min, buffer, 16 );
	outputDouble( chan->_power_min, buffer, 20 );
	outputDouble( chan->_voltage_max, buffer, 24 );
	outputDouble( chan->_current_max, buffer, 28 );
	outputDouble( chan->_power_max, buffer, 32 );
	outputInt32( chan->_tracker_nb, buffer, 36 );
	outputInt32( chan->_gainValue, buffer, 40 );

	Wire.write( buffer, 44 );
	resetChanData( channel );
}

void outputDouble( float value ) {
	u.f = value;
	Wire.write( u.bytes, 4 );
}


void outputDouble( float value, unsigned char *buffer, unsigned char index ) {
	int32_t val = 0;
	u.f = value;
	//memcpy( &val, &value, 4 );
	buffer[ index ] = u.bytes[ 0 ];
	buffer[ index + 1 ] = u.bytes[ 1 ];
	buffer[ index + 2 ] = u.bytes[ 2 ];
	buffer[ index + 3 ] = u.bytes[ 3 ];
}


void outputInt32( int32_t val, unsigned char *buffer, unsigned char index ) {
	buffer[ index ] = val >> 24;
	buffer[ index + 1 ] = val >> 16;
	buffer[ index + 2 ] = val >> 8;
	buffer[ index + 3 ] = val >> 0;
}


void outputInt32( int32_t val ) {

	byte buffer[ 4 ];
	buffer[ 0 ] = val >> 24;
	buffer[ 1 ] = val >> 16;
	buffer[ 2 ] = val >> 8;
	buffer[ 3 ] = val >> 0;

	Wire.write( buffer, 4 );
}


void resetChanData( byte chanId ) {

	struct channel* chan = channels[ chanId ];

	chan->_voltage_max = -10.0;
	chan->_current_max = -10.0;
	chan->_power_max = -10.0;

	chan->_voltage_min = 10.0;
	chan->_current_min = 10.0;
	chan->_power_min = 10.0;

	chan->_voltage_mean = 0.0;
	chan->_current_mean = 0.0;
	chan->_power_mean = 0.0;

	chan->_tracker_nb = 0;
}


void findVoc( byte chanId ) {
	findVoc( chanId, false );
}

void findVoc( byte chanId, bool synchronous ) {

	struct channel* chan = channels[ chanId ];


	if( chan->_processing_voc ) {
		return;
	}
	chan->_tracker_fixedcurrent_vswitches = 0;
	chan->_processing_voc = 1;
	chan->_voc_counter = 0;
	chan->_tracker_voltage_code_current = chan->_tracker_voltage_code_voc;
	setVoltageCode( chanId, chan->_tracker_voltage_code_current );

	uint16_t i = 0;
	if( synchronous ) {
		while( ! findVocSubroutine( chanId, i, synchronous ) ) {
			i++;
		}
	}
}

bool findVocSubroutine( uint8_t chanId, uint16_t counter, bool synchronous ) {

	struct channel* chan = channels[ chanId ];
	if( chan->_tracker_fixedcurrent_vswitches >= 10 || counter > 3000 ) {


		// Store the voltage
		chan->_voc = chan->_voltage_mean;
		setVoltageCode( chanId, channels[ chanId ]->_tracker_voltage_code );
		resetChanData( chanId );
		chan->_processing_voc = 0;
		return true;
	}

	// Stop the temporary voc finding
	if( synchronous ) {
		getVoltage( chanId );
		getCurrent( chanId );
	}

	trackChannelVoc( chanId, 10 - 1 * chan->_tracker_fixedcurrent_vswitches );
	return false;

}

void findJsc( byte chanId ) {

	struct channel* chan = channels[ chanId ];
	chan->_processing_jsc = 1;
	chan->_tracker_last_time = millis(); // Reset the timer to the current value
	trackChannelJsc( chanId ); // Set the channel at 0V already. This won't be updated until the actual measurement !
}


int iv_number() {
	int counter = 0;
	for( int i = 1; i <= 16; i++ ) {
		if( channels[ i ]->_processing_iv ) {
			counter++;
		}
	}
	return counter;
}


void iv( byte chanId ) {

	measuring = true;
	struct channel* chan = channels[ chanId ];

	// An IV curve is already being done.
	if ( chan->_processing_iv ) {
	//	iv_number--;
//		return;
	}

	chan->_iv_percentage = 0.0;
	chan->_iv_iterator = 0;
	chan->_iv_sweep_first = 1;
	chan->_processing_iv = 1;

	if ( chan -> iv_autoStart ) {

		findVoc( chanId, true );
		chan->_iv_start = getDacVoltageFromCode( chanId, chan->_tracker_voltage_code_current ) + 0.02; // Add 20mV to be sure to be above voc

	} else {
		chan->_iv_start = chan->iv_start;
	}


	// Sets the first voltage point
	setVoltage( chanId, chan->_iv_start );

	// we need to let the driving filter stabilize, as the voltage / current will be directly required !
	delay( 10 );

	chan->_iv_startTime = millis();

	measuring = false;
};

void ivNext( struct channel *chan ) {
	return ivNext( chan, chan->_iv_start, chan->iv_stop );

}

void ivNext( struct channel *chan, float vStart, float vStop ) {

	if( chan->iv_rate < 0.0001 ) {
		chan->_processing_iv = 0;

		return;
	}

	unsigned long totalTime = (unsigned long) ( (float) abs( vStop - vStart ) / chan->iv_rate );
	float percentage = (float) ( millis() - chan->_iv_startTime ) / 1000 / (float) totalTime;

	float voltage, current;
	if ( percentage - chan->_iv_percentage < 0.01 ) {
		  return;
	}

	if( chan->_iv_iterator >= 1000 ) {
		chan->_processing_iv = 0;

		return;
	}

	measuring = true;

	SRAM_STORE_IV( (uint8_t) chan->_chanId, SRAM_STORE_INDICES[ chan->_chanId ], chan->_tracker_last_voltage_code, chan->_tracker_last_current_code, chan->_gainValue );


//	ivdata[ iv_iterator ] = getVoltage( iv_doing );

//	iv_iterator++;

//	if( iv_iterator % 16 == 15 )  { // Byte skipping
//		iv_iterator++;
//	}

	//ivdata[ iv_iterator ] = getCurrent( iv_doing );

//	iv_iterator++;

//	if( iv_iterator % 16 == 15 )  { // Byte skipping
//		iv_iterator++;
//	}


	//percentage = (float) ( millis() - iv_startTime ) / 1000 /  (float) totalTime;
	chan->_iv_percentage = percentage;

	if ( percentage > 1 && chan->_iv_sweep_first == 1 ) {
		percentage = 1;
		chan->_iv_sweep_first = 0;
	} else if ( percentage > 2 ) {
		percentage = 2;
	}

	if ( percentage == 1 && ! chan->iv_hysteresis ) {
		// End of IV
		setVoltageCode( chan->_chanId, chan->_tracker_voltage_code );
		resetChanData( chan->_chanId );
		chan->_processing_iv = 0;

		measuring = false;
		return;

	} else if ( percentage == 2 ) {
		// End of IV
		setVoltageCode( chan->_chanId, chan->_tracker_voltage_code );
		resetChanData( chan->_chanId );
		chan->_processing_iv = 0;

		measuring = false;
		return;
	}

	float currentVoltage;

	// Vstart: -2
	// Vend 0
	// percentage = 0.5
	// CurrentVoltage = -2 -(-2-0)*0.5 = -2 + 1 = 1;
	// Works in reverse too...

	if ( chan->_iv_sweep_first == 1 ) {
		currentVoltage = vStart - ( vStart - vStop ) * percentage;
	} else {
		currentVoltage = ( vStop - vStart ) * ( 1 - ( percentage - 1 ) ) + vStart;
	}

	setVoltage( chan->_chanId, currentVoltage );
	measuring = false;
	return;
}


////// CURRENT READING

void getCurrent( unsigned char channel ) {

	int32_t code;

	do {
		code = getCurrentCode( channel );
	} while ( channels[ channel ]->autoGain && checkAutogain( channel, code ) );

	channels[ channel ]->_tracker_last_current_code = code;
	channels[ channel ]->_tracker_last_current = getCurrentFromCode( channel, channels[ channel ]->_tracker_last_current_code );
	channels[ channel ]->_current_overload = ADC_OUT_OF_RANGE(); // Only refers to the latest conversion !
}

float getCurrentFromCode( byte channel, int32_t code ) {
	int pgaVal = channels[ channel ]->_gainValue;
	return getCurrentFromCode( channel, code, pgaVal );
}

float getCurrentFromCode( byte channel, int32_t code, int8_t pgaVal ) {
	return (float) ( (float) code - *getCurrentOffset( channel, pgaVal ) )  / getCurrentGain( channel, pgaVal );
}


int32_t getCurrentCode( byte channel ) {
	return ADCReadCurrentCode( channel, channels[ channel ]->_gainValue );
}


////// VOLTAGE READING

void getVoltage( byte channel ) {
	channels[ channel ]->_tracker_last_voltage_code = ADCReadVoltageCode( channel, ADC_VOLTAGE_GAIN );
	channels[ channel ]->_tracker_last_voltage = getVoltageFromCode( channel, channels[ channel ]->_tracker_last_voltage_code );
}

float getVoltageFromCode( byte channel, int32_t code ) {
	return (float) ( (float) code - getVoltageOffset( channel ) ) /  getVoltageGain( channel );
}

int32_t getVoltageCode( byte channel ) {
	return ADCReadVoltageCode( channel, ADC_VOLTAGE_GAIN );
}

////// VOLTAGE SETTING

void setVoltage( byte chanId, float voltage ) {
	setVoltageCode( chanId, (uint16_t) ( ( voltage - getDacVoltageOffset( chanId ) ) / getDacVoltageGain( chanId ) ) );
}

void setVoltage( byte chanId, float voltage, bool assignToChannel ) {
	uint16_t voltageCode = (uint16_t) ( ( voltage - getDacVoltageOffset( chanId ) ) / getDacVoltageGain( chanId ) );
	if ( assignToChannel ) {
		channels[ chanId ]->_tracker_voltage_code = voltageCode;
	}
	setVoltageCode( chanId, voltageCode );
}

float getDacVoltageFromCode( byte chanId, uint16_t code ) {
	return (float) ( (float) code * getDacVoltageGain( chanId ) + getDacVoltageOffset( chanId ) );
}

void setVoltageCode( byte chanId, uint16_t voltageCode ) {

	if( voltageCode > (uint16_t) DAC_MAX_CODE ) {
		voltageCode = DAC_MAX_CODE;
	}
	setDAC( chanId, voltageCode );
}


void getPD1() {
	int32_t val = measurePD( 1 );
	pd1Code = val;
	pd1Val = calibrationPD( val, 1 );
}


void getPD2() {
	int32_t val = measurePD( 2 );
	pd2Code = val;
	pd2Val = calibrationPD( val, 2 );
}

int32_t measurePD( byte PD ) {

	if ( PD == 1 ) {
		digitalWrite( PIN_PD_SWITCH, 0 );
	} else {
		digitalWrite( PIN_PD_SWITCH, 1 );
	}

	return ADCReadPDCode();
}

void setupAll() {

	for( int i = 1; i <= 16; i++ ) {
		channels[ i ]->_processing_iv = 0;
		channels[ i ]->_processing_voc = 0;
		channels[ i ]->_processing_jsc = 0;
	}

	digitalWrite( PIN_REF_EN, HIGH );
	ADCSetup();
	configureDAC();
	SRAM_INIT();
}

void setup() {

	Wire.begin(8);                // join i2c bus with address #8
	Wire.onRequest( i2cRequest ); // register event
	Wire.onReceive( i2cReceive ); // register event

	// Setting the pin direction


	pinMode( 13, INPUT );
	pinMode( PIN_LED_INDICATOR, OUTPUT );
	pinMode( PIN_REF_EN, OUTPUT );


	pinMode( PIN_MUX0, OUTPUT );
	pinMode( PIN_MUX1, OUTPUT );
	pinMode( PIN_MUX2, OUTPUT );
	pinMode( PIN_MUX3, OUTPUT );
	pinMode( PIN_MUXEN1, OUTPUT );
	pinMode( PIN_MUXEN2, OUTPUT );

	pinMode( PIN_PD_SWITCH, OUTPUT );

	pinMode( PIN_DAC_CS_1, OUTPUT );
	pinMode( PIN_DAC_CS_2, OUTPUT );
	pinMode( PIN_DAC_RESET, OUTPUT );

	pinMode( PIN_PGA_CS, OUTPUT );
	pinMode( PIN_PGA_ERR, INPUT );
	pinMode( PIN_PGA_CURRENT_BUFFER, OUTPUT );



	// Setting the pin values
	digitalWrite( PIN_LED_INDICATOR, LOW );
	digitalWrite( PIN_REF_EN, LOW );

	digitalWrite( PIN_MUX0, LOW );
	digitalWrite( PIN_MUX1, LOW );
	digitalWrite( PIN_MUX2, LOW );
	digitalWrite( PIN_MUX3, LOW );
	digitalWrite( PIN_MUXEN1, HIGH );
	digitalWrite( PIN_MUXEN2, HIGH );

	digitalWrite( PIN_PD_SWITCH, LOW );

	digitalWrite( PIN_DAC_CS_1, HIGH );
	digitalWrite( PIN_DAC_CS_2, HIGH );
	digitalWrite( PIN_DAC_RESET, HIGH );

	digitalWrite( PIN_PGA_CS, HIGH );
	digitalWrite( PIN_PGA_CURRENT_BUFFER, LOW );


	SPI.begin();

	setupAll();

	for( i = 1; i <= 16; i++ ) {
		channels[ i ]->_chanId = (byte) i;
	}

//	setupWDT( 11 ); // initialize and activate WDT with maximum period



}

