#include <Arduino.h>
#include <Wire.h>
#define CLIENT_ID 10023


#define PIN_POWER 4
#define PIN_LIGHT_PWM_1 8
#define PIN_LIGHT_PWM_2 9
#define PIN_OE 19
#define PIN_RELAY_CS 25
#define PIN_RELAY_RCK 6
#define PIN_LIGHT_1_ONOFF_READ 16
#define PIN_LIGHT_2_ONOFF_READ 15

#define PIN_SSR_1 3
#define PIN_SSR_2 10

#define PIN_RESET_SLAVE 38
#define PIN_RESET_I2C 30
#define PIN_LIGHT1_MODE_DEC 2 
#define PIN_LIGHT2_MODE_DEC 5

#define I2C_ADDR_SLAVE 8

#define MUX_CURRENT_SWITCH 18

#define ADC_MUX0_ADDR 0x00
#define ADC_VBIAS_ADDR 0x01
#define ADC_MUX1_ADDR 0x02
#define ADC_SYS0_ADDR 0x03
#define ADC_OFC0_ADDR 0x04
#define ADC_OFC1_ADDR 0x05
#define ADC_OFC2_ADDR 0x06
#define ADC_FSC0_ADDR 0x07
#define ADC_FSC1_ADDR 0x08
#define ADC_FSC2_ADDR 0x09
#define ADC_IDAC0_ADDR 0x0A
#define ADC_IDAC1_ADDR 0x0B
#define ADC_GPIOCFG_ADDR 0x0C
#define ADC_GPIODIR_ADDR 0x0D
#define ADC_GPIODAT_ADDR 0x0E


#define ADC_RATE_5SPS B00000000
#define ADC_RATE_10SPS B00000001
#define ADC_RATE_20SPS B00000010
#define ADC_RATE_40SPS B00000011
#define ADC_RATE_80SPS B00000100
#define ADC_RATE_160SPS B00000101
#define ADC_RATE_320SPS B00000110
#define ADC_RATE_640SPS B00000111
#define ADC_RATE_1000SPS B00001000
#define ADC_RATE_2000SPS B00001001

#define ADC_PGA_1 B00000000
#define ADC_PGA_2 B00010000
#define ADC_PGA_4 B00100000
#define ADC_PGA_8 B00110000
#define ADC_PGA_16 B01000000
#define ADC_PGA_32 B01010000
#define ADC_PGA_64 B01100000
#define ADC_PGA_128 B01110000


#define CALIBRATION_4_20MA_GAIN 0.020071
#define CALIBRATION_4_20MA_OFFSET 0.07333


// We removed the 0x00 command. The byte 0x00 means no command received from now on.RE

#define COMMAND_TRACKER_INTERVAL 		0x01
#define COMMAND_TRACKER_STEP 			0x02
#define COMMAND_TRACKER_FWBW 			0x03
#define COMMAND_TRACKER_BWFW 			0x04
#define COMMAND_TRACKER_SWITCHDELAY 	0x05
#define COMMAND_IV_START 				0x06
#define COMMAND_IV_STOP 				0x07
#define COMMAND_IV_RATE 				0x08
#define COMMAND_IV_HYSTERESIS 			0x09
#define COMMAND_IV_AUTOSTART 			0x0A
#define COMMAND_IV_TRIGGER 				0x0B
#define COMMAND_AUTOGAIN 				0x0C
#define COMMAND_GAIN 					0x0D
#define COMMAND_PAUSE 					0x0E
#define COMMAND_VOC_TRIGGER 			0x0F
#define COMMAND_JSC_TRIGGER 			0x10
#define COMMAND_SET_VOLTAGE 			0x11
#define COMMAND_REFERENCE_ENABLE 		0x12
#define COMMAND_REFERENCE_DISABLE 		0x13
#define COMMAND_SETUP 					0x14
#define COMMAND_SET_VOLTAGE_CODE 		0x15
#define COMMAND_TRACKER_SPEED 			0x16
#define COMMAND_SET_DEVICE_PHOTODIODE 	0x17
#define COMMAND_TRACKER_RESET 			0x18
#define COMMAND_ENABLE_CHANNEL 			0x41
#define COMMAND_DISABLE_CHANNEL 		0x42
#define COMMAND_TRACKER_MODE			0x43
#define COMMAND_AUTOZERO				0x44
#define COMMAND_RESET					0x45
#define COMMAND_TRACKER_VOLTAGE			0x46

#define COMMAND_REQ_TRACKDATA 			0x20
#define COMMAND_REQ_VOC 				0x21
#define COMMAND_REQ_JSC 				0x22
#define COMMAND_REQ_IV_DOING 			0x23
#define COMMAND_REQ_VOC_DOING 			0x25
#define COMMAND_REQ_JSC_DOING 			0x26
#define COMMAND_REQ_CURRENT 			0x27
#define COMMAND_REQ_VOLTAGE 			0x28
#define COMMAND_REQ_CURRENT_CODE 		0x29
#define COMMAND_REQ_VOLTAGE_CODE 		0x2A
#define COMMAND_REQ_VOLTAGE_DAC_CODE 	0x2B
#define COMMAND_REQ_PD2 				0x2C
#define COMMAND_REQ_PD1 				0x2D
#define COMMAND_REQ_PD2_CODE	 		0x2E
#define COMMAND_REQ_PD1_CODE	 		0x2F
#define COMMAND_REQ_IV_LENGTH 			0x30
#define COMMAND_REQ_PAUSE	 			0x32


void pause_wait();
void pause_resume();

int16_t ADCReadPDCode();
void ADC_WAKEUP();
void ADC_SLEEP();
void ADC_SYNC();
void ADC_RESET();
int16_t ADC_READ_ONCE();
void ADC_MODE_READ_CONTINUOUS();
void ADC_MODE_STOP_READ_CONTINUOUS();
void ADC_SYSTEM_OFFSET_CAL();
void ADC_SYSTEM_GAIN_CAL();
void ADC_SYSTEM_SELF_OFFSET_CAL();
uint8_t ADC_READ_REGISTER( uint8_t registerNumber );
uint8_t readADC();
uint8_t readADC( uint8_t value );
void sendADC( uint8_t toSend );
void ADC_WRITE_REGISTER( uint8_t registerNumber, uint8_t newValue );
void ADC_MUX_CURRENT();
void ADC_MUX_VOLTAGE();
void ADC_MUX_IN_REFERENCE();
void ADC_POWERUP_REFERENCE();
void ADC_POWERDOWN_REFERENCE();
void ADC_SET_RATE( uint8_t ADCRate );
void ADC_ENABLE();
void ADC_DISABLE();
void ADC_MONITOR_PINREADY();
void ADCSetup();
int16_t ADCReadCurrentCode();
float ADCReadCurrent();
int16_t ADCReadVoltageCode();
int16_t ADCReadVoltageCode( int chanId );
int16_t ADCReadCurrentCode( int chanId, byte pgaGain );
float ADCReadVoltage( int chanId );
float ADCReadCurrent( int chanId, byte pgaGain );
void ADCCalibrateOffset();
void ADCCalibrateGain();

void DACUpdate( int dacID );
void configureDAC();
void setDAC( int channel, uint16_t value );

byte getChannel( struct scpi_token* args );
byte getChannelFromSource( struct scpi_token* args );


void muxVoltage( int chanId );
void muxCurrent( int chanId );
void mux( int chanId );


void enableChannel( int chanId );
void disableChannel( int chanId );
void updateChannels();
void disableChannels();
void resetChannels();

float measurePD( int chanId );
float measurePDSun( int chanId );

void printDouble2( float f );
void printDouble( float f, unsigned char commas );

void reset_slave();


void slaveCommand( byte commandByte );
void slaveCommand( byte commandByte, byte chanId );
void slaveCommand( byte commandByte, byte chanId, float value );
void slaveCommand( byte commandByte, byte chanId, int32_t value );
void slaveCommand( byte commandByte, byte chanId, byte value );

void readFromWire( int howMany );
byte readByteFromWire( );
int32_t readIntFromWire( );
uint16_t readUInt16FromWire( );
float readFloatFromWire();

void getTrackData( unsigned char chanId );

extern bool debug;
extern byte statusByte;
extern TwoWire WireSlave;