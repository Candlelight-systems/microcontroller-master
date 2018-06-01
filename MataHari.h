
#include <Arduino.h>

#define PIN_POWER 4
#define PIN_DCDC_CS_1 42
#define PIN_DCDC_CS_2 14
#define PIN_DCDC_EN_1 18
#define PIN_DCDC_EN_2 17
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

#define PIN_LIGHT1_MODE_DEC 2 
#define PIN_LIGHT2_MODE_DEC 5

#define I2C_ADDR_SLAVE 8

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

