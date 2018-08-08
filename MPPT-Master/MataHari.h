#define PIN_LED_INDICATOR 14
#define PIN_REF_EN 42

#define PIN_MUX0 8
#define PIN_MUX1 17
#define PIN_MUX2 18
#define PIN_MUX3 41
#define PIN_MUXEN1 19
#define PIN_MUXEN2 25

#define PIN_PD_SWITCH 16

#define PIN_DAC_CS_1 6
#define PIN_DAC_CS_2 7
#define PIN_DAC_RESET 12

#define PIN_PGA_CS 5
#define PIN_PGA_ERR 2
#define PIN_PGA_CURRENT_BUFFER 38
#define PIN_ADC_DRDY 10

#define ADC_CFG0_ADDR 0x00
#define ADC_CFG1_ADDR 0x01
#define ADC_CFG2_ADDR 0x02
#define ADC_OFC0_ADDR 0x03
#define ADC_OFC1_ADDR 0x04
#define ADC_OFC2_ADDR 0x05
#define ADC_FSC0_ADDR 0x06
#define ADC_FSC1_ADDR 0x07
#define ADC_FSC2_ADDR 0x08

#define ADC_RATE_10SPS B00000000
#define ADC_RATE_17SPS B00000001
#define ADC_RATE_50SPS B00000010
#define ADC_RATE_60SPS B00000011
#define ADC_RATE_400SPS B00000100
#define ADC_RATE_1200SPS B00000101
#define ADC_RATE_3600SPS B00000110
#define ADC_RATE_14400SPS B00000111


extern bool i2cLease;
extern volatile bool adcError;
extern bool SRAM_READ_OPEN;

int iv_number();
void trackChannel( byte chanId );
void outputDouble( float val );
void outputDouble( float val, byte *buffer, unsigned char index  );
void outputInt32( int32_t val, byte *buffer, unsigned char index  );
void outputFloat( float f );

void getTrackData( byte channel );

void getCurrent( byte channel );
int32_t getCurrentCode( byte channel );
int32_t getVoltageCode( byte channel );

void getVoltage( byte channel );
void outputInt32( int32_t val );

void flash( int number );
void getPD1();
void getPD2();

void setVoltageCode( byte chanId, uint16_t code );
void iv( byte chanId );
void findVoc( byte chanId );
void findVoc( byte chanId, bool synchronous );
void findJsc( byte chanId );

void resetChanData( byte chanId );
void setVoltage( byte chanId, float voltage );
void setVoltage( byte chanId, float voltage, bool assignToChannel );
void setupAll();
void trackChannelPO( byte channel );
void trackChannelVoc( byte channel );
void trackChannelVoc( byte channel, uint8_t stepSize );
void trackChannelJsc( byte channel );
void trackChannelCurrent( byte channel, float current );
void trackChannelCurrent( byte channel, float current, uint8_t stepSize );
float getDacVoltageFromCode( byte chanId, uint16_t code );
float getVoltageFromCode( byte channel, int32_t code );
float getCurrentFromCode( byte channel, int32_t code );
float getCurrentFromCode( byte channel, int32_t code, int8_t pgaVal );
bool findVocSubroutine( uint8_t chanId,uint16_t counter, bool synchronous );
void ivNext( struct channel* );
void ivNext( struct channel*, float from, float to );

int32_t measurePD( byte PD );
