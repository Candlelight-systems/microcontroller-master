void sendPGA( uint8_t toSend );
uint8_t readPGA();

void PGA_SET_GAIN( uint8_t gainValue );

void PGA_MUX_PD( );

void PGA_MUX_MAIN( );

void PGA_WRITE_REGISTER( uint8_t registerNumber, uint8_t newValue );

uint8_t PGA_READ_REGISTER( uint8_t registerNumber );

uint8_t PGA_CS_CONTROL( );
void PGA_DISABLE();

void PGA_ENABLE();

void ADC_WAKEUP();

void ADC_SLEEP();

void ADC_RESET();

int32_t ADC_READ_ONCE();

void sendADC( byte toSend );
byte readADC();
byte readADC( byte value );
void ADC_WRITE_COMMAND( byte command );
void ADC_WRITE_REGISTER( byte registerNumber, byte newValue );
byte ADC_READ_REGISTER( byte registerNumber );
void ADC_ENABLE();
void ADC_WRITE_CONFIG();
void ADC_SET_SPEED( byte speed );
void ADC_DISABLE();
void ADC_PINREADY_CHANGE();
void ADC_MONITOR_PINREADY();
void ADCSetup();
byte ADC_OUT_OF_RANGE();

int32_t ADCReadVoltageCode( int chanId, byte pgaValue );
int32_t ADCReadVoltageCode( int chanId );
int32_t ADCReadCurrentCode( int chanId, byte pgaValue );
int32_t ADCReadPDCode( );



void sendPGA( byte toSend );
byte readPGA();
void PGA_SET_GAIN( byte gainValue );
void PGA_MUX_PD( );
void PGA_MUX_MAIN(  );
void PGA_WRITE_REGISTER( byte registerNumber, byte newValue );
byte PGA_READ_REGISTER( byte registerNumber );
byte PGA_CS_CONTROL( );
void PGA_DISABLE();
void PGA_ENABLE();