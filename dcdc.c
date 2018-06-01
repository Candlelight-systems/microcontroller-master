#define currentLSB 0.0005 // 1 mA per bit

struct dcdc {
  float voltage;
  float current;
  bool enabled;
  byte i2caddress;
  byte cspin;
  byte enpin;
};


struct dcdc* dcdcs[ 3 ] = {
  NULL,
  new dcdc,
  new dcdc
};


void setDCDCOutput( int channel, int value ) {

    SPI.begin();
    SPI.beginTransaction( SPISettings(10000, MSBFIRST, SPI_MODE0 ) );
    digitalWrite( dcdcs[ channel ]->cspin, LOW );
    SPI.transfer( 0b01000000 );
    SPI.transfer( 0b00001111 );
    digitalWrite( dcdcs[ channel ]->cspin, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( dcdcs[ channel ]->cspin, LOW );
    SPI.transfer( 0b00000000 );
    SPI.transfer( value );
    digitalWrite( dcdcs[ channel ]->cspin, HIGH );
    
    SPI.endTransaction();
    SPI.end();
}

byte enPinFromChannel( int channel ) {
    return dcdcs[ channel ]->enpin;
}


void enableDCDC( int channel ) {

    byte ENPin = enPinFromChannel( channel );

    if( ENPin == 0 ) {
      return;
    }

    dcdcs[ channel ]->enabled = true;
    digitalWrite( ENPin, HIGH );
}


void disableDCDC( int channel ) {

    byte ENPin = enPinFromChannel( channel );

    if( ENPin == 0 ) {
      return;
    }

    dcdcs[ channel ]->enabled = false;
    digitalWrite( ENPin, LOW );
}

void dcdc_setup() {

    
  dcdcs[1]->i2caddress = 0b1000001;
  dcdcs[2]->i2caddress = 0b1000000;
  
  dcdcs[1]->cspin = PIN_DCDC_CS_1;
  dcdcs[2]->cspin = PIN_DCDC_CS_2;
  
  dcdcs[1]->enpin = PIN_DCDC_EN_1;
  dcdcs[2]->enpin = PIN_DCDC_EN_2;


  for( int i = 1; i <= 2; i += 1 ) {
    digitalWrite( dcdcs[ i ]->enpin, LOW );
    digitalWrite( dcdcs[ i ]->cspin, HIGH );

    Wire.beginTransmission( dcdcs[ i ]->i2caddress ); // INA219
    Wire.write( 0x00 ); // Register 0 => configuration
    Wire.write( 0b00100111 ); // 32V bus, , 40mV FSR, 128 samples averaging (68ms), shunt and bus, continuous
    Wire.write( 0b11111111 ); 
    Wire.endTransmission();

    Wire.beginTransmission( dcdcs[ i ]->i2caddress ); // INA219
    Wire.write( 0x05 ); // Register 0 => configuration

    int16_t cal = ( int16_t ) ( 0.04096 / ( 0.01 * currentLSB ) );
    
    Wire.write( cal >> 8 ); 
    Wire.write( cal ); 
    Wire.endTransmission();
    
  }
  
}


void dcdc_read( byte channel ) {

  int b1, b2;

  

  Wire.beginTransmission( dcdcs[ channel ]->i2caddress );
  Wire.write( 0x01 ); // Read the current
  Wire.endTransmission();
  Wire.requestFrom( dcdcs[ channel ]->i2caddress, 2 ); // Expecting 2 byte
  b1 = Wire.read();
  b2 = Wire.read();


  
  Wire.beginTransmission( dcdcs[ channel ]->i2caddress );
  Wire.write( 0x04 ); // Read the current
  Wire.endTransmission();
  Wire.requestFrom( dcdcs[ channel ]->i2caddress, 2 ); // Expecting 2 byte
  b1 = Wire.read();
  b2 = Wire.read();
  float current =  ( (float) ( ( (b1 << 8 )  | b2 ) ) ) * currentLSB;


  

  
  Wire.beginTransmission( dcdcs[ channel ]->i2caddress );
  Wire.write( 0x02 ); // Read the voltage
  Wire.endTransmission();
  Wire.requestFrom( dcdcs[ channel ]->i2caddress, 2 ); // Expecting 2 byte
  b1 = Wire.read();
  b2 = Wire.read();
  float voltage = ( ( (float) ( ( b1 << 8 | b2 ) >> 3 & 0x1FFF ) ) * 0.004 );

  dcdcs[ channel ]->voltage = voltage;
  dcdcs[ channel ]->current = current;
}



scpi_error_t scpi_valuedcdc(struct scpi_parser_context* context, struct scpi_token* args)
{
  struct scpi_numeric value;
  int chanId = getChannel( args );

  value = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, -2.5, 2.5 );
  scpi_free_tokens(args);

  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  setDCDCOutput( chanId, value.value );
  return SCPI_SUCCESS;
}


scpi_error_t scpi_enabledcdc(struct scpi_parser_context* context, struct scpi_token* args)
{
  int chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  enableDCDC( chanId );
  return SCPI_SUCCESS;
}


scpi_error_t scpi_isenableddcdc(struct scpi_parser_context* context, struct scpi_token* args)
{
  int chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  SerialUSB.println( dcdcs[ chanId ]->enabled );
  return SCPI_SUCCESS;
}



scpi_error_t scpi_disabledcdc(struct scpi_parser_context* context, struct scpi_token* args)
{
  int chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  disableDCDC( chanId );
  return SCPI_SUCCESS;
}



scpi_error_t scpi_currentdcdc(struct scpi_parser_context* context, struct scpi_token* args)
{
  
  int chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  dcdc_read( chanId );
  printDouble( dcdcs[ chanId ]->current, 6 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}


scpi_error_t scpi_voltagedcdc(struct scpi_parser_context* context, struct scpi_token* args)
{
  int chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }
  dcdc_read( chanId );
  printDouble( dcdcs[ chanId ]->voltage, 6 );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}

