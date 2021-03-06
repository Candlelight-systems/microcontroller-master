
void muxI2C( byte id ) {

  WireSlave.beginTransmission( 0b110000 );
  if( id == 1 ) {  
    WireSlave.write( 0x00000001 );
  } else {
    WireSlave.write( 0x00000010 );
  }
  WireSlave.endTransmission();
}


// ENVIRONMENT:HUMIDITY?:SLAVE1 2
//       0         1      2    3 
scpi_error_t get_humidity_box(struct scpi_parser_context* context, struct scpi_token* args) { 
 
  struct scpi_numeric I2C_SLAVE = scpi_parse_numeric( args->next->next->next->value, args->next->next->next->length, 0, 0, 255 ); 
  scpi_free_tokens(args);
  muxI2C( (int) ( args->next->next->value[ 5 ] - '0' ) );
  aquireHumiditySensor( I2C_SLAVE.value ); 
}


// ENVIRONMENT:TEMPBASE?:CH1:SLAVE1 2
//       0         1      2    3   4
scpi_error_t get_temperature_base(struct scpi_parser_context* context, struct scpi_token* args) {

  struct scpi_numeric I2C_SLAVE = scpi_parse_numeric( args->next->next->next->next->value, args->next->next->next->next->length, 0, 0, 255 );
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
  
  struct scpi_numeric I2C_SLAVE = scpi_parse_numeric( args->next->next->next->next->value, args->next->next->next->next->length, 0, 0, 255 );
  byte channel = getChannel( args );
  scpi_free_tokens(args);
  muxI2C( (int) ( args->next->next->next->value[ 5 ] - '0' ) );
  int16_t val = slave_readADS1015( (byte) I2C_SLAVE.value, (byte) channel );
  SerialUSB.write( val >> 8 );
  SerialUSB.write( val );
  SerialUSB.println("");
  return SCPI_SUCCESS;
}


int16_t slave_readADS1015( byte address, byte channel ) {

  byte b1, b2;
  byte channelCfg;
  
  switch( channel ) {

    case 0:
      channelCfg = 0b01000000;
    break;
    
    case 1:
      channelCfg = 0b01010000;
    break;
    
    case 2:
      channelCfg = 0b01100000;
    break;
    
    case 3:
      channelCfg = 0b01110000;
    break;
  }

  WireSlave.beginTransmission( address );
  WireSlave.write( 0x00 | 0b00000001 ); // Write to the configuration register
  WireSlave.write( 0b10000101 | channelCfg ); // PGA at 2.048V, single shot, at a specific channel. Executes a conversion
  WireSlave.write( 0b10000111 ); // 1.6kSPS, disable comparator
  WireSlave.endTransmission();
  delay( 2 ); // Wait for the conversion to complete
  WireSlave.beginTransmission( address );
  WireSlave.write( 0x00 | 0b00000000 ); // Points to the data register
  WireSlave.endTransmission();

  WireSlave.requestFrom( address, 2 );
  b1 = WireSlave.read();
  b2 = WireSlave.read();
  return ( ( b1 << 8 | b2 ) );   // The data is in the first 12 bits. We don't shift them to the right because we want to keep the signing. The software should divide by 32 (2^4) to get the real code value
}




void aquireHumiditySensor( byte address ) {

    WireSlave.beginTransmission( address );
    WireSlave.write( 0x02 ); // Config register
    WireSlave.write( B00110000 );
    WireSlave.write( 0 );
    WireSlave.endTransmission();

    WireSlave.beginTransmission( address );
    WireSlave.write( 0x00 ); // Temperature register
    WireSlave.endTransmission();
    delay( 20 );
  
    WireSlave.requestFrom( address, 4 );
    int temperature1 = WireSlave.read();
    int temperature2 = WireSlave.read();
    
    int temperature = ( temperature1 << 8 | temperature2 );

    printDouble( ( (float) temperature ) / ( 65543 ) * 165.0 - 40.0, 6 );  
    SerialUSB.println("");
        
    int humidity1 = WireSlave.read();
    int humidity2 = WireSlave.read();
    
    int humidity = ( humidity1 << 8 | humidity2 );

    printDouble( ( (float)  ( humidity ) / ( 65543 ) ), 4 );
    SerialUSB.println("");
}

