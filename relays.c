
byte relays_gen = 0b00000000;
byte relays_1_8 = 0b00000000;
byte relays_9_16 = 0b00000000;
byte relays_ext_1_8 = 0b00000000;
byte relays_ext_9_16 = 0b00000000;

void enableChannelGeneral( byte chanId ) {
  if ( chanId == 1 ) {
    relays_gen |= 0b00001000;
  } else if ( chanId == 2 ) {
    relays_gen |= 0b00000100;
  }
  updateChannels();
}

void disableChannelGeneral( byte chanId ) {
  if ( chanId == 1 ) {
    relays_gen &= ~0b00001000;
  } else if ( chanId == 2 ) {
    relays_gen &= ~0b00000100;
  }
  updateChannels();
}


scpi_error_t light_enable(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  lightEnable( chanId );
  return SCPI_SUCCESS;
}

void lightEnable( byte chanId ) {
  if ( chanId == 1 ) {
    relays_gen |= 0b00000010;
  } else if ( chanId == 2 ) {
    relays_gen |= 0b00000001;
  }

  updateChannels();
}

scpi_error_t light_disable(struct scpi_parser_context* context, struct scpi_token* args) {
  byte chanId = getChannel( args );
  scpi_free_tokens(args);
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  lightDisable( chanId );
  
  return SCPI_SUCCESS;
}

void lightDisable( byte chanId ) {
  if ( chanId == 1 ) {
    relays_gen &= ~0b00000010;
  } else if ( chanId == 2 ) {
    relays_gen &= ~0b00000001;
  }

  updateChannels();
}



void enableChannel( int chanId ) {
  if( chanId < 9 ) {
     relays_1_8 |= 1 << ( 8 - chanId );
  } else {
     relays_9_16 |= 1 << ( 16 - chanId );
  }
  updateChannels();

}

void disableChannel( int chanId ) {
  if( chanId < 9 ) {
     relays_1_8 &= ~( 1 << ( 8 - chanId ) );
  } else {
     relays_9_16 &= ~( 1 << ( 16 - chanId ) );
  }
  updateChannels();
}


void enableChannelExternal( int chanId ) {
  if( chanId < 9 ) {
     relays_ext_1_8 |= 1 << ( chanId - 1 );//1 << ( 8 - chanId );
  } else {
     relays_ext_9_16 |= 1 << ( chanId - 9 );//1 << ( 16 - chanId );
  }
  updateChannels();

}

void disableChannelExternal( int chanId ) {
  if( chanId < 9 ) {
     relays_ext_1_8 &= ~( 1 << ( chanId - 1 ) );
  } else {
     relays_ext_9_16 &= ~( 1 << ( chanId - 9 ) );
  }
  updateChannels();
}

void resetChannels() {
   relays_gen = 0x00;
   relays_1_8 = 0x00;
   relays_9_16 = 0x00;
   updateChannels();
}

void updateChannels() {

  SPI.begin();
  SPI.beginTransaction( SPISettings( 10000, MSBFIRST, SPI_MODE0 ) );


  digitalWrite( PIN_RELAY_CS, HIGH );

  // External relay driver (for muxing)
  SPI.transfer( relays_ext_9_16 );
  SPI.transfer( relays_ext_1_8 );

  // Internal relay driver (for enabling / disabling channels)
  SPI.transfer( relays_9_16 );
  SPI.transfer( relays_1_8 );

  // General purpose relays
  SPI.transfer( relays_gen );

  delayMicroseconds( 10 );
  digitalWrite( PIN_RELAY_RCK, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( PIN_RELAY_RCK, LOW );
  digitalWrite( PIN_RELAY_CS, LOW );

SerialUSB.println( relays_ext_1_8, BIN );
SerialUSB.println( relays_ext_9_16, BIN );
  SPI.endTransaction();
  SPI.end();
}


void disableChannels() {

  //digitalWrite( PIN_RELAY_CS, HIGH );

  SPI.begin();
  SPI.beginTransaction( SPISettings(1000000, MSBFIRST, SPI_MODE1 ) );

  delay( 1 );
  SPI.transfer16( 0x00 );
  SPI.transfer16( 0x00 );
  SPI.transfer16( 0x00 );

  digitalWrite( PIN_RELAY_RCK, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( PIN_RELAY_RCK, LOW );

  digitalWrite( PIN_RELAY_CS, LOW );
  SerialUSB.println( relays_gen );
  SPI.endTransaction();
  SPI.end();
  
}



