


scpi_error_t ssr_on(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 ) {
    digitalWrite( PIN_SSR_1, HIGH );  
  } else if( chanId == 2 ) {
    digitalWrite( PIN_SSR_2, HIGH );
  }
  
  return SCPI_SUCCESS;
}

scpi_error_t ssr_off(struct scpi_parser_context* context, struct scpi_token* args) {

  byte chanId = getChannel( args );
  if ( chanId == -1 ) {
    return SCPI_SUCCESS;
  }

  if( chanId == 1 ) {
    digitalWrite( PIN_SSR_1, LOW );  
  } else if( chanId == 2 ) {
    digitalWrite( PIN_SSR_2, LOW );
  }
  
  return SCPI_SUCCESS;
}

