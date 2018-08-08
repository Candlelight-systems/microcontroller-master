/*
 * light_expander.h
 *
 *  Created on: 8 Jun 2018
 *      Author: normanpellet
 */

#include "scpiparser.h"
/*
 * light_expander.cpp
 *
 *  Created on: 8 Jun 2018
 *      Author: normanpellet
 */


// PCA9685
// https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
void light_expander_init();
scpi_error_t light_expander_setPWM( struct scpi_parser_context* context, struct scpi_token* args);
void light_expander_updatePWM();
void lightExpander_switchUpdate();
scpi_error_t light_expander_enable( struct scpi_parser_context* context, struct scpi_token* args );
scpi_error_t light_expander_disable( struct scpi_parser_context* context, struct scpi_token* args );
scpi_error_t light_expander_isEnabled( struct scpi_parser_context* context, struct scpi_token* args );

