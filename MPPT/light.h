/*
 * light.h
 *
 *  Created on: 14 Apr 2018
 *      Author: normanpellet
 */

#ifndef LIGHT_H_
#define LIGHT_H_

float measurePD( unsigned char chanId );
float measurePDSun( unsigned char chanId );
int32_t measurePDCode( unsigned char chanId );

#include <Arduino.h>
#include "MPPT.h"
#include "scpiparser.h"

bool light_isenabled( byte chanId );
bool light_isautomatic( byte chanId );
void light_setPWM( byte chanId, uint32_t pwm );

void light_check( byte chanId, bool force );
void lightSetSetpoint( byte chanId, float setPoint );
void lightSetScaling( byte chanId, float scaling );

#endif /* LIGHT_H_ */
