#include <Arduino.h>

/*
 * slave.h
 *
 *  Created on: 14 Apr 2018
 *      Author: normanpellet
 */

#ifndef SLAVE_H_
#define SLAVE_H_

void muxI2C( byte id );
int16_t slave_readADS1015( byte address, byte channel );
//float slave_readVEML6070( );
void aquireHumiditySensor( byte address );

#endif /* SLAVE_H_ */
