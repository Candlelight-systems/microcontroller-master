#include "pid.h"
/*
 * ssr.h
 *
 *  Created on: 14 Apr 2018
 *      Author: normanpellet
 */

#ifndef SSR_H_
#define SSR_H_


extern PID PID_SSR1;
extern PID PID_SSR2;
void SSRDisable( unsigned char chanId );
void SSREnable( unsigned char chanId );
void SSRSetValue( unsigned char chanId, float val );
void SSRSetValue( unsigned char chanId, uint32_t newValue );

#endif /* SSR_H_ */
