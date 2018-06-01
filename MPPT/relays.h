/*
 * relays.h
 *
 *  Created on: 14 Apr 2018
 *      Author: normanpellet
 */

#ifndef RELAYS_H_
#define RELAYS_H_

void enableChannelGeneral( byte chanId );
void disableChannelGeneral( byte chanId );
void lightEnable( byte chanId );
void lightDisable( byte chanId );
void enableChannel( int chanId );
void disableChannel( int chanId );
void enableChannelExternal( int chanId );
void disableChannelExternal( int chanId );
void resetChannels();
void updateChannels();
void disableChannels();
uint8_t channelEnabled( int chanId );

#endif /* RELAYS_H_ */
