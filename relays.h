
#include <Arduino.h>

#ifdef __cplusplus

  extern "C" {
  
#endif

void enableChannel( int chanId );
void disableChannel( int chanId );
void updateChannels();
void disableChannel();

#ifdef __cplusplus
  }
#endif

