  
  float getVoltageOffset( byte chanId );
  float getVoltageGain( byte chanId );
  
  float getDacVoltageOffset( byte chanId );
  float getDacVoltageGain( byte chanId );
float getCurrentGain( byte chanId, int pgaVal );
float* getCurrentOffset( byte chanId, int pgaVal );
float calibrationPD( int32_t codeVal, int pdnum );

