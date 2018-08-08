

void SRAM_STORE_IV( uint8_t chanId, uint16_t& address, int32_t voltage, int32_t current, int8_t pgaVal );
void SRAM_INIT();
int32_t SRAM_READ( uint8_t numBytes );
void SRAM_READ_INIT( uint8_t chanId, uint16_t address );
void SRAM_READ_TERMINATE( );