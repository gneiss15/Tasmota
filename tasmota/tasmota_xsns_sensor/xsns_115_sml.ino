#ifdef USE_SML_X

#define XSNS_115	115	// = MAILBOX_CMD_IDX

//!#ifdef ESP32
//!  #ifdef USE_ESP32_SW_SERIAL
   #ifndef ESP32_SWS_BUFFER_SIZE
     #define ESP32_SWS_BUFFER_SIZE 256
   #endif
//!  #endif
//!#endif

#include <TasmotaSerial.h>

//###################################################################
// namespace
//###################################################################

namespace XSML
 {
  #define USE_PROGMEM
  #include "include/Crc.hpp"
  //Crc< uint16_t, 0x1021, 0xFFFF, true,  true,  0xFFFF > CRC16_X_25;  

//###################################################################
// Utils
//###################################################################
template <typename T>
  inline void Delete( T *& p )
   {
    delete p;
    p = nullptr;
   }

void Hexdump( uint8_t * sbuff, uint32_t slen )
 {
  char cbuff[ slen * 3 + 10 ];
  char *cp = cbuff;
  *cp++ = '>';
  *cp++ = ' ';
  for( uint32_t cnt = 0;  cnt < slen;  cnt ++ )
   {
    sprintf( cp, "%02x ", sbuff[ cnt ] );
    cp += 3;
   }
  AddLogData( LOG_LEVEL_INFO, cbuff );
 }

//###################################################################
// TSmlIoBase
//###################################################################
class TSmlIoBase : public Stream
 {
 public:
                   TSmlIoBase(void)                                  {}
  virtual          ~TSmlIoBase(void)                                 {}

  virtual void     end(void)                                         {}
  virtual int32_t  peek(void)                                        { return -1; }
  int              read(void) override                               { return -1; }
  size_t           write( uint8_t byte ) override                    { return 0; }
  int              available(void) override                          { return 0; }
  virtual void     flush(void) override                              {}
  virtual void     setRxBufferSize( size_t size )                    {}
  virtual void     updateBaudRate( uint32_t baud )                   {}

  virtual bool     begin( uint32_t speed, uint32_t smode,
                          int8_t rxPin, int8_t txPin,
                          int invert )                               = 0;

  //?using            Print::write;
 };

//###################################################################
// TSmlIoHwSerial
//###################################################################
class TSmlIoHwSerial : public TSmlIoBase
 {
 protected:
  uint32_t         uart_index;
  HardwareSerial * hws;
  int8_t           m_rx_pin;
  int8_t           m_tx_pin;
 public:
                   TSmlIoHwSerial( uint32_t uartIdx )                { uart_index = uartIdx; hws = nullptr; }
  virtual          ~TSmlIoHwSerial(void)                             { if( hws ) { hws->end(); delete( hws ); } }

  int32_t          peek(void) override                               { if (hws) return hws->peek(); return -1; }
  int              read(void) override                               { if (hws) return hws->read(); return -1; }
  size_t           write( uint8_t byte ) override                    { if (hws) { return hws->write(byte); } return 0; }
  int              available(void) override                          { if (hws) return hws->available(); return 0; }
  void             flush(void) override                              { if (hws) hws->flush(); }
  void             setRxBufferSize( size_t size ) override           { if (hws) hws->setRxBufferSize( size ); }
  void             updateBaudRate( uint32_t baud ) override          { if (hws) hws->updateBaudRate( baud ); }

  bool             begin( uint32_t speed, uint32_t smode,
                          int8_t rxPin, int8_t txPin,
                          int invert ) override;
 };

//###################################################################
// TSmlIoSwSerial
//###################################################################
class TRingBuffer
 {
 protected:
  uint8_t *        Buffer;
  uint8_t *        InPtr;
  uint8_t *        OutPtr;
  size_t           Size;
 protected:
 public:
                   TRingBuffer( size_t size )                        : Buffer( NULL ) { SetSize( size ); }
                   ~TRingBuffer(void)                                { Reset(); }
  void             Reset(void)                                       { Delete( Buffer ); Clear(); }
  void             SetSize( size_t size )                            { Reset(); Size = size; Buffer = new uint8_t[ Size ]; Clear(); }
  void             Clear(void)                                       { InPtr = OutPtr = Buffer; }
  int32_t          Peek(void)                                        { if( OutPtr == InPtr ) return -1; return *OutPtr; }
  int              Read(void)                                        { if( OutPtr == InPtr ) return -1; uint8_t ch = *OutPtr++; if( OutPtr >= Buffer + Size ) OutPtr = Buffer; return ch; }
  int              Available(void)                                   { int res = InPtr - OutPtr; if( res < 0 ) res += Size; return res; }
  void             Put( uint8_t byte )                               { uint8_t * p = InPtr + 1; if( p >= Buffer + Size ) p = Buffer; if( p != OutPtr ) { *InPtr = byte; InPtr = p; } }
 };
class TSmlIoSwSerial : public TSmlIoBase
 {
 private:
  uint32_t         CyclesPerBit;
  uint32_t         SBitStartCycle;
  uint32_t         ActBitNo;
  uint16_t         CurrentByte;
  static 
    void IRAM_ATTR staticIrqRead( void * self )                      { ( (TSmlIoSwSerial*)self )->irqRead(); };
  void             irqRead(void);
 protected:
  TRingBuffer      MRingBuffer;
  int8_t           m_rx_pin;
  int8_t           m_tx_pin;
 protected:
  void             Setbaud( uint32_t baud )                          { CyclesPerBit = ESP.getCpuFreqMHz() * 1000000 / baud; }
 public:
                   TSmlIoSwSerial(void)                              : MRingBuffer( ESP32_SWS_BUFFER_SIZE ) {}
  virtual          ~TSmlIoSwSerial(void)                             { detachInterrupt( m_rx_pin ); }

  void             end(void) override                                { MRingBuffer.Reset(); }
  int32_t          peek(void) override                               { return MRingBuffer.Peek(); }
  int              read(void) override                               { return MRingBuffer.Read(); }
  size_t           write( uint8_t byte ) override                    { return 0; }
  int              available(void) override                          { return MRingBuffer.Available(); }
  void             flush(void) override                              { MRingBuffer.Clear(); }
  void             setRxBufferSize( uint32_t size )                  { MRingBuffer.SetSize( size ); }
  void             updateBaudRate( uint32_t baud ) override          { Setbaud( baud ); }

  bool             begin( uint32_t speed, uint32_t smode,
                          int8_t rxPin, int8_t txPin,
                          int invert ) override;
 };

//###################################################################
// TSmlIoSerial
//###################################################################

//###################################################################
// TSmlIoHwSerial
//###################################################################
bool TSmlIoHwSerial::begin( uint32_t speed, uint32_t smode,
                            int8_t rxPin, int8_t txPin,
                            int invert )
 {
  m_rx_pin = rxPin;
  m_tx_pin = txPin;
  if( hws )
   { 
    hws->end();
    delete( hws );
   }
  hws = new HardwareSerial( uart_index );
  if( hws )
    hws->begin( speed, smode, m_rx_pin, m_tx_pin, invert );

  return true;
 }

//###################################################################
// TSmlIoSwSerial
//###################################################################
bool TSmlIoSwSerial::begin( uint32_t speed, uint32_t smode,
                            int8_t rxPin, int8_t txPin,
                            int invert )
 {
  m_rx_pin = rxPin;
  m_tx_pin = txPin;

  Setbaud( speed );
  pinMode( m_rx_pin, INPUT_PULLUP );
  ActBitNo = 0;
  MRingBuffer.Clear();
  attachInterruptArg( m_rx_pin, staticIrqRead, this, CHANGE );

  return true;
 }


#if 0
// First byte is allways broken !
// no wait mode only 8N1 (or 7X1, obis only, ignoring parity)
void IRAM_ATTR TSmlIoSwSerial::irqRead(void)
 {
  #define STOP_BIT 9

  uint32_t level = digitalRead( m_rx_pin );

  // CurrentByte: s76543210S, S: Start-Bit (not stored), s: Stop-Bit
  // ActBitNo:    9876543210

  if( !ActBitNo )                                                    // Not in Byte
   {
    if( level )                                                      // Pos Edge -> Ignore
      return;
    SBitStartCycle = ESP.getCycleCount() - ( CyclesPerBit / 4 );     // Neg Edge -> Start-Condition
    CurrentByte = 0;
    ActBitNo = 1;
    return;
   }

  // calc Bit-Cells since Start-Condition Start:0, 0:1,... 7:8, Stop: 9
  uint32_t bitNo = ( ESP.getCycleCount() - SBitStartCycle ) / CyclesPerBit;

  if( bitNo < 1 )                                                   // Edge earlier than one Bit-Cell (inside Start-Bit) -> Start over
   {
    ActBitNo = 0;
    return;
   }

  if( !level && bitNo > STOP_BIT )                                  // At Start bit of next byte, store and restart
   {
    //for( uint32_t i = ActBitNo - 1;  i <= STOP_BIT - 1;  i++ )
    //  CurrentByte |= ( 1 << i );
    CurrentByte |= 0xFF << ( ActBitNo - 1 );                        // Neg Edge -> earlier Bits were 1
    MRingBuffer.Put( CurrentByte );

    SBitStartCycle = ESP.getCycleCount() - ( CyclesPerBit / 4 );
    CurrentByte = 0;
    ActBitNo = 1;
    return;
   }

  if( bitNo >= STOP_BIT )                                           // At Stop-Bit -> wait next Start-Condition
   {
    // last bits were 0
    MRingBuffer.Put( CurrentByte );
    CurrentByte = 0;
    ActBitNo = 0;
    return;
   }

  // shift in
  if( !level )                                                       // Neg Edge -> earlier Bits were 1
    for( uint32_t i = ActBitNo - 1;  i < bitNo - 1;  i++ )
      CurrentByte |= ( 1 << i );
  ActBitNo = bitNo;

  #undef STOP_BIT
 }
#endif

#if 1
// First byte is allways broken !
// no wait mode only 8N1 (or 7X1, obis only, ignoring parity)
void IRAM_ATTR TSmlIoSwSerial::irqRead(void)
 {
  #define STOP_BIT 9

  uint32_t level = digitalRead( m_rx_pin );

  // CurrentByte: s76543210S, S: Start-Bit (not stored), s: Stop-Bit
  // ActBitNo:    9876543210

  if( !ActBitNo )
   {
    if( level )
      return;
    // start condition
    SBitStartCycle = ESP.getCycleCount() - ( CyclesPerBit / 4 );
    CurrentByte = 0;
    ActBitNo = 1;
    return;
   }

  uint32_t bitNo = ( ESP.getCycleCount() - SBitStartCycle ) / CyclesPerBit;

  if( !level && bitNo > STOP_BIT )
   {
    // start bit of next byte, store  and restart
    // leave irq at change
    for( uint32_t i = ActBitNo - 1;  i <= STOP_BIT - 1;  i++ )
      CurrentByte |= ( 1 << i );
    MRingBuffer.Put( CurrentByte );

    SBitStartCycle = ESP.getCycleCount() - ( CyclesPerBit / 4 );
    CurrentByte = 0;
    ActBitNo = 1;
    return;
   }

  if( bitNo >= STOP_BIT )
   {
    // last bits were 0
    MRingBuffer.Put( CurrentByte );
    CurrentByte = 0;
    ActBitNo = 0;
    return;
   }

  // shift in
  if( !level )
    for( uint32_t i = ActBitNo - 1;  i < bitNo - 1;  i++ )
      CurrentByte |= ( 1 << i );
  ActBitNo = bitNo;

  #undef STOP_BIT
 }
#endif

#if 0
// no wait mode only 8N1 (or 7X1, obis only, ignoring parity)
void IRAM_ATTR TSmlIoSwSerial::irqRead(void)
 {
  #define LASTBIT 8

  uint32_t level = digitalRead( m_rx_pin );
  
  // CurrentByte: s76543210[S], S: Start-Bit, not stored, s: Stop-Bit
  if( ActBitNo < 0 )                                                 // Not in Byte
   {
    if( level )                                                      // Pos Edge -> Ignore
      return;
    CurrentByte = 0;                                                 // Neg Edge -> Start-Condition
    SBitStartCycle = ESP.getCycleCount() - ( CyclesPerBit / 4 );
    ActBitNo = 0;
    return;
   }
  
  // calc Bit-Cells since Start-Condition Start:0, 0:1,... 7:8, Stop: 9
  uint32_t bitNo = ( ESP.getCycleCount() - SBitStartCycle ) / CyclesPerBit;
  
  if( bitNo == 0 )                                                   // Edge earlier than one Bit-Cell (inside Start-Bit) -> Start over
   {
    ActBitNo = -1;
    return;
   }
  
  bitNo--;                                                           // bitNo 1..9 -> 0..8

  if( bitNo > LASTBIT )                                              // At least at Stop-Bit
   {
    if( !level )                                                     // Neg Edge -> earlier Bits were 1
      CurrentByte |= 0xFF << ActBitNo;
    MRingBuffer.Put( CurrentByte );
    
    CurrentByte = 0;
    if( !level )
     {                                                               // At Start-Bit of next Byte
      SBitStartCycle = ESP.getCycleCount() - ( CyclesPerBit / 4 );
      ActBitNo = 0;
     }
     else
      ActBitNo = -1;                                                 // At Stop-Bit -> wait next Start-Condition
    return;
   }
  
  // shift in
  if( !level )                                                       // Neg Edge -> earlier Bits were 1
    for( ;  ActBitNo < bitNo;  ActBitNo++ )
      CurrentByte |= ( 1 << ActBitNo );
   else
    ActBitNo = bitNo;

  #undef LASTBIT
 }
#endif

//###################################################################
// Globals
//###################################################################

TSmlIoBase * hwSerial;
TSmlIoBase * swSerial;

TSmlIoBase * serIn;
uint8_t ser_act_LED_pin = 255;

#if 0
struct XSML_GLOBS
 {
  bool ready;
  uint8_t sml_send_blocks;
  uint8_t sml_100ms_cnt;
  uint8_t sml_desc_cnt;
  uint8_t meters_used;
  uint8_t maxvars;
  uint8_t *meter_p;
  double *meter_vars;
  uint8_t *dvalid;
  double dvalues[MAX_DVARS];
  uint32_t dtimes[MAX_DVARS];
  char sml_start;
  uint8_t dump2log = 0;
  uint8_t ser_act_meter_num = 0;
  uint16_t sml_logindex;
  char *log_data;
	uint16_t logsize = SML_DUMP_SIZE;
#if defined(ED300L) || defined(AS2020) || defined(DTZ541) || defined(USE_SML_SPECOPT)
  uint8_t sml_status[MAX_METERS];
  uint8_t g_mindex;
#endif
#ifdef USE_SML_MEDIAN_FILTER
  struct SML_MEDIAN_FILTER *sml_mf;
#endif
	uint8_t *script_meter;
	struct METER_DESC *mp;
  uint8_t to_cnt;
 }
 xsml_globs;
#endif

//###################################################################
// Init
//###################################################################
void XSNS_115_Init(void)
 {
  hwSerial = new TSmlIoHwSerial( 2 );
  swSerial = new TSmlIoSwSerial();
  
               // speed, smode,      rx, tx, inv
  hwSerial->begin( 9600, SERIAL_8N1, 16, 17, 0 );
  swSerial->begin( 9600, SERIAL_8N1,  5, 18, 0 );
  
  serIn = swSerial;
 }

//###################################################################
// Loop
//###################################################################
void XSNS_115_Loop(void)
 {
  // poll for serial input
  //!if( ser_act_LED_pin != 255 )
  //!  digitalWrite( ser_act_LED_pin, serIn->available() && !digitalRead( ser_act_LED_pin ) ); // Invert LED, if queue is continuously full

  while( serIn->available() )
   {
    //sml_shift_in( meters, 0 );
    int b = serIn->read();
    AddLog( LOG_LEVEL_ERROR, PSTR( "XSNS_115_Loop: %02X '%c'" ), b, b );
   }
 }

//###################################################################
// Cmd
//###################################################################

bool XSNS_115_cmd(void)
 {
  bool serviced = true;
  if( XdrvMailbox.data_len > 0 )
   {
    char *cp = XdrvMailbox.data;
    if( *cp == 's' )
     {
      //AddLog( LOG_LEVEL_ERROR, PSTR( "AddLog XSNS_115_cmd: %s" ), cp );
      //TasConsole.printf( PSTR( "printf XSNS_115_cmd: %s\n" ), cp );
      while( *++cp )
       {
        hwSerial->write( *cp );
        //AddLog( LOG_LEVEL_ERROR, PSTR( "XSNS_115_CMD: %02X '%c'" ), *cp, *cp );
       }
      //ResponseTime_P( PSTR(",\"XSML\":{\"CMD\":\"s: %s\"}}"), XdrvMailbox.data );
     }
     else
      serviced = false;
   }
  return serviced;
 }

void XSNS_115_EverySec()
 {
  //hwSerial->write( 'X' );
 }

//###################################################################
// Interface
//###################################################################

//###################################################################
// namespace
//###################################################################
 }

bool Xsns115( uint32_t function )
 {
  bool result = false;
  //XSML::CRC16_X_25.Start();
  //XSML::CRC16_X_25.Next( 1 );
  
  switch( function )
   {
    case FUNC_INIT:
      XSML::XSNS_115_Init();
      break;
    case FUNC_COMMAND_SENSOR:
      if( XSNS_115 == XdrvMailbox.index )
        result = XSML::XSNS_115_cmd();
      break;
    case FUNC_LOOP:
      XSML::XSNS_115_Loop();
      break;
    case FUNC_EVERY_SECOND:
      XSML::XSNS_115_EverySec();
      break;
    default:
      break;
#if 0
    case FUNC_LOOP:
      if( bitRead( Settings->rule_enabled, 0 ) )
       {
        if( xsml_globs.ready )
         {
          SML_Counter_Poll();
          if( xsml_globs.dump2log )
            dump2log();
           else
            SML_Poll();
         }
       }
       break;
    case FUNC_EVERY_100_MSECOND:
      if( bitRead( Settings->rule_enabled, 0 ) )
        if( xsml_globs.ready )
          SML_Check_Send();
      break;
    case FUNC_EVERY_SECOND:
      if( bitRead( Settings->rule_enabled, 0 ) )
        if( xsml_globs.ready )
          SML_Counter_Poll_1s();
      break;
    case FUNC_JSON_APPEND:
      if( xsml_globs.ready )
       if( sml_options & SML_OPTIONS_JSON_ENABLE )
         SML_Show( 1 );
      break;
    #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        if( xsml_globs.ready )
          SML_Show(0);
      break;
    #endif  // USE_WEBSERVER
    case FUNC_SAVE_BEFORE_RESTART:
    case FUNC_SAVE_AT_MIDNIGHT:
      if( xsml_globs.ready )
        SML_CounterSaveState();
      break;
#endif
   }
  return result;
 }

#endif  // USE_XSML_M

