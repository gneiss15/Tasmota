/*
  Crc.hpp - universal CRC Class 

  Copyright (C) 2023  Günter Neiß

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Usage:
  
  This Template-Class can compute CRC8,  CRC16 and CRC32
  The LookupTables are generated at compile time using Variadic Templates,
  which will result in the most effective code.

  To use it, instantiate a Class with the parameter for the CRC-Type You wish to use.
  If You wish to place the Table(s) into PROGMEM, define USE_PROGMEM prior to include this file.

  Some examples:

  Crc< uint8_t, 0x07, 0x00, false, false, 0x00 > CRC_8;
  
  Crc< uint16_t, 0x1021, 0x1D0F, false, false, 0x0000 > CRC16_CCITT;
  Crc< uint16_t, 0x1021, 0xFFFF, true,  true,  0xFFFF > CRC16_X_25;
  Crc< uint16_t, 0x8005, 0x0000, true,  true,  0x0000 > CRC16_ARC;
  
  Crc< uint32_t, 0x04C11DB7, 0xFFFFFFFF, true, true, 0xFFFFFFFF > CRC_32;
  Crc< uint32_t, 0x814141AB, 0x00000000, false, false, 0x00000000 > CRC_32Q;

  The nessesary parameters may be found on https://crccalc.com/

  You may calculate the CRC byte by byte using Start, Next and Get, or 'en block' using Calc.
*/

#ifndef __CRC_HPP__
#define __CRC_HPP__

#if ! defined( __cplusplus ) | __cplusplus < 201703L
  #error This file must be compiled in C++ Mode with (at least) C++17 standard (set -std=gnu++17)
#endif

#include <cstdint>
#include <cstddef>

#ifdef USE_PROGMEM
  #define PROGMEM_ PROGMEM
 #else
  #define PROGMEM_
#endif

// Function to Bit-Revert
template< typename VALUETYPE >
  static constexpr VALUETYPE Revert( VALUETYPE in, uint8_t bits )
   {
    VALUETYPE res = 0;
    
    for( int i = 0;  i < bits;  i++ )
     {
      res = ( res << 1 ) | ( in & 1 );
      in >>= 1;
     }
    return res;
   }

// Function to calculate a single table entry
template< typename VALUETYPE >
  static constexpr VALUETYPE TableValue( VALUETYPE crc, VALUETYPE poly, bool revIn )
   {
    uint8_t NoOfBits = sizeof( VALUETYPE ) * 8;
    VALUETYPE TopBit = 1 << ( NoOfBits - 1 );
    VALUETYPE Mask = ( TopBit - 1 ) << 1 | 1;
    
    if( revIn )
      crc = Revert< VALUETYPE >( crc, 8 );
    crc <<= NoOfBits - 8;
    for( int i = 0;  i < 8;  i++ )
      if( crc & TopBit )
        crc = ( crc << 1 ) ^ poly;
      else
        crc <<= 1;
    crc &= Mask;
    if( revIn )
      crc = Revert< VALUETYPE >( crc, NoOfBits );
    return crc & Mask;
   }

// Variadic template for a recursive helper struct.
template< typename VALUETYPE, VALUETYPE poly, bool revIn, int N = 0, VALUETYPE ...D >
  struct CrcLookupTableHelper : CrcLookupTableHelper< VALUETYPE, poly, revIn, N + 1, D..., TableValue< VALUETYPE >( N, poly, revIn )> {};

// Specialization of the template to end the recursion when the table size reaches 256.
template< typename VALUETYPE, VALUETYPE poly, bool revIn, VALUETYPE ...D >
  struct CrcLookupTableHelper< VALUETYPE, poly, revIn, 256, D... >
   { static constexpr VALUETYPE PROGMEM_ CrcLookupTable[ 256 ] = { D... }; };

template< typename VALUETYPE, VALUETYPE poly, VALUETYPE init, bool revIn, bool revOut, VALUETYPE XorOut >
  class Crc
   {
    static constexpr uint8_t   NoOfBits                                = sizeof( VALUETYPE ) * 8;
    static constexpr uint8_t   ShiftBits                               = NoOfBits - 8;
    static constexpr VALUETYPE Mask                                    = ( (VALUETYPE)( 1 << ( sizeof( VALUETYPE ) * 8 - 1 ) ) - 1 ) << 1 | 1;

   private:
    VALUETYPE        crc;

   public:
    void             Start(void)                                       { crc = init; }

    void             Next( uint8_t byte )                              {
                                                                        if( NoOfBits >= 16 )
                                                                          if( revIn )
                                                                            crc = ( crc >> 8 ) ^ CrcLookupTableHelper< VALUETYPE, poly, revIn >::CrcLookupTable[ ( crc & 0xFF ) ^ byte ];
                                                                           else
                                                                            crc = ( crc << 8 ) ^ CrcLookupTableHelper< VALUETYPE, poly, revIn >::CrcLookupTable[ ( ( crc >> ShiftBits ) & 0xFF ) ^ byte ];
                                                                         else
                                                                           crc = CrcLookupTableHelper< VALUETYPE, poly, revIn >::CrcLookupTable[ crc ^ byte ];
                                                                       }
    
    VALUETYPE        Get(void)                                         { crc &= Mask; if( revOut != revIn ) crc = Revert< VALUETYPE >( crc, NoOfBits ); return ( crc ^ XorOut ) & Mask; }
  
    VALUETYPE        Calc( uint8_t * bytes, size_t cnt )               { Start(); for( uint8_t * p = bytes;  p < bytes + cnt;  p++ ) Next( *p ); return Get(); }
   };

#endif


