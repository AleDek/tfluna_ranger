/* File Name: TFLI2C.cpp
 * Developer: Bud Ryerson
 * Date:      10 JUL 2021
 * Version:   0.1.1 - Fixed some typos in comments.
              Added a `p_` prefix to some pointer variables.
              Changed some register addresses from hard to symbolic.
              Changed TFL_DEFAULT_ADDR and TFL_DEFAULT_FPS
              to TFL_DEF_ADR and TFL_DEF_FPS in the header file.
              0.2.0 - Corrected (reversed) Enable/Disable commands
 * Described: Arduino Library for the Benewake TF-Luna Lidar sensor
              configured for the I2C interface
 *
 * Default settings for the TF-Luna are:
 *    0x10  -  default slave device I2C address `TFL_DEF_ADR`
 *    100Hz  - default data frame-rate `TFL_DEF_FPS`
 *
 *  There are is one important function: 'getData'.
 *  `getData( dist, flux, temp, addr)`
      Reads the disance measured, return signal strength and chip temperature.
      - dist : unsigned integer : distance measured by the device, in cm.
      - flux : unsigned integer : signal strength, quality or confidence
               If flux value too low, an error will occur.
      - temp : unsigned integer : temperature of the chip in 0.01 degrees C
      - addr : unsigned byte : address of slave device.
      Returns true, if no error occurred.
         If false, error is defined by a status code
         that can be displayed using 'printFrame()' function.

 * NOTE : If you only want to read distance, use getData( dist, addr)
 *
 *  There are several explicit commands
 */

#include "i2clink.h"

#include "TFLI2C.h"        //  TFLI2C library header
// #include <Wire.h>          //  Arduino I2C/Two-Wire Library

// Constructor/Destructor
TFLI2C::TFLI2C(I2cLink *pWire){
    _pWire = pWire;
}
TFLI2C::~TFLI2C(){}

bool TFLI2C::initI2c(std::string device){
    return _pWire->OpenBus(device.c_str());
}

bool TFLI2C::scan( uint8_t addr)
{   
    return _pWire->Begin(addr);
    // _pWire->beginTransmission(_addr);
    // if (_pWire->endTransmission() == 0){
    //     return true;
    // }
    // return false;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - -
//             GET DATA FROM THE DEVICE
// - - - - - - - - - - - - - - - - - - - - - - - - - -
bool TFLI2C::getData( int16_t &dist, int16_t &flux, int16_t &temp, uint8_t addr)
{
    tfStatus = TFL_READY;    // clear status of any error condition

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Use the `Wire` function `readReg` to fill the six byte
    // `dataArray` from the contiguous sequence of registers `TFL_DIST_LO`
    // to `TFL_TEMP_HI` that declared in the header file 'TFLI2C.h`.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (uint8_t reg = TFL_DIST_LO; reg <= TFL_TEMP_HI; reg++)
    {
      if( !readReg( reg, addr)) return false;
          else dataArray[ reg] = regReply;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Shift data from read array into the three variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    dist = dataArray[ 0] + ( dataArray[ 1] << 8);
    flux = dataArray[ 2] + ( dataArray[ 3] << 8);
    temp = dataArray[ 4] + ( dataArray[ 5] << 8);

/*
    // Convert temperature from hundredths
    // of a degree to a whole number
    temp = int16_t( temp / 100);
    // Then convert Celsius to degrees Fahrenheit
    temp = uint8_t( temp * 9 / 5) + 32;
*/

    // - - Evaluate Abnormal Data Values - -
    // Signal strength <= 100
    // if( flux < (int16_t)100)
    // {
    //   tfStatus = TFL_WEAK;
    //   return false;
    // }
    // // Signal Strength saturation
    // else if( flux == (int16_t)0xFFFF)
    // {
    //   tfStatus = TFL_STRONG;
    //   return false;
    // }
    // else
    // {
    //   tfStatus = TFL_READY;
    //   return true;
    // }
    return true;
}

// Get Data short version
bool TFLI2C::getData( int16_t &dist, uint8_t addr)
{
  static int16_t flux, temp;
  return getData( dist, flux, temp, addr);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - -
//              EXPLICIT COMMANDS
// - - - - - - - - - - - - - - - - - - - - - - - - - -

//  = =  GET DEVICE TIME (in milliseconds) = = =
//  Pass back time as an unsigned 16-bit variable
bool TFLI2C::Get_Time( uint16_t &tim, uint8_t adr)
{
    // Recast the address of the unsigned integer `tim`
    // as a pointer to an unsigned byte `p_tim`...
    uint8_t * p_tim = (uint8_t *) &tim;

    // ... then address the pointer as an array.
    if( !readReg( TFL_TICK_LO, adr)) return false;
        else p_tim[ 0] = regReply;  // Read into `tim` array
    if( !readReg( TFL_TICK_HI, adr)) return false;
        else p_tim[ 1] = regReply;  // Read into `tim` array
    return true;
}

//  = =  GET PRODUCTION CODE (Serial Number) = = =
// When you pass an array as a parameter to a function
// it decays into a pointer to the first element of the array.
// The 14 byte array variable `tfCode` declared in the example
// sketch decays to the array pointer `p_cod`.
bool TFLI2C::Get_Prod_Code( uint8_t * p_cod, uint8_t adr)
{
   for (uint8_t i = 0; i < 14; ++i)
    {
      if( !readReg( ( 0x10 + i), adr)) return false;
        else p_cod[ i] = regReply;  // Read into product code array
    }
    return true;
}

//  = = = =    GET FIRMWARE VERSION   = = = =
// The 3 byte array variable `tfVer` declared in the
// example sketch decays to the array pointer `p_ver`.
bool TFLI2C::Get_Firmware_Version( uint8_t * p_ver, uint8_t adr)
{
    for (uint8_t i = 0; i < 3; ++i)
    {
      if( !readReg( ( 0x0A + i), adr)) return false;
        else p_ver[ i] = regReply;  // Read into version array
    }
    return true;
}

//  = = = = =    SAVE SETTINGS   = = = = =
bool TFLI2C::Save_Settings( uint8_t adr)
{
    return( writeReg( TFL_SAVE_SETTINGS, adr, 1));
}

//  = = = =   SOFT (SYSTEM) RESET   = = = =
bool TFLI2C::Soft_Reset( uint8_t adr)
{
    return( writeReg( TFL_SOFT_RESET, adr, 2));
}

//  = = = = = =    SET I2C ADDRESS   = = = = = =
// Range: 0x08, 0x77. Must reboot to take effect.
bool TFLI2C::Set_I2C_Addr( uint8_t adrNew, uint8_t adr)
{
    return( writeReg( TFL_SET_I2C_ADDR, adr, adrNew));
}

//  = = = = =   SET ENABLE   = = = = =
bool TFLI2C::Set_Enable( uint8_t adr)
{
    return( writeReg( TFL_DISABLE, adr, 1));
}

//  = = = = =   SET DISABLE   = = = = =
bool TFLI2C::Set_Disable( uint8_t adr)
{
    return( writeReg( TFL_DISABLE, adr, 0));
}

//  = = = = = =    SET FRAME RATE   = = = = = =
bool TFLI2C::Set_Frame_Rate( uint16_t &frm, uint8_t adr)
{
    // Recast the address of the unsigned integer `frm`
    // as a pointer to an unsigned byte `p_frm` ...
    uint8_t * p_frm = (uint8_t *) &frm;

    // ... then address the pointer as an array.
    if( !writeReg( ( TFL_FPS_LO), adr, p_frm[ 0])) return false;
    if( !writeReg( ( TFL_FPS_HI), adr, p_frm[ 1])) return false;
    return true;
}

//  = = = = = =    GET FRAME RATE   = = = = = =
bool TFLI2C::Get_Frame_Rate( uint16_t &frm, uint8_t adr)
{
    uint8_t * p_frm = (uint8_t *) &frm;
    if( !readReg( TFL_FPS_LO, adr)) return false;
        else p_frm[ 0] = regReply;  // Read into `frm` array
    if( !readReg( TFL_FPS_HI, adr)) return false;
        else p_frm[ 1] = regReply;  // Read into `frm` array
    return true;
}

//  = = = =   HARD RESET to Factory Defaults  = = = =
bool TFLI2C::Hard_Reset( uint8_t adr)
{
    return( writeReg( TFL_HARD_RESET, adr, 1));
}

//  = = = = = =   SET CONTINUOUS MODE   = = = = = =
// Sample LiDAR chip continuously at Frame Rate
bool TFLI2C::Set_Cont_Mode( uint8_t adr)
{
    return( writeReg( TFL_SET_TRIG_MODE, adr, 0));
}

//  = = = = = =   SET TRIGGER MODE   = = = = = =
// Device will sample only once when triggered
bool TFLI2C::Set_Trig_Mode( uint8_t adr)
{
    return( writeReg( TFL_SET_TRIG_MODE, adr, 1));
}

//  = = = = = =   SET TRIGGER   = = = = = =
// Trigger device to sample once
bool TFLI2C::Set_Trigger( uint8_t adr)
{
    return( writeReg( TFL_TRIGGER, adr, 1));
}
//
// = = = = = = = = = = = = = = = = = = = = = = = =

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       READ OR WRITE A GIVEN REGISTER OF THE SLAVE DEVICE
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool TFLI2C::readReg( uint8_t nmbr, uint8_t addr)
{
    bool b;
    b = _pWire->Begin(addr);
    uint8_t regbuf[1] = {nmbr};
    b = _pWire->Write(regbuf ,1);

    if(!b){  // If write error... 
        tfStatus = TFL_I2CWRITE;        // then set status code...
        return false;                   // and return `false`.
    }
    regbuf[0] = 0;

    b = _pWire->Read(regbuf ,1);
    if(!b){  // If write error... 
        tfStatus = TFL_I2CREAD;        // then set status code...
        return false;                   // and return `false`.
    }

    regReply = regbuf[0];
    return true;


//   Wire.beginTransmission( addr);
//   Wire.write( nmbr);

//   if( Wire.endTransmission() != 0)  // If write error...
//   {
//     tfStatus = TFL_I2CWRITE;        // then set status code...
//     return false;                   // and return `false`.
//   }
//   // Request 1 byte from the device
//   // and release bus when finished.
//   Wire.requestFrom( ( int)addr, 1, true);
//     if( Wire.peek() == -1)            // If read error...
//     {
//       tfStatus = TFL_I2CREAD;         // then set status code.
//       return false;
//     }
//   regReply = ( uint8_t)Wire.read();   // Read the received data...
//   return true;
}

bool TFLI2C::writeReg( uint8_t nmbr, uint8_t addr, uint8_t data)
{
    bool b;
    b = _pWire->Begin( addr);
    uint8_t regbuf[1] = {nmbr};
    b = _pWire->Write(regbuf ,1);
    regbuf[0] = data;
    b = _pWire->Write(regbuf ,1);
    if(!b){  // If write error...
        tfStatus = TFL_I2CWRITE;        // then set status code...
        return false;                   // and return `false`.
    }
    else return true;
    
//   Wire.beginTransmission( addr);
//   Wire.write( nmbr); //reg
//   Wire.write( data);
//   if( Wire.endTransmission( true) != 0)  // If write error...
//   {
//     tfStatus = TFL_I2CWRITE;        // then set status code...
//     return false;                   // and return `false`.
//   }
//   else return true;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - -    The following is for testing purposes    - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Called by either `printFrame()` or `printReply()`
// Print status condition either `READY` or error type
void TFLI2C::printStatus()
{
    printf("Status: ");
    if( tfStatus == TFL_READY)          printf( "READY\n");
    else if( tfStatus == TFL_SERIAL)    printf( "SERIAL\n");
    else if( tfStatus == TFL_HEADER)    printf( "HEADER\n");
    else if( tfStatus == TFL_CHECKSUM)  printf( "CHECKSU\nM");
    else if( tfStatus == TFL_TIMEOUT)   printf( "TIMEOUT\n");
    else if( tfStatus == TFL_PASS)      printf( "PASS\n");
    else if( tfStatus == TFL_FAIL)      printf( "FAIL\n");
    else if( tfStatus == TFL_I2CREAD)   printf( "I2C-READ\n");
    else if( tfStatus == TFL_I2CWRITE)  printf( "I2C-WRITE\n");
    else if( tfStatus == TFL_I2CLENGTH) printf( "I2C-LENGTH\n");
    else if( tfStatus == TFL_WEAK)      printf( "Signal weak\n");
    else if( tfStatus == TFL_STRONG)    printf( "Signal strong\n");
    else if( tfStatus == TFL_FLOOD)     printf( "Ambient light\n");
    else if( tfStatus == TFL_INVALID)   printf( "No Command\n");
    else printf( "OTHER\n");
}


// // Print error type and HEX values
// // of each byte in the data frame
// void TFLI2C::printDataArray()
// {
//     printStatus();
//     // Print the Hex value of each byte of data
//     Serial.print(" Data:");
//     for( uint8_t i = 0; i < 6; i++)
//     {
//       Serial.print(" ");
//       Serial.print( dataArray[ i] < 16 ? "0" : "");
//       Serial.print( dataArray[ i], HEX);
//     }
//     Serial.println();
// }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -