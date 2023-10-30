/*******************************************************************************
 * Copyright (c) 2023 Renesas Electronics Corporation
 * All Rights Reserved.
 *
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    zmod4xxx_hal.c
 * @brief   zmod4xxx hardware abstraction layer function definitions
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */

#include <string.h>   // for memcpy

#include "hal.h"
#include "zmod4xxx_hal.h"

// static HAL_t    _hal;  // 2023.08.27
static uint8_t  _i2cBuffer [ 257 ];

static void _msSleep(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

static int8_t
_i2c_read_reg ( const i2c_dev_t dev, uint8_t  reg_addr, uint8_t*  data, uint8_t  len ) {
    if (i2c_dev_read_reg(&dev, reg_addr, data, len) )
      return ERROR_I2C;
    return ZMOD4XXX_OK;
}
// wrapper function, mapping register read api to generic I2C API
// static int8_t
// _i2c_read_reg ( uint8_t  slaveAddr, uint8_t  addr, uint8_t*  data, uint8_t  len ) {
//   if ( _hal . i2cWriteRead ( slaveAddr, &addr, 1, data, len ) )
//     return ERROR_I2C;
//   return ZMOD4XXX_OK;
// }

static int8_t
_i2c_write_reg ( const i2c_dev_t dev, uint8_t  reg_addr, uint8_t*  data, uint8_t  len ) {
    _i2cBuffer [ 0 ] = reg_addr;
    memcpy ( _i2cBuffer + 1, data, len );
    if ( i2c_dev_write_reg ( &dev, reg_addr, _i2cBuffer, len + 1 ) )
      return ERROR_I2C;
    return ZMOD4XXX_OK;
}
// wrapper function, mapping register write api to generic I2C API
// static int8_t
// _i2c_write_reg ( uint8_t  slaveAddr, uint8_t  addr, uint8_t*  data, uint8_t  len ) {
//   _i2cBuffer [ 0 ] = addr;
//   memcpy ( _i2cBuffer + 1, data, len );
//   if ( _hal . i2cWrite ( slaveAddr, _i2cBuffer, len + 1 ) )
//     return ERROR_I2C;
//   return ZMOD4XXX_OK;
// }



int
init_hardware ( zmod4xxx_dev_t*  dev) {
  // initialize the target hardware and populate functions in _hal object
  // int  errorCode = HAL_Init ( &_hal ); // 2023.08.27
  // if ( errorCode )  return errorCode;  // 2023.08.27
  
  // verify we have all functions required for the ZMOD4xxx API
  // if ( !_hal . i2cWriteRead )
  //   return heI2CWriteReadRequried;
  
  // if ( !_hal . i2cWrite )
  //   return heI2CReadRequried;
  
  // if ( !_hal . msSleep )
  //   return heI2CReadRequried;
  
  // populate function pointers in legacy ZMOD4xxx API
  dev -> write    = _i2c_write_reg;
  dev -> read     = _i2c_read_reg;
  dev -> delay_ms = _msSleep;
  // dev -> delay_ms = _hal . msSleep;    2023.08.27
  dev -> delay_ms ( 200 );
  
  return ecSuccess;
}

int
deinit_hardware ( ) {
  return  i2cdev_done();
  // return  HAL_Deinit ( &_hal ); // 2023.08.27
}
