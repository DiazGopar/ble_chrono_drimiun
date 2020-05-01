
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include "drimiun_drv_twim.h"
#include <string.h>
#include <time.h>
#include <math.h>


extern nrfx_twim_t twim;



int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  
  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t *reg;

  reg = (uint8_t *) malloc(count+2);
  if(reg == NULL) return(-41);

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;

  memcpy(&reg[2], pdata, count);
  err_code = nrfx_twim_tx(&twim,dev,reg,5,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
  free(reg);
    
  return status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  
  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[2];

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;

  err_code = nrfx_twim_tx(&twim,dev,reg,2,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  } else {
    err_code = nrfx_twim_rx(&twim,dev,pdata,count);
    if(err_code != 0) {
        status = VL53L1_ERROR_CONTROL_INTERFACE;
    } 
  }
    
  return status;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
  
  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[3]; 

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;
  reg[2] = data;

  err_code = nrfx_twim_tx(&twim,dev,reg,3,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
  
  return status; 
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
  
  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[4];

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;
  reg[2] = data >> 8;
  reg[3] = data & 0xFF;  

  err_code = nrfx_twim_tx(&twim,dev,reg,4,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
    
  return status;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
  
  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[6];

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;
  reg[2] = (data >> 24) & 0xFF;
  reg[3] = (data >> 16) & 0xFF;
  reg[4] = (data >> 8)  & 0xFF;
  reg[5] = data & 0xFF;

  err_code = nrfx_twim_tx(&twim,dev,reg,6,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
    
  return status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {

  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[2];

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;

  err_code = nrfx_twim_tx(&twim,dev,reg,2,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  } else {
    err_code = nrfx_twim_rx(&twim,dev,data,sizeof(*data));
    if(err_code != 0) {
        status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
  }
  
  return status; 
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
  
  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[2];

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;
  
  err_code = nrfx_twim_tx(&twim,dev,reg,2,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  } else {
    err_code = nrfx_twim_rx(&twim,dev,reg,2);
    if(err_code != 0) {
        status = VL53L1_ERROR_CONTROL_INTERFACE;
    } else {
      *data = ((uint16_t)reg[0]<<8) + (uint16_t)reg[1];
    }
  }
    
  return status; 
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {

  VL53L1_Error status = VL53L1_ERROR_NONE;
  ret_code_t err_code;
  uint8_t reg[4];

  reg[0] = index >> 8;
  reg[1] = index & 0xFF;

  err_code = nrfx_twim_tx(&twim,dev,reg,2,true);
  if(err_code != 0) {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  } else {
    err_code = nrfx_twim_rx(&twim,dev,reg,4);
    if(err_code != 0) {
        status = VL53L1_ERROR_CONTROL_INTERFACE;
    } else {
      *data = ((uint32_t)reg[0]<<24) + ((uint32_t)reg[1]<<16) 
            + ((uint32_t)reg[2]<<8) + (uint32_t)reg[3];
    }
  }
    
  return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
  
  nrf_delay_ms(wait_ms);
  return 0; // to be implemented
}
