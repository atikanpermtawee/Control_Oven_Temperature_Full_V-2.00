/*
 * File:   MCP9600.c
 * Author: Atikan
 *
 * Created on December 18, 2017, 4:46 PM
 */


#include <xc.h>
#include "MCP9600.h"

uint8_t MCP9600_Read(uint8_t reg, uint8_t *pData,uint8_t count)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    static I2C1_TRANSACTION_REQUEST_BLOCK trb[2];
    I2C1_MasterWriteTRBBuild(&trb[0], &reg, 1, MCP9600_ADDR);
    I2C1_MasterReadTRBBuild(&trb[1], pData, count,MCP9600_ADDR);    
    I2C1_MasterTRBInsert(2, &trb[0], &status);
}