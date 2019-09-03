#include <xc.h> // include processor files - each processor file is guarded.  
#include "mcc_generated_files/mcc.h"
// MCP9600 I2C address
#define MCP9600_ADDR                        0x60  //when; ADDR pin connect to GND 
                                                  //Real = 0xC0 but in I2C.C  ptrb->address = address << 1; = 0x60
uint8_t MCP9600_Buff[2];
// Register address
#define MCP9600_TH                              0x00
#define MCP9600_Td                              0x01
#define MCP9600_TC                              0x02
#define MCP9600_Raw_ADC_Data                    0x03
#define MCP9600_Status                          0x04
#define MCP9600_Sensor_Config                   0x05
#define MCP9600_Device_Config                   0x06
#define MCP9600_Alert1_Config                   0x08
#define MCP9600_Alert2_Config                   0x09
#define MCP9600_Alert3_Config                   0x0A
#define MCP9600_Alert4_Config                   0x0B
#define MCP9600_THYST1                          0x0C
#define MCP9600_THYST2                          0x0D
#define MCP9600_THYST3                          0x0E
#define MCP9600_THYST4                          0x0F
#define MCP9600_TALERT1                         0x10
#define MCP9600_TALERT2                         0x11
#define MCP9600_TALERT3                         0x12
#define MCP9600_TALERT4                         0x13
#define MCP9600_DEVICE_ID                       0x20

uint8_t MCP9600_Read(uint8_t reg, uint8_t *pData,uint8_t count);

