/*-----------------------------------------------------------------------------
/
/
/
/
/----------------------------------------------------------------------------*/

#include <avr/io.h>
#include <util/delay.h>
#include "xmega_digital.h"

#define ACK 1
#define NO_ACK 0

#define READ 1
#define WRITE 0

#define write_address(addr) ((addr << 1)|WRITE)
#define read_address(addr) ((addr << 1)|READ)

void I2C_Init();

void I2C_Start();

void I2C_Stop();

void I2C_WriteBit( unsigned char c );

unsigned char I2C_ReadBit();

unsigned char I2C_Write( unsigned char c );

unsigned char I2C_Read( unsigned char ack );
