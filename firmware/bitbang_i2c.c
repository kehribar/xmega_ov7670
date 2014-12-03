/*-----------------------------------------------------------------------------
/
/
/
/
/----------------------------------------------------------------------------*/

// Port for the I2C
#define I2C_DELAY() _delay_us(50)

#define I2C_DATA_HI() pinMode(D,3,INPUT) 
#define I2C_DATA_LO() pinMode(D,3,OUTPUT) 

#define I2C_CLOCK_HI() pinMode(D,2,INPUT) 
#define I2C_CLOCK_LO() pinMode(D,2,OUTPUT) 

// ----------------------------------------------------------------------------
#include "./bitbang_i2c.h"

void I2C_WriteBit( unsigned char c )
{
	if ( c > 0 )
	{
		I2C_DATA_HI();
	}
	else
	{
		I2C_DATA_LO();
	}

	I2C_CLOCK_HI();
	I2C_DELAY();
	
	I2C_CLOCK_LO();
	I2C_DELAY();

	if ( c > 0 )
	{
		I2C_DATA_LO();
	}
}

unsigned char I2C_ReadBit()
{
	I2C_DATA_HI();

	I2C_CLOCK_HI();
	I2C_DELAY();

	unsigned char c = digitalRead(C,0);

	I2C_CLOCK_LO();
	I2C_DELAY();

	return c;
}

void I2C_Init()
{
	// I2C_PORT &= ~( ( 1 << I2C_DAT ) | ( 1 << I2C_CLK ) );
	digitalWrite(C,0,LOW);
	digitalWrite(C,1,LOW);

	I2C_CLOCK_HI();
	I2C_DATA_HI();

	I2C_DELAY();
}

void I2C_Start()
{
	// set both to high at the same time
	// I2C_DDR &= ~( ( 1 << I2C_DAT ) | ( 1 << I2C_CLK ) );
	I2C_CLOCK_HI();
	I2C_DATA_HI();
	I2C_DELAY();

	I2C_DATA_LO();
	I2C_DELAY();

	I2C_CLOCK_LO();
	I2C_DELAY();
}

void I2C_Stop()
{
	I2C_DATA_LO();
	I2C_CLOCK_LO();
	I2C_DELAY();

	I2C_CLOCK_HI();
	I2C_DELAY();

	I2C_DATA_HI();
	I2C_DELAY();
}

unsigned char I2C_Write( unsigned char c )
{
	char i;

	for (i=0;i<8;i++)
	{
		I2C_WriteBit( c & 128 );

		c<<=1;
	}

	return I2C_ReadBit();
}

unsigned char I2C_Read( unsigned char ack )
{
	unsigned char res = 0;
	char i;

	for (i=0;i<8;i++)
	{
		res <<= 1;
		res |= I2C_ReadBit();
	}

	if ( ack > 0)
	{
		I2C_WriteBit( 0 );
	}
	else
	{
		I2C_WriteBit( 1 );
	}

	I2C_DELAY();

	return res;
}
// ----------------------------------------------------------------------------
