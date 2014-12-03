/*-----------------------------------------------------------------------------
/ OV7670 image sensor data capture with Atxmega32E5 without using external FIFO
/------------------------------------------------------------------------------
/ With current configuration, this system sends 80 x 60 grayscale image data 
/ over 3Mbaud serial stream. There is 3 byte preamble {0xAA,0x55,0xAA} to
/ indicate the start of a new image frame. Computer software / other MCU can 
/ use those bytes to sync with the data stream.
/------------------------------------------------------------------------------
/ PORTD.7    => UART TX (UART is configured as 3Mbaud)
/ PORTD.5    => CAMERA RESET 
/ PORTD.4    => CAMERA XLCK (Master clock)
/ PORTD.3    => CAMERA SIOD (I2C data)
/ PORTD.2    => CAMERA SIOC (I2C clock)
/ PORTD.1    => CAMERA HSYNC (Horizontal SYNC)
/ PORTD.0    => CAMERA VSYNC (Vertical SYNC)
/ PORTA.7    => CAMERA PCLK (Pixel clock)
/ PORTC[7:0] => CAMERA DATA[7:0] (Parallel data bus)
/------------------------------------------------------------------------------
/ Heavily inspired by "Interfacing a cheap phone camera module to a PIC32
/ microcontroller" project by @mikeselectricstuff.
/ Video link of that project: [http://www.youtube.com/watch?v=rQYByorpoFk]
/------------------------------------------------------------------------------
/ “THE COFFEEWARE LICENSE” (Revision 1):
/ <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
/ can do whatever you want with this stuff. If we meet some day, and you think
/ this stuff is worth it, you can buy me a coffee in return.
/----------------------------------------------------------------------------*/
#include <avr/io.h>	
#include "xprintf.h"
#include <util/delay.h>
#include "xmega_digital.h"
#include "bitbang_i2c.h"
#include "ov7670reg.h"
#include <avr/interrupt.h>
/*---------------------------------------------------------------------------*/
void init_uart();
void ov7670_init();
void init_clockout();
void initClock_32Mhz();
void sendch(uint8_t ch);
void store_pixels_dma_ch2();
uint8_t ov7670_get(uint8_t addr);
uint8_t ov7670_set(uint8_t addr, uint8_t val);
void send_uart_dma_ch0(uint8_t* buf, uint16_t len);
/*---------------------------------------------------------------------------*/
uint8_t linebuffer[160]; /* Buffer for a single line */
/*---------------------------------------------------------------------------*/
#define WIDTH_MAX 160
#define HEIGHT_MAX 120
/*---------------------------------------------------------------------------*/
volatile uint8_t line_index = 0;
volatile uint8_t width_scale = 2; /* minimum: 2 */
volatile uint8_t height_scale = 2; /* minimum: 2 */
/*---------------------------------------------------------------------------*/
int main()
{		
    /*-----------------------------------------------------------------------*/
    
    initClock_32Mhz();
    
    init_uart();
    
    ov7670_init();

    /* Enable the EDMA module */
    /* 0,1 are peripheral channels. 2 is standard channel */
    EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD2_gc; 

    /*-----------------------------------------------------------------------*/

    /* Port C - Data pins - All inputs */
    PORTC.DIR = 0x00;

    pinMode(D,1,INPUT); // HSYNC
    PORTD.PIN1CTRL |= PORT_ISC_FALLING_gc; // falling edge on hsync indicates end of line
    
    pinMode(D,0,INPUT); // VSYNC
    PORTD.PIN0CTRL |= PORT_ISC_FALLING_gc; // falling edge on vsync indicates end of an image 

    /* Enable interrupts on the PORTD */
    PORTD.INTCTRL = PORT_INTLVL_HI_gc;
    PORTD.INTMASK = (1<<1) | (1<<0);

    /*-----------------------------------------------------------------------*/  

    pinMode(A,7,INPUT); // PCLK
    PORTA.PIN7CTRL |= PORT_ISC_FALLING_gc; // falling edge pixel clock
    EVSYS.CH0MUX = EVSYS_CHMUX_PORTA_PIN7_gc;

    /*-----------------------------------------------------------------------*/    

    /* Generate capture event from pixel clock using a timer */
    TCC4.CCA = 0;
    TCC4.CTRLE = (1<<0);
    TCC4.PER = (2 * width_scale) - 1;
    TCC4.CTRLA = TC45_CLKSEL_EVCH0_gc;      
    EVSYS.CH1MUX = EVSYS_CHMUX_TCC4_CCA_gc;    

    /*-----------------------------------------------------------------------*/    

    /* Data capture event output for debugging */
    pinMode(R,0,OUTPUT);
    PORTCFG.ACEVOUT = (1<<0) | (1<<3) | (1<<4) | (1<<5);

    /*-----------------------------------------------------------------------*/    

    /* Enable all interrupt levels */
    PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;    

    /* Clear all previous interrupt flags */
    PORTD.INTFLAGS = 0xFF;

    sei(); 

    /*-----------------------------------------------------------------------*/    

    while(1)
    {	        
          /* Empty main loop ... */
    }

    /*-----------------------------------------------------------------------*/    
}
/*---------------------------------------------------------------------------*/
ISR(PORTD_INT_vect)
{
    /* End of a single line */
    if(PORTD.INTFLAGS & (1<<1))
    {                  
        /* Capture the next line */
        if(line_index == (height_scale -1))
        {                              
            line_index = 0;  
            
            /* Reset timer counter */                       
            TCC4.CNT = 0x00; 

            /* Get ready to capture the next line */
            store_pixels_dma_ch2(linebuffer,WIDTH_MAX / width_scale);            
        }
        /* Skip the next line */
        else
        {
            /* If we've just captured a line, transfer that line */
            if(line_index == 0)
            {
                send_uart_dma_ch0(linebuffer,WIDTH_MAX / width_scale);                    
            } 

            line_index++;                        
        }
        
        PORTD.INTFLAGS = (1<<1); /* Clear ISR flag */
    }
    /* End of a complete image */
    else if(PORTD.INTFLAGS & (1<<0))
    {
        line_index = 0;              

        /* Send preamble for the image data */   
        sendch(0xAA);
        sendch(0x55);
        sendch(0xAA);     

        PORTD.INTFLAGS = (1<<0); /* Clear ISR flag */
    }   
}
/*---------------------------------------------------------------------------*/
void sendch(uint8_t ch)
{
    while(!(USARTD0.STATUS & USART_DREIF_bm));
    USARTD0.DATA = ch;
}
/*---------------------------------------------------------------------------*/
void init_clockout()
{
    /* Clock out pin */
    pinMode(D,4,OUTPUT);
    digitalWrite(D,4,HIGH);

    /* Each clock tick is: 32.000.000 / prescale Hz */
    TCD5.CTRLA = TC45_CLKSEL_DIV1_gc;

    /* Frequency mode */
    TCD5.CTRLB = 0x01;

    /* 16 / (n+1) = Output frequency */
    TCD5.CCA = 0;

    /* Output compare enabled for CCA */
    TCD5.CTRLE = (1<<0);
}
/*---------------------------------------------------------------------------*/
void init_uart()
{
    /* Set UART pin driver states */
    pinMode(D,6,INPUT);
    pinMode(D,7,OUTPUT);

    /* Remap the UART pins */
    PORTD.REMAP = PORT_USART0_bm;

    USARTD0.CTRLB = USART_RXEN_bm|USART_TXEN_bm;
    USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc|USART_PMODE_DISABLED_gc|USART_CHSIZE_8BIT_gc;
    
    USARTD0.CTRLB |= (1<<2); /* CLK2X */
    
    /* 3000000 baud rate with 32MHz clock */
    USARTD0.BAUDCTRLA = 43; USARTD0.BAUDCTRLB = (-7 << USART_BSCALE_gp);        
}
/*---------------------------------------------------------------------------*/
void initClock_32Mhz()
{
    /* Generates 32Mhz clock from internal 2Mhz clock via PLL */
    OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | 16;
    OSC.CTRL |= OSC_PLLEN_bm ;
    while((OSC.STATUS & OSC_PLLRDY_bm) == 0);
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}
/*---------------------------------------------------------------------------*/
void ov7670_init()
{
    I2C_Init();
    init_clockout();

    /* Reset pin */
    pinMode(D,5,OUTPUT);
    digitalWrite(D,5,LOW);  _delay_ms(100);
    digitalWrite(D,5,HIGH); _delay_ms(100);

    ov7670_set(REG_COM7, 0x80); /* reset to default values */
    _delay_ms(100);
    ov7670_set(REG_COM7, 0x00); /* reset to default values */
    _delay_ms(100);

    /* These settings are mostly coming from the link below */
    /* http://www.rpg.fi/desaster/blog/2012/05/07/ov7670-camera-sensor-success/ */
    ov7670_set(REG_COM11, 0x0A);
    ov7670_set(REG_TSLB, 0x04);
    
    ov7670_set(REG_HSTART, 0x16);
    ov7670_set(REG_HSTOP, 0x04);
    ov7670_set(REG_HREF, 0x24);
    ov7670_set(REG_VSTART, 0x02);
    ov7670_set(REG_VSTOP, 0x7a);
    ov7670_set(REG_VREF, 0x0a);
    ov7670_set(REG_COM10, 0x02 + (1<<5));
    ov7670_set(REG_COM3, 0x04);
    ov7670_set(REG_MVFP, 0x27);
    
    ov7670_set(REG_COM14, 0x1a); // divide by 4
    ov7670_set(0x72, 0x22); // downsample by 4    
    ov7670_set(0x73, 0xf2); // divide by 4
}
/*---------------------------------------------------------------------------*/
uint8_t ov7670_set(uint8_t addr, uint8_t val)
{   
    int rc;

    I2C_Start();
        rc = I2C_Write(write_address(0x21));
        rc = I2C_Write(addr);
        rc = I2C_Write(val);
    I2C_Stop();

    return rc;
}
/*---------------------------------------------------------------------------*/
uint8_t ov7670_get(uint8_t addr)
{
    int rc;
    uint8_t data;

    I2C_Start();
        rc = I2C_Write(write_address(0x21));
        rc = I2C_Write(addr);
    I2C_Stop();

    _delay_ms(1);

    I2C_Start();
        rc = I2C_Write(read_address(0x21));
        data = I2C_Read(NO_ACK);
    I2C_Stop();

    return data;
}
/*---------------------------------------------------------------------------*/
void store_pixels_dma_ch2(uint8_t* buf, uint16_t len)
{
     /* Source address */
    EDMA.CH2.ADDR = &(PORTC.IN);

    /* Transfer size */
    EDMA.CH2.TRFCNT = len;

    /* Destination address */
    EDMA.CH2.DESTADDR = buf;

    /* Destination Memory address mode: Increment */
    EDMA.CH2.DESTADDRCTRL = EDMA_CH_DIR_INC_gc;

    /* Trigger source */
    EDMA.CH2.TRIGSRC = EDMA_CH_TRIGSRC_EVSYS_CH1_gc;

    /* Single shot mode */
    EDMA.CH2.CTRLA = EDMA_CH_SINGLE_bm;    

    /* Enable the channel #2 ! */
    EDMA.CH2.CTRLA |= EDMA_CH_ENABLE_bm;
}
/*---------------------------------------------------------------------------*/
void send_uart_dma_ch0(uint8_t* buf, uint16_t len)
{   
    /* Source address */
    EDMA.CH0.ADDR = buf; 

    /* Transfer size */
    EDMA.CH0.TRFCNT = len;

    /* Trigger source: Data register empty */
    EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;

    /* Memory address mode: Increment */
    EDMA.CH0.ADDRCTRL = EDMA_CH_DIR_INC_gc;

    /* Single shot mode */
    EDMA.CH0.CTRLA = EDMA_CH_SINGLE_bm;

    /* Enable the channel #0 ! */
    EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
}
/*---------------------------------------------------------------------------*/
