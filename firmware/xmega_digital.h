/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
*/

/* Xmega spesific digital I/O macros */

#ifndef DIGITAL
#define DIGITAL

#include <avr/io.h>

#define HIGH 1
#define LOW 0

#define OUTPUT 1
#define INPUT 0

#define pinMode(port,pin,state) state ? (PORT ##port .DIRSET = (1<<pin)) : (PORT ##port .DIRCLR = (1<<pin))

#define digitalWrite(port,pin,state) state ? (PORT ##port .OUTSET = (1<<pin)) : (PORT ##port .OUTCLR = (1<<pin))

#define digitalRead(port,pin) (PORT ##port .IN & (1<<pin))

#define togglePin(port,pin) (PORT ##port .OUTTGL = (1<<pin))

#define setInternalPullup(port,pin) (PORT ##port .PIN ##pin ##CTRL = 3<<3)

#define setInternalPullDown(port,pin) (PORT ##port .PIN ##pin ##CTRL = 2<<3)

#define disableDigitalBuffer(port,pin) (PORT ##port .PIN ##pin ##CTRL = 15)

#endif