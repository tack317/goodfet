/*! \file rs232.c
  \author Andreas Droescher
  \brief rs232 path through uart0 <-> uart1 @ 115200
*/

#include "platform.h"
#include "command.h"

#ifndef _GNU_ASSEMBLER_
#include <msp430.h>
#endif

void serialrelayloop() {
  char c;

  //Check UART0
  if(IFG2 & UCA0RXIFG) {
    c = UCA0RXBUF;
    IFG2 &=~ UCA0RXIFG;

    while ((UC1IFG & UCA1TXIFG) == 0);
      UCA1TXBUF = c;
    led_toggle();
  }

  //CHECK UART1
  if(UC1IFG & UCA1RXIFG) {
    c = UCA1RXBUF;
    UC1IFG &=~ UCA1RXIFG;

    while ((IFG2 & UCA0TXIFG) == 0);
      UCA0TXBUF = c;
    led_toggle();
  }
}
