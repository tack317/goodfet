//! MSP430F2618 clock and I/O definitions

// Included by other 2xx ports, such as the 2274.
#include <msp430.h>
#include <sys/crtld.h>


#include "platform.h"

#include "dco_calib.h"



//! Receive a byte.
unsigned char serial0_rx(){
  char c;

  while(!(IFG2&UCA0RXIFG));//wait for a byte
  c = UCA0RXBUF;
  IFG2&=~UCA0RXIFG;

  //UCA0CTL1 &= ~UCA0RXSE;
  return c;
}

//! Receive a byte.
unsigned char serial1_rx(){
  char c;

#ifdef useuart1
  while (!(UC1IFG&UCA1RXIFG));               // USCI_A1 TX buffer ready?
  c = UCA1RXBUF;
  UC1IFG&=~UCA1RXIFG;
#endif
  
  return c;
}

//! Transmit a byte.
void serial0_tx(unsigned char x){
  while ((IFG2 & UCA0TXIFG) == 0); //loop until buffer is free
  UCA0TXBUF = x;	/* send the character */
  while(!(IFG2 & UCA0TXIFG));
}
//! Transmit a byte on the second UART.
void serial1_tx(unsigned char x){
#ifdef useuart1
  while ((UC1IFG & UCA1TXIFG) == 0); //loop until buffer is free
  UCA1TXBUF = x;	/* send the character */
  while(!(UC1IFG & UCA1TXIFG));
#endif
}


//! Set the baud rate.
void setbaud0(unsigned char rate){

  //Table 15-4, page 481 of 2xx Family Guide
  switch(rate){
  case 1://9600 baud
    UCA0BR1 = 0x06;
    UCA0BR0 = 0x82;
    break;
  case 2://19200 baud
    UCA0BR1 = 0x03;
    UCA0BR0 = 0x41;
    break;
  case 3://38400 baud
    UCA0BR1 = 0xa0;
    UCA0BR0 = 0x01;
    break;
  case 4://57600 baud
    UCA0BR1 = 0x1d;
    UCA0BR0 = 0x01;
    break;
  default:
  case 5://115200 baud
    UCA0BR0 = 0x8a;
    UCA0BR1 = 0x00;
    break;
  }
}

//! Set the baud rate of the second uart.
void setbaud1(unsigned char rate){
#if useuart1 || enableuart1
  //Table 15-4, page 481 of 2xx Family Guide
  switch(rate){
  case 1://9600 baud
    UCA1BR1 = 0x06;
    UCA1BR0 = 0x82;
    break;
  case 2://19200 baud
    UCA1BR1 = 0x03;
    UCA1BR0 = 0x41;
    break;
  case 3://38400 baud
    UCA1BR1 = 0xa0;
    UCA1BR0 = 0x01;
    break;
  case 4://57600 baud
    UCA1BR1 = 0x1d;
    UCA1BR0 = 0x01;
    break;
  default:
  case 5://115200 baud
    UCA1BR0 = 0x8a;
    UCA1BR1 = 0x00;
    break;
  }
#endif
}

#define BAUD0EN 0x41
#define BAUD1EN 0x03

void msp430_init_uart(){

  // Serial0 on P3.4, P3.5
  P3SEL |= BIT4 + BIT5;
  P3DIR |= BIT4;

  //UCA0CTL1 |= UCSWRST;                    /* disable UART */

  UCA0CTL0 = 0x00;
  //UCA0CTL0 |= UCMSB ;

  UCA0CTL1 |= UCSSEL_2;                     // SMCLK

  //UCA0BR0 = BAUD0EN;                      // 115200
  //UCA0BR1 = BAUD1EN;
  setbaud(5);                               //default baud, 115200

  UCA0MCTL = 0;                             // Modulation UCBRSx = 5
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**


  //Leave this commented!
  //Interrupt is handled by target code, not by bootloader.
  //IE2 |= UCA0RXIE; //DO NOT UNCOMMENT
  
  #if useuart1 || enableuart1
  // Serial1 on P3.6, P3.7
  P3SEL |= BIT6 + BIT7;
  P3DIR |= BIT6;

  UCA1CTL0 = 0x00;
  UCA1CTL1 |=  UCSSEL_2;                     // SMCLK

  setbaud1(5);                               //default baud, 115200

  UCA1MCTL  =  0;
  UCA1CTL1 &= ~UCSWRST;                      // Initialize USCI state machine
  #endif
}


//This must be in .noinit.
__attribute__ ((section (".noinit"))) char dcochoice;

//! Initialization is correct.
void msp430_init_dco_done(){
  //char *dcochoice=(char *) DCOCHOICEAT; //First word of RAM.
  dcochoice--;
}

//! Initialize the MSP430 clock.
void msp430_init_dco() {
  int i=1000;
  //char *dcochoice=(char *) DCOCHOICEAT; //First word of RAM.
  
  #ifdef __MSP430_HAS_PORT8__
  P8SEL = 0; // disable XT2 on P8.7/8
  #endif
  
  //Set P2.6 mode for MSP430F2274
  #ifndef __MSP430_HAS_PORT5__
  P2SEL = 0; //disable XIN on 2274
  #endif
  
  
  #ifdef STATICDCO
  BCSCTL1 = (STATICDCO>>8);
  DCOCTL  = (STATICDCO&0xFF);
  #else
  if(CALBC1_16MHZ!=0xFF){
    //Info is intact, use it.
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
  }else{
    /*
      Info is missing, guess at a good value.

      A list of correct calibrations in included as dco_calib.c,
      generated by script.
    */
    DCOCTL = 0x00; //clear DCO

    BCSCTL1 =  dco_calibrations[2*dcochoice+1];
    DCOCTL  =  dco_calibrations[2*dcochoice];
    dcochoice++;
    dcochoice%=dco_calibrations_count;
  }
  #endif

  //Minor delay.
  while(i--);



  return;
}

