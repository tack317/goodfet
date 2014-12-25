/*! \file avrdude.c
  Based on ArduinoISP version 04m3
*/

#include "platform.h"
#include "command.h"
#include "avr.h"

// Ardurino Definiton
#define LOW 0
#define HIGH 1

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

int error=0;
int pmode=0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  int flashpoll;
  int eeprompoll;
  int pagesize;
  int eepromsize;
  int flashsize;
} parameter;

parameter param;

void putch(char c) {
  while ((IFG2 & UCA0TXIFG) == 0);
	  asm("nop");
  UCA0TXBUF = c;
}

void putstr(char* str) {
	int8_t i = -1;
	do {
		i++;
		putch(str[i]);
	} while(str[i] != '\0');
}

uint8_t available() {
	return ((IFG2 & UCA0RXIFG) != 0);
}

uint8_t getch() {
  while(!available())
	  asm("nop");
  char c = UCA0RXBUF;
  IFG2 &=~ UCA0RXIFG;
  return c;
}

void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

uint8_t spi_send(uint8_t b) {
	return spitrans8(b);
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n;
  spi_send(a); 
  n=spi_send(b);
  //if (n != a) error = -1;
  n=spi_send(c);
  return spi_send(d);
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    putch(STK_INSYNC);
    putch(STK_OK);
  } 
  else {
    error++;
    putch(STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    putch(STK_INSYNC);
    putch(b);
    putch(STK_OK);
  } 
  else {
    error++;
    putch(STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch(c) {
  case 0x80:
    breply(HWVER);
    break;
  case 0x81:
    breply(SWMAJ);
    break;
  case 0x82:
    breply(SWMIN);
    break;
  case 0x93:
    breply('S'); // serial programmer
    break;
  default:
    breply(0);
  }
}

void set_parameters() {
  // call this after reading paramter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8]; 
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
    + buff[17] * 0x00010000
    + buff[18] * 0x00000100
    + buff[19];
}

void start_pmode() {
  //Prepare Timer
  prep_timer();
  
  //Get MCU into ISP Mode
  SPIDIR|=SCK+BIT0;      //BIT0 might be SS
  SETSS;
  DIRSS;
  DIRCE;
  
  SPIDIR|=MOSI;
  SETSS;
  CLRCLK;
  delay_ms(20);
  CLRSS;
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  SPIDIR &= ~(MISO+MOSI+SCK+BIT0);
  pmode   =  0;
}

void universal() {
  int w;
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, int addr, uint8_t data) {
  spi_transaction(0x40+8*hilo, 
  addr>>8 & 0xFF, 
  addr & 0xFF,
  data);
}
void commit(int addr) {
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

int current_page(int addr) {
  if (param.pagesize == 32)  return here & 0xFFFFFFF0;
  if (param.pagesize == 64)  return here & 0xFFFFFFE0;
  if (param.pagesize == 128) return here & 0xFFFFFFC0;
  if (param.pagesize == 256) return here & 0xFFFFFF80;
  return here;
}

uint8_t write_eeprom_chunk(int start, int length) {
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)
  fill(length);
  for (int x = 0; x < length; x++) {
    int addr = start+x;
    spi_transaction(0xC0, (addr>>8) & 0xFF, addr & 0xFF, buff[x]);
    delay_ms(45);
  } 
  return STK_OK;
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  int page = current_page(here);
  while (x < length) {
    if (page != current_page(here)) {
      commit(page);
      page = current_page(here);
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);
  return STK_OK;
}

void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    putch(STK_INSYNC);
    putch((char)write_flash_pages(length));
  } 
  else {
    error++;
    putch(STK_NOSYNC);
  }
}

#define EECHUNK (32)
uint8_t write_eeprom(int length) {
  // here is a word address, get the byte address
  int start = here * 2;
  int remaining = length;
  if (length > param.eepromsize) {
    error++;
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}

void program_page() {
  char result = (char) STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      putch(STK_INSYNC);
      putch(result);
    } 
    else {
      error++;
      putch(STK_NOSYNC);
    }
    return;
  }
  putch(STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, int addr) {
  return spi_transaction(0x20 + hilo * 8,
  (addr >> 8) & 0xFF,
  addr & 0xFF,
  0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x+=2) {
    uint8_t low = flash_read(LOW, here);
    putch(low);
    uint8_t high = flash_read(HIGH, here);
    putch(high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    putch(ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    error++;
    putch(STK_NOSYNC);
    return;
  }
  putch(STK_INSYNC);
  if (memtype == 'F') result = flash_read_page(length);
  if (memtype == 'E') result = eeprom_read_page(length);
  putch(result);
  return;
}

void read_signature() {
  if (CRC_EOP != getch()) {
    error++;
    putch(STK_NOSYNC);
    return;
  }
  putch(STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  putch(high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  putch(middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  putch(low);
  putch(STK_OK);
}

void avrisp() { 
  led_on();
  uint8_t data, low, high;
  uint8_t ch = getch();
  switch (ch) {
  case '0': // signon
    error = 0;
    empty_reply();
    break;
  case '1':
    if (getch() == CRC_EOP) {
      putch(STK_INSYNC);
      putstr("AVR ISP");
      putch(STK_OK);
    }
    break;
  case 'A':
    get_version(getch());
    break;
  case 'B':
    fill(20);
    set_parameters();
    empty_reply();
    break;
  case 'E': // extended parameters - ignore for now
    fill(5);
    empty_reply();
    break;

  case 'P':
    start_pmode();
    empty_reply();
    break;
  case 'U': // set address (word)
    here = getch();
    here += 256 * getch();
    empty_reply();
    break;

  case 0x60: //STK_PROG_FLASH
    low = getch();
    high = getch();
    empty_reply();
    break;
  case 0x61: //STK_PROG_DATA
    data = getch();
    empty_reply();
    break;

  case 0x64: //STK_PROG_PAGE
    program_page();
    break;

  case 0x74: //STK_READ_PAGE 't'
    read_page();    
    break;

  case 'V': //0x56
    universal();
    break;
  case 'Q': //0x51
    error=0;
    end_pmode();
    empty_reply();
    break;

  case 0x75: //STK_READ_SIGN 'u'
    read_signature();
    break;

    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
  case CRC_EOP:
    error++;
    putch(STK_NOSYNC);
    break;

    // anything else we will return STK_UNKNOWN
  default:
    error++;
    if (CRC_EOP == getch()) 
      putch(STK_UNKNOWN);
    else
      putch(STK_NOSYNC);
  }
  led_off();
}

void avrdudeloop(void) {
  if (available()) {
    avrisp();
  }
}

