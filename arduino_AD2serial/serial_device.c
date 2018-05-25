#include <inttypes.h>
#include <avr/io.h>
#include "serial_device.h"

#define NUM (F_CPU/16)

#define BAUDRATE_L(x) UINT8_C((NUM/x-1)      & 0xff)
#define BAUDRATE_H(x) UINT8_C((NUM/x-1) >> 8 & 0xf)

/*
 * Initialize the UART0
 */
void serial_init(void) {
 UBRR0= 8;
 UCSR0A= 0; //(1<<U2X0);
 UCSR0B = _BV(RXEN0) | _BV(TXEN0);
}


uint8_t serial_get(void) {
  // polling on RXC until data received
  loop_until_bit_is_set(UCSR0A, RXC0);  	
  return UDR0;
}


bool serial_can_read(void) {
  // test whether there is something to read
  return bit_is_set(UCSR0A, RXC0);
}


void serial_put(uint8_t c) {
  // wait last transmision finishes
  loop_until_bit_is_set(UCSR0A, UDRE0);
  // send new byte
  UDR0 = c;
}

