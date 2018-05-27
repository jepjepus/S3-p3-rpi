#include <avr/interrupt.h>
#include "adc.h"
#include "tmr0.h"
#include "serial_device.h"
#include <stdio.h>

#include "pwm_tmr2.h"
//volatile uint8_t estat=1;
//volatile uint32_t n;
volatile int flag=0;
//volatile double f0,err;
//uint32_t fs,fclk;

//uint8_t n=0;


#define N 196

/********************************************
                Clock measurement
--------------------------------------------	-
This program takes samples from the analog
input A5 of the Ardiuno UNO Board.
The conversion time is 13/fadc_clk, fadc_clk=16e6/adc_pre.

With the 8 most significant bits
it updates the PWM output at terminal ~3 or ~11.

An interrupt is activated each 50us (ocr0a=99, tmr0_pre=8)

The digital output PD4 is set to '1'
at the beginning of the ISR
and to '0' at the end.

26 September 2014
*********************************************/
static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,_FDEV_SETUP_WRITE);
static int uart_putchar(char c, FILE *stream){
  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
}

int pinLED = 13;

void setup(){
  setup_ADC(1,5,16);//(adc_input,v_ref,adc_pre)
  //adc_input 0-5 (default=5),8 Tª, 14 1.1V, 15 GND 
  //v_ref 0 (AREF), 1(1.1V), default=5 (5V)
  //adc_pre 2,4,8,16(default),32,64,128
  setup_tmr0(250,8);//(ocr0a, tmr0_pre)
  //tmr0_pre 1,default=8,64,256,1024
  //TMR0=prescaler*(ocr0a+1)*T_clk
  setup_pwm_tmr2(11);//(pwm_out) 3,default=11
  DDRD |=(1<<4);//pin 4 Arduino as an output. It shows sampling period (period) and ISR execution time (pulse wide)
  DDRD |=(1<<5);//pin  Arduino as an output. It shows sampling period (period) and ISR execution time (pulse wide)
  DDRB |= _BV(DDB5); /* set pin 5 of PORTB for output*/
  serial_init();  
  stdout = &mystdout;
  sei();
}

// change_led(): canvia l'estat del LED
void change_led(void)
{
  static int estat=1;
  switch(estat)
    {
    case 1: PORTB |= _BV(PORTB5); break;
      //delay(500);
    case 0: PORTB &= ~_BV(PORTB5); break;
    }
  if (estat==1) estat=0;
  else estat=1;
  }

int main(void){
  setup();
  //printf("main");
  while(1){
    // ---------CALC----------
    if (flag==1){
      PORTD |= (1<<PD5);
      change_led();
      flag=0;
      PORTD &= ~(1<<PD5);
    }
    
  }
}

static uint8_t n=0;

ISR(TIMER0_COMPA_vect){
  PORTD |= (1<<PD4);

  uint8_t Xn=read8_ADC();
  start_ADC();
  n++; //n=0 --> N-1	N--> t=Ts*N =N/Fs
  printf("%c",Xn); // enviem mostra
  //printf("%c",'A'); // enviem mostra
  if (n==N-1){ //arribem a ultima mostra de la finestra
    n=0; // posem a 0 el comptador de la finestra
    flag=1;
  }
  
 // set_pwm_tmr2(value);
  PORTD &= ~(1<<PD4);
}

