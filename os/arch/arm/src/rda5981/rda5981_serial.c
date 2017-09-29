#include "rda5981_serial.h"

typedef  unsigned char uint8_t;
#define RDA_AHB0_BASE         (0x40000000UL)
#define RDA_UART0_BASE        (RDA_AHB0_BASE + 0x12000)
#define UART_TXH 0x0
#define myputreg8(v, a)        (*(volatile uint8_t *)(a) = (v))


void rda_printint(int input)
{ 
  char str[12];
  char str2[12];
  int i = 0;
  unsigned int n = input;
  while(n)
  {
    str[i++] = n % 10 + '0';
    n/=10;
  }

  int j;
  int k = 0;
  for(j = i-1; j>=0; j--)
  {
      str2[k++] = str[j];
  }
  str2[k++] = '\0';

  rda_printf(str2); 
}

void uart_send_byte(unsigned char ch) 
{
   myputreg8(ch, RDA_UART0_BASE + UART_TXH); 

}

int up_putc(int ch) 
{
    uart_send_byte(ch);
    return 0;

}

void rda_printfc(unsigned char *s) 
{
    uart_send_byte(*s);
}

void rda_printf(char *s) 
{
    while(*s)
    { 
          
        if (*s == '\n')
            uart_send_byte('\r');

        uart_send_byte(*s);
        s++;
    }   
}
