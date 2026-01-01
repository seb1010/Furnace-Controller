#include "project.h"

#include "utils/uart/uart_1m.S"




.global read_adc
read_adc:
  push r17
  push r18
  push r19

  in r19, _SFR_IO_ADDR(PORTB)
  sbr r19, (1 << TC_SCLK) // set clock high
  out _SFR_IO_ADDR(PORTB), r19
  cbr r19, (1 << TC_CSB) // set csb low
  out _SFR_IO_ADDR(PORTB), r19

  clr r24
  clr r25

  ldi r18, 7
  first_byte_out:
    cbr r19, (1 << TC_SCLK) // clock low
    out _SFR_IO_ADDR(PORTB), r19
    lsl r25
    sbr r19, (1 << TC_SCLK) // clock high
    out _SFR_IO_ADDR(PORTB), r19
    in r17, _SFR_IO_ADDR(PINA)
    sbrc r17, TC_SO
      inc r25

  dec r18
  brne first_byte_out
  cbr r25, 0xf0 // 4 msb are trash

  ldi r18, 8
  second_byte_out:
    cbr r19, (1 << TC_SCLK) // clock low
    out _SFR_IO_ADDR(PORTB), r19
    lsl r24
    sbr r19, (1 << TC_SCLK) // clock high
    out _SFR_IO_ADDR(PORTB), r19
    in r17, _SFR_IO_ADDR(PINA)
    sbrc r17, TC_SO
      inc r24

  dec r18
  brne second_byte_out

  sbr r19, (1 << TC_CSB) // chip select back high
  out _SFR_IO_ADDR(PORTB), r19

  pop r19
  pop r18
  pop r17
  ret
