#include "powerman.h"
#include <Arduino.h>

int lpm_status;

void enter_lpm(int mode){
  switch(mode){
    case LPM_GPSNAV_NOBAILOUT: //this is same as gpsnav, except we also turn the receiver off
      PORTB &= ~(1 << PCONTROL_5V_FET);
    case LPM_NOSIGNAL:
    case LPM_GPSNAV_BAILOUT:
      PORTB &= ~(1 << PCONTROL_9V_FET1);
    break;
  }
}

void exit_lpm(){
  switch(lpm_status){
    case LPM_GPSNAV_NOBAILOUT: //this is same as gpsnav, except we also turn the receiver off
      PORTB |= (1 << PCONTROL_5V_FET);
    case LPM_GPSNAV_BAILOUT:
    case LPM_NOSIGNAL:
      PORTB |= (1 << PCONTROL_9V_FET1);
      break;
  }
}

void init_lpm_tick(){
  TCCR4A = 0;
  TCCR4B = (1 << OCIE4A);
  OCR4A = 62500;
}
