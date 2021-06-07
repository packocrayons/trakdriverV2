#include "powerman.h"
#include <Arduino.h>

int lpm_status;

void enter_lpm(int mode){
  switch(mode){
    case LPM_REQUESTED_KILLALL:
      PORTB &= ~(1 << PCONTROL_RX_FET);
    case LPM_NOSIGNAL:
      PORTB &= ~(1 << PCONTROL_CAM_FET);
    case LPM_REQUESTED_NOVIDEO:
      PORTB &= ~(1 << PCONTROL_VTX_FAN_FET);
      break;
  }
}

void exit_lpm(){
  //always just turn everything back on. Worst case the next enter lpm cycle will catch it
      PORTB |= (1 << PCONTROL_RX_FET);
      PORTB |= (1 << PCONTROL_CAM_FET);
      PORTB |= (1 << PCONTROL_VTX_FAN_FET); 
}

void init_lpm_tick(){
  TCCR4A = 0;
  TCCR4B = (1 << OCIE4A);
  OCR4A = 62500;
}

void do_lpm(uint16_t channels[]){
  static int last_lpm_mode = LPM_NONE;
  //gonna be lazy and hardcode this for now
  if (channels[3] > 1800){
    enter_lpm(LPM_REQUESTED_KILLALL);
  }
  else if (channels[3] > 1500){
    last_lpm_mode = LPM_REQUESTED_NOVIDEO;
    enter_lpm(LPM_REQUESTED_NOVIDEO);
  }
  if (channels[3] < 1500){
    if (last_lpm_mode != LPM_NONE) exit_lpm();
  }
}
