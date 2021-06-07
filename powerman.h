#include <Arduino.h>

#define LPM_NOSIGNAL 3
#define LPM_REQUESTED_NOVIDEO 2
#define LPM_REQUESTED_KILLALL 1
#define LPM_NONE 0 

#define PCONTROL_VTX_FAN_FET PB6
//I don't know if I'll use this for powermanagement or for something much more stupid like a railgun, but we'll see
#define PCONTROL_RX_FET PB7
#define PCONTROL_CAM_FET PB5

void enter_lpm(int mode);
void exit_lpm();
void init_lpm_tick();
void do_lpm(uint16_t channels[]);
