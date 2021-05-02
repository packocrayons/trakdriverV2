#include <Arduino.h>

#define LPM_NOSIGNAL 3
#define LPM_GPSNAV_BAILOUT 2
#define LPM_GPSNAV_NOBAILOUT 1

#define PCONTROL_9V_FET1 PB6
//I don't know if I'll use this for powermanagement or for something much more stupid like a railgun, but we'll see
#define PCONTROL_9V_FET2 PB7
#define PCONTROL_5V_FET PB5

void enter_lpm(int mode);
void exit_lpm();
void init_lpm_tick();
