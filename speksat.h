#include <stdint.h> 

#define NUM_SER_BYTES 16
#define NUM_CHANNELS 8
#define NUM_DATA_FRAMES 7
#define NON_CH_FRAMES 2
#define SYSTEM_FRAME 1
#define DSMX_11MS_2048_SIGNATURE 0xB2

void speksat_init();
void speksat_bind(int bind_code);
int get_sat_data(char bytes[]);
int speksat_process_channels(char serdata[], uint16_t channels[]);
void debug_blink(int l);
