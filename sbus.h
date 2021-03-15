#include <stdint.h>
#include <Arduino.h>

#define SBUS_FRAME_LENGTH 25
#define NUM_SER_BYTES 23
#define NUM_CHANNELS 18
#define NUM_DATA_FRAMES 7
#define NON_CH_FRAMES 2
#define SYSTEM_FRAME 1
#define SBUS_BAUDRATE 100000
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x04
#define SBUS_FOOTER2  0x00
#define SBUS_CH17 0x01
#define SBUS_CH18 0x02
#define CH_MID 992
#define CH_ZERO 192
#define CH_MAX 1792
#define RANGE_MAX 800
#define RANGE_MIN -800
#define RANGE_MID 0
#define SBUS_SERIAL Serial1

//#define DEBUG

void sbus_init();
int get_sbus_data(char bytes[]);
int sbus_process_channels(char serdata[], uint16_t channels[]);
