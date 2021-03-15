#include <stdint.h>
#include <Arduino.h>
#include "speksat.h"

//#define DEBUG
//#define SPEK_BIND 9

/* I'm confused on how the frame contains 7 channels, but there's 8 channels. Are the last two lower refresh rate and alternate? Only hardware will tell */
#if defined(DEBUG)
#define BLINK_PIN 13
void debug_blink(int l){
  for (int i = 0; i < l; ++i){
    digitalWrite(BLINK_PIN, HIGH);
    delay(1000);
    digitalWrite(BLINK_PIN, LOW);
    delay(1000);
  }
  delay(2000);
}
#else
void debug_blink(int l){
  return;
}
#endif

void speksat_init(){
  #if defined(SPEK_BIND)
  speksat_bind(SPEK_BIND);
  #endif
  Serial.begin(1152000, SERIAL_8N1);
}

void speksat_bind(int bind_code){
  pinMode(15, OUTPUT);
  for (int i = 0; i < bind_code; ++i){
    digitalWrite(15, HIGH);
    delay(1);
    digitalWrite(15,LOW);
    delay(1);
  }
}

int get_sat_data(char bytes[]){
  while (Serial.available() > NUM_SER_BYTES){
  	for (int i = 0; i < NUM_SER_BYTES; ++i){
  		bytes[i] = Serial.read();
      for (int j = 0; j < 8; ++j){
        Serial.print((bytes[i] >> j) & 0x1 ? '0' : '1');
      }
      Serial.println();
  	}
   
  	return 0;
  }
  return 1;
}

int speksat_process_channels(char serdata[], uint16_t channels[]){
	int chId;
	uint16_t channel;
  //if (serdata[SYSTEM_FRAME] != DSMX_11MS_2048_SIGNATURE) return 1;
	for (int i = NON_CH_FRAMES; i < (NUM_DATA_FRAMES * 2) + NON_CH_FRAMES; i += 2){ //start at 2, go to 16, by 2s
		channel = ((serdata[i] & 0x07) << 8) + serdata[i + 1]; //2 + 3, 4 + 5, etc
		chId = (serdata[i] >> 3) & 0x7;
    if (chId > NUM_CHANNELS + 1){
      continue; //skip this one? The protocol is vague here.
			//return -1; //this might triggrer on channel 8
		}
		channels[chId] = channel & (0x7FF);
	}
 return 0;
}
