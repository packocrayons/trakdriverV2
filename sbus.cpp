#include "sbus.h"
#include <Arduino.h>

void sbus_init(){
	SBUS_SERIAL.begin(SBUS_BAUDRATE, SERIAL_8N2);
//	Serial1.flush();
}

int get_sbus_data(char bytes[]){
  static uint16_t serial_pointer = 0;
	char c;
	while(SBUS_SERIAL.available()){
		c = (char)(SBUS_SERIAL.read() & 0xFF);
		if (serial_pointer == 0){
			if ((c & 0xFF) != SBUS_HEADER){ //we're at zero, if it's not the header, we might lose an entire frame, but we just keep crashing until the next header comes around
				return -1;
			}
//      Serial3.println("h");
			serial_pointer+=1;
		} else {
			if (serial_pointer < (SBUS_FRAME_LENGTH - 1)){
				bytes[serial_pointer - 1] = c;
//        .print(serial_pointer - 1);
//        Serial3.println((int) bytes[serial_pointer] & 0xFF, BIN);
        serial_pointer++;
			} else {
//        Serial3.println(c & 0xFF, HEX);
        serial_pointer = 0;
        if((c & 0xFF) == SBUS_FOOTER || (c & 0x0F) == SBUS_FOOTER2){
//          Serial3.println('f');
          return 0;
        }
        return -2;
			}
		}
	}
  return -3;
}

int sbus_process_channels(char serdata[], uint16_t* channels){
	  channels[0]  = (uint16_t) ((serdata[0] & 0xFF      | serdata[1]  << 8)                         & 0x07FF);
      channels[1]  = (uint16_t) (((serdata[1] & 0xFF)  >> 3  | serdata[2]  << 5)                         & 0x07FF);
      channels[2]  = (uint16_t) (((serdata[2] & 0xFF)  >> 6  | serdata[3]  << 2  | serdata[5] << 10)  & 0x07FF);
      channels[3]  = (uint16_t) (((serdata[4] & 0xFF)  >> 1  | serdata[5]  << 7)                         & 0x07FF);
      channels[4]  = (uint16_t) (((serdata[5] & 0xFF)  >> 4  | serdata[6]  << 4)                         & 0x07FF);
      channels[5]  = (uint16_t) (((serdata[6] & 0xFF)  >> 7  | serdata[7]  << 1  | serdata[9] << 9)   & 0x07FF);
      channels[6]  = (uint16_t) (((serdata[8] & 0xFF)  >> 2  | serdata[9] << 6)                         & 0x07FF);
      channels[7]  = (uint16_t) (((serdata[9] & 0xFF) >> 5  | serdata[10] << 3)                         & 0x07FF);
      channels[8]  = (uint16_t) (((serdata[11] & 0xFF)       | serdata[12] << 8)                         & 0x07FF);
      channels[9]  = (uint16_t) (((serdata[12] & 0xFF) >> 3  | serdata[13] << 5)                         & 0x07FF);
      channels[10] = (uint16_t) (((serdata[13] & 0xFF) >> 6  | serdata[14] << 2  | serdata[16] << 10) & 0x07FF);
      channels[11] = (uint16_t) (((serdata[15] & 0xFF) >> 1  | serdata[16] << 7)                         & 0x07FF);
      channels[12] = (uint16_t) (((serdata[16] & 0xFF) >> 4  | serdata[17] << 4)                         & 0x07FF);
      channels[13] = (uint16_t) (((serdata[17] & 0xFF) >> 7  | serdata[18] << 1  | serdata[20] << 9)  & 0x07FF);
      channels[14] = (uint16_t) (((serdata[19] & 0xFF) >> 2  | serdata[20] << 6)                         & 0x07FF);
      channels[15] = (uint16_t) (((serdata[20] & 0xFF) >> 5  | serdata[21] << 3)                         & 0x07FF);
      /* Channel 17 */
      channels[16] = serdata[22] & 0xFF & SBUS_CH17;
      /* Channel 18 */
      channels[17] = serdata[22] & 0xFF & SBUS_CH18;
      for (int i = 0; i < 4; ++i){
        DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" : "); DEBUG_SERIAL.println(channels[i], DEC);
      }
      return 0; //at the moment, this is blind
}
