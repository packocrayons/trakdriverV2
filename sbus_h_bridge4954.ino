//#include "speksat.h"
#include "sbus.h"
#include "powerman.h"
//#include <stdint.h>
/*Hello I'm brydon gibson and ~~I cheated and used someone else's library but I promise I did all the other stuff myself~~ fuck it I'll do it myself*/

#define RX_GET_DATA get_sbus_data
#define RX_PROCESS_CHANNELS sbus_process_channels
#define RX_INIT sbus_init

#define DIR_FORWARD 0
#define DIR_REVERSE 1

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define NUM_MOTORS 2
#define NUM_USED_CHANNELS 4
#define LED_CHANNEL 3

#define ADMUX_VAL 0000

#define STATUS_LED_PIN PA0

//mixing is done in the channel order (TAER, but in this case TA[SW1]) and is a (float) multiplier of how much that channels' variance from 0 affects the motor (basic multiwii mixing)
//TODO: read the betaflight code because those guys actually know how to write code
#define M0_MMIX {1, 1, 0, 0}
#define M1_MMIX {-1, 1, 0, 0}
#define L_MMIX { 0, 0, 1, 0}

int ledToggle = 1;
int interrupted = 0;

uint8_t ledCNTR = 0;

typedef struct motor{
  int in1;
  int in2;
  uint16_t spd;
  int dir;
  float mix[NUM_USED_CHANNELS];
} motor;

char serdata[NUM_SER_BYTES];
uint16_t channels[NUM_CHANNELS];

int milliseconds = 0;
int millis_tick = 0;

motor motors[2];
motor led; //cheating so I don't have to keep multiple copies of data

void init_timers(){
  TCCR3A = 0;
  TCCR3B = (1 << CS30); //8 clock cycles to process the overflow interrupt. might get some jitter
  TIMSK3 = (1 << TOIE3) | (1 << OCIE3A) | (1 << OCIE3B) | (1 << OCIE3C);
  init_lpm_tick();
}

void init_adc(char admux_val){
  ADMUX = (1 << REFS0) | ADMUX_VAL;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //slowest clock, not interested in speed
}

void setup() {
//  OSCCAL = 0x63;
  motors[0] = {.in1=PD7, .in2=PD6, .spd = 0, .dir = DIR_FORWARD, .mix = M0_MMIX};
  motors[1] = {.in1=PD4, .in2=PD5, .spd = 0, .dir = DIR_FORWARD, .mix = M1_MMIX};
  led = {.in1=PB4, .in2=0, .spd=0, .dir=DIR_FORWARD, .mix=L_MMIX};   
  //protip - make the LED driver a "motor" with high_PFET == the dim pin and set dir to forward. spd is then LED brightness
  for (int i = 0; i < 2; ++i){
    DDRD |= (1 << motors[i].in1) | (1 << motors[i].in2); //only works for motors on portd
  };
  DDRB |= (1 << led.in1);
  DDRA |= (1 << STATUS_LED_PIN);
   
  cli();
  init_timers();
  init_adc(ADMUX_VAL);
  RX_INIT();
  sei();
  Serial2.begin(115200);
  Serial2.println("beep boop, I'm a tank");
  PORTA &= ~(1 << STATUS_LED_PIN);
  delay(1000);
  PORTA |= (1 << STATUS_LED_PIN);
  delay(1000);
  PORTA &= ~(1 << STATUS_LED_PIN);  
}

void do_millis_tick(){
  milliseconds++;
  
}

void mixmotor(motor* m, uint16_t channels[]){
  int t = (channels[0] - CH_MID) * m->mix[0];
  for (int i = 1; i < NUM_USED_CHANNELS; ++i){
    t += (((channels[i] - CH_MID)) * m->mix[i]);
    if (t > RANGE_MAX) t = RANGE_MAX;
    if (t < RANGE_MIN) t = RANGE_MIN;
  }
  if (t < 0) {
    m->dir = DIR_REVERSE;
  } else {
    m->dir = DIR_FORWARD;
  }
  m->spd = map(abs(t), 0, RANGE_MAX, 0, 65535);
}

void mixmotors(motor m[], int len, uint16_t channels[]){
	int t;
	float mixsum;
	for (int i = 0; i < len; ++i){
		t = 0;
		mixsum = 0;
		for (int c = 0; c < NUM_USED_CHANNELS; ++c){
			mixsum += m[i].mix[c];
			t += (m[i].mix[c] * (channels[c] - CH_MID));
		}
		if (t < 0){
			m[i].dir = DIR_REVERSE;
		} else {
			m[i].dir = DIR_FORWARD;
		}
		m[i].spd = map(abs(t), 0, ((CH_MAX / 2) * mixsum), 0, 255);
	}
}

int r;
int lastGoodPacketTime;
void loop() {
  r = RX_GET_DATA(serdata);
  if (lastGoodPacketTime - milliseconds > 4000){
    if (r) {
      PORTA |= (1 << STATUS_LED_PIN);
      enter_lpm(LPM_NOSIGNAL);
    } else {
      exit_lpm();
    }
  }
  if (!r){ // 0 is success
    PORTA &= ~(1 << STATUS_LED_PIN);
    lastGoodPacketTime = milliseconds;
    (RX_PROCESS_CHANNELS(serdata, channels));
    mixmotor(&motors[0], channels);
    mixmotor(&motors[1], channels);
    mixmotor(&led, channels);
    do_lpm(channels);    
  }
  if (millis_tick){
    do_millis_tick();
    millis_tick--;
  }
}

ISR(TIMER3_COMPA_vect){
  PORTD &= ~((1 << motors[0].in1) | (1 << motors[0].in2));//direction doesn't matter here, wasting a GPIO write but it's probably just as fast as a compare
}

ISR(TIMER3_COMPB_vect){
  PORTD &= ~((1 << motors[1].in1) | (1 << motors[1].in2));//direction doesn't matter here, wasting a GPIO write but it's probably just as fast as a compare
}

ISR(TIMER3_COMPC_vect){
 PORTB &= ~(1 << led.in1); 
}
//
ISR(TIMER3_OVF_vect){
 for (int i = 0; i < NUM_MOTORS; ++i){
  if (motors[i].spd > 0){
    if (motors[i].dir == DIR_FORWARD){
      PORTD |= (1 << motors[i].in1);
    } else {
      PORTD |= (1 << motors[i].in2);
    }
  } 
 }
 PORTB |= (1 << led.in1);
 OCR3A = motors[0].spd;
 OCR3B = motors[1].spd;
 OCR3C =  led.spd;
}

ISR(TIMER4_COMPA_vect){
  millis_tick++; //let's hope [company redacted] doesn't pull lawyers on my patented queued-IST algorithm
  TCNT4H = 0;
  TCNT4L = 0;
}
