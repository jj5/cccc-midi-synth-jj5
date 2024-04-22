#include <Arduino.h>

#include <avr/io.h>
#include <avr/interrupt.h>

// 2024-04-23 jj5 - baud rate for serial communication
//
#define BAUD_RATE 115200

// 2024-04-23 jj5 - ASCII control characters we support
//
#define ASCII_BS 8
#define ASCII_CR 13

// 2024-04-23 jj5 - maximum length of a command
//
#define CMD_LEN 64

// 2024-04-23 jj5 - pin assignments
//
#define LED_PIN 13
#define PWM_PIN 9

// 2024-04-23 jj5 - strings
//
const char *STR_EMPTY = "";
const char *STR_READY = "Ready.";
const char *STR_COMMAND_PROMPT = "Command: ";
const char *STR_COMMAND_TOO_LONG = "Command too long.";
const char *STR_UNKNOWN_COMMAND = "Unknown command.";

// 2024-04-23 jj5 - LED control commands
//
const char *CMD_ON    = "on";
const char *CMD_OFF   = "off";
const char *CMD_FLASH = "flash";

// 2024-04-23 jj5 - tone control commands
//
const char *CMD_START = "start";
const char *CMD_STOP  = "stop";

//
// 2024-04-23 jj5 - LED states
//

enum led_state { LED_OFF, LED_ON, LED_FLASH };

volatile enum led_state led_state = LED_OFF;

//
// 2024-04-23 jj5 - we use timer/counter2 to toggle the LED every 500 ms
//

volatile int timer_2_counter = 0;

ISR( TIMER2_COMPA_vect ) {
  timer_2_counter++;
  if ( timer_2_counter >= 50 ) {
    timer_2_counter = 0;
    // 2024-04-22 jj5 - toggle the LED every 500 ms
    if ( led_state == LED_FLASH ) {
      int pin = digitalRead( LED_PIN );
      digitalWrite( LED_PIN, !pin );
    }
  }
}

void setup_timer_2() {

  // 2024-04-23 jj5 - set up Timer/Counter2 for CTC mode

  // 2024-04-23 jj5 - set to CTC Mode
  //
  TCCR2A = 0x02;

  // 2024-04-23 jj5 - use a prescaler of 1024
  //
  TCCR2B = 0x07;  

  // 2024-04-23 jj5 - to calculate OCR2A value for 500 ms delay:
  // timer frequency = 16MHz / 1024 = 15625 Hz
  // ticks needed = 15625 * 0.5s = 7812.5
  // (7812 / 50) - 1 = 156 approximately
  //
  OCR2A = 156;

  // 2024-04-23 jj5 - enable Timer2 Compare Match A interrupt
  //
  TIMSK2 = 0x02;
  
  // 2024-04-23 jj5 - enable global interrupts
  //
  sei();

}

//
// 2024-04-23 jj5 - we use timer/counter1 to generate a 261.63 Hz tone (middle C)
//

void timer_1_stop() {
  TCCR1A = 0;
  TCCR1B = 0;
}

void timer_1_start() {

  // 2024-04-23 jj5 - stop the timer
  //
  timer_1_stop();

  // 2024-04-23 jj5 - set the mode to Fast PWM, TOP in ICR1
  //
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // 2024-04-23 jj5 - set the prescaler to 8
  //
  TCCR1B |= (1 << CS11);

  // 2024-04-23 jj5 - calculate the value of ICR1 for the desired frequency (261.63 Hz, middle C)
  //
  unsigned int icr1 = (16000000 / (8 * 261.63)) - 1;
  ICR1 = icr1;

  // 2024-04-23 jj5 - set OCR1A for a 50% duty cycle
  //
  OCR1A = icr1 / 2;

  // 2024-04-23 jj5 - connect PWM output to pin OC1A (pin 9 on most Arduinos)
  //
  TCCR1A |= (1 << COM1A1);

}

void setup_timer_1() {
  timer_1_start();
}

void setup() {
  pinMode( LED_PIN, OUTPUT );
  pinMode( PWM_PIN, OUTPUT );
  setup_timer_2();
  setup_timer_1();
  Serial.begin( BAUD_RATE );
  Serial.println( STR_READY );
}

bool is_control_char( char c ) { return c < 32 || c == 127; }

void read_command() {

  int command_index = 0;
  char command_buffer[ CMD_LEN ];

  //char debug_buffer[ CMD_LEN ];

  Serial.print( STR_COMMAND_PROMPT );

  for ( ;; ) {

    if ( Serial.available() > 0 ) {

      char chr = Serial.read();
      //int ord = chr;

      if ( is_control_char( chr ) ) {

        //sprintf( debug_buffer, "The ASCII code is control character %d.\n", ord );

        switch ( chr ) {
          case ASCII_BS :
            if ( command_index > 0 ) {
              command_index--;
              Serial.print( chr );
            }
            break;
          case ASCII_CR :
            Serial.println( STR_EMPTY );
            command_buffer[ command_index ] = '\0';
            command_index = 0;
            if ( strcmp( command_buffer, CMD_ON ) == 0 ) {
              led_state = LED_ON;
            }
            else if ( strcmp( command_buffer, CMD_OFF ) == 0 ) {
              led_state = LED_OFF;
            }
            else if ( strcmp( command_buffer, CMD_FLASH ) == 0 ) {
              led_state = LED_FLASH;
            }
            else if ( strcmp( command_buffer, CMD_START ) == 0 ) {
              timer_1_start();
            }
            else if ( strcmp( command_buffer, CMD_STOP ) == 0 ) {
              timer_1_stop();
            }
            else {
              Serial.println( STR_UNKNOWN_COMMAND );
            }
            return;
        }
      }
      else {

        //sprintf( debug_buffer, "The ASCII code of '%c' is %d.\n", chr, ord );

        Serial.print( chr );

        command_buffer[ command_index++ ] = chr;

        if ( command_index >= CMD_LEN ) {
          command_index = 0;
          Serial.println( STR_EMPTY );
          Serial.println( STR_COMMAND_TOO_LONG );
          Serial.print( STR_COMMAND_PROMPT );
        }
      }
    }
  }
}

void loop() {
  read_command();
  switch ( led_state ) {
    case LED_ON :
      digitalWrite( LED_PIN, HIGH );
      break;
    case LED_OFF :
      digitalWrite( LED_PIN, LOW );
      break;
    case LED_FLASH :
      // 2024-04-22 jj5 - this is handled in the interrupt handler for timer/counter2
      break;
  }
}
