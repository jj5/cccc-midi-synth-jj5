#include <Arduino.h>

#include <avr/io.h>
#include <avr/interrupt.h>

// 2024-04-23 jj5 - ASCII control characters we support
//
#define ASCII_BS 8
#define ASCII_CR 13

// 2024-04-23 jj5 - maximum length of a command
//
#define CMD_LEN 64

// 2024-04-23 jj5 - pin assignments
//
int led_pin = 13;
int pwm_pin = 9;

// 2024-04-23 jj5 - LED control commands
//
const char *CMD_ON    = "on";
const char *CMD_OFF   = "off";
const char *CMD_FLASH = "flash";

// 2024-04-23 jj5 - tone control commands
//
const char *CMD_START = "start";
const char *CMD_STOP  = "stop";

// 2024-04-23 jj5 - LED states
//

enum led_state { LED_OFF, LED_ON, LED_FLASH };

volatile enum led_state led_state = LED_OFF;

// 2024-04-23 jj5 - we use timer/counter2 to toggle the LED every 500 ms

volatile int timer_2_counter = 0;

ISR( TIMER2_COMPA_vect ) {
  timer_2_counter++;
  if ( timer_2_counter >= 50 ) {
    timer_2_counter = 0;
    // 2024-04-22 jj5 - toggle the LED every 500 ms
    if ( led_state == LED_FLASH ) {
      int pin = digitalRead( led_pin );
      digitalWrite( led_pin, !pin );
    }
  }
}

void setup_timer_2() {

  // Set up Timer/Counter2 for CTC mode
  TCCR2A = 0x02;  // Set to CTC Mode
  TCCR2B = 0x07;  // Use a prescaler of 1024

  // Calculate OCR2A value for 500 ms delay
  // Timer frequency = 16MHz / 1024 = 15625 Hz
  // Ticks needed = 15625 * 0.5s = 7812.5
  // Since Timer2 is 8-bit, and max value is 255, we use an additional counter
  OCR2A = 156;    // (7812 / 50) - 1 approximately

  TIMSK2 = 0x02;  // Enable Timer2 Compare Match A interrupt
  
  sei();          // Enable global interrupts

}

// 2024-04-23 jj5 - we use timer/counter1 to generate a 261.63 Hz tone (middle C)

void timer_1_stop() {
  TCCR1A = 0;
  TCCR1B = 0;
}

void timer_1_start() {

  // Stop the timer
  timer_1_stop();

  // Set the mode to Fast PWM, TOP in ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Set the prescaler to 8
  TCCR1B |= (1 << CS11);

  // Calculate the value of ICR1 for the desired frequency
  unsigned int icr1 = (16000000 / (8 * 261.63)) - 1;

  ICR1 = icr1;

  // Set OCR1A for a 50% duty cycle
  OCR1A = icr1 / 2;

  // Connect PWM output to pin OC1A (pin 9 on most Arduinos)
  TCCR1A |= (1 << COM1A1);

}

void setup_timer_1() {

  // Set the pin you will use for the PWM output
  pinMode( pwm_pin, OUTPUT );

  timer_1_start();

}

void setup() {
  pinMode( led_pin, OUTPUT );
  setup_timer_2();
  setup_timer_1();
  Serial.begin( 115200 );
  Serial.println( "Ready." );
}

bool is_control_char( char c ) { return c < 32 || c == 127; }

void read_command() {

  int command_index = 0;
  char command_buffer[ CMD_LEN ];

  //char debug_buffer[ CMD_LEN ];

  Serial.print( "Command: " );

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
            Serial.println( "" );
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
              Serial.println( "Unknown command." );
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
          Serial.println( "" );
          Serial.println( "Command too long." );
          Serial.print( "Command: " );
        }
      }
    }
  }
}

void loop() {
  read_command();
  switch ( led_state ) {
    case LED_ON :
      digitalWrite( led_pin, HIGH );
      break;
    case LED_OFF :
      digitalWrite( led_pin, LOW );
      break;
    case LED_FLASH :
      // 2024-04-22 jj5 - this is handled in the interrupt
      break;
  }
}
