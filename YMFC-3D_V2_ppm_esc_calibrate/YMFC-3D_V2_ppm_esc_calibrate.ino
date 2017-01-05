///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>                                  //Include the EEPROM.h library so we can store information onto the EEPROM

//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, start;
int receiver_input[5];
//int temp;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

// AA
////////////////////////////////////////////////////////
// PPM Input                                          //
////////////////////////////////////////////////////////
volatile unsigned long last_ppm_clock = 99999;
volatile unsigned long current_ppm_clock = 0;
volatile unsigned long ppm_dt = 0;
volatile boolean ppm_read = true;
volatile boolean ppm_sync = false;
volatile unsigned short ppm_current_channel = 99;
volatile unsigned long ppm_channels[11] = {0,0,0,0,0,0,0,0,0,0,0}; // at most 10 channels (sync channel + 10 = 11)
#define NUMBER_OF_PPM_CHANNELS 4

void ppmRising() {
  ppm_read = false;
    {
      current_ppm_clock = micros();
      ppm_dt = current_ppm_clock - last_ppm_clock;
      if( ppm_dt >= 3500 ) {
        ppm_sync = true;
        ppm_current_channel = 0;
        ppm_channels[ppm_current_channel] = ppm_dt;         
      }
      else {
        if( ppm_sync ) {
          ppm_current_channel++;
          if( ppm_current_channel > NUMBER_OF_PPM_CHANNELS ) ppm_sync = false;
          else ppm_channels[ppm_current_channel] = ppm_dt; 
        }
      }
      last_ppm_clock = current_ppm_clock;   
    }
  ppm_read = true;
}
///////////////////////////////////////////////////
// PPM Input                                     //
///////////////////////////////////////////////////
// AA

//Setup routine
void setup(){  
  // Serial.begin(57600);
  
  //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs
  DDRD |= B11110000;                                 //Configure digital poort 4, 5, 6 and 7 as output
  DDRB |= B00010000;                                 //Configure digital poort 12 as output

// AA removing pin change interrupt begin used for PWM input
// Original PWM input setup   
//  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
//  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
//  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
//  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
//  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change
// AA

// AA PPM input setup
  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);  
// AA PPM   
  
  //Read EEPROM for fast access data
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  
  //Check the EEPROM signature to make sure that the setup program is executed
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(500);
    digitalWrite(12, !digitalRead(12));              //Change the led status to indicate error.
  }
  wait_for_receiver();                               ///Wait until the receiver is active.
  zero_timer = micros();                             //Set the zero_timer for the first loop.
}

//Main program loop
int throttle = 1000;
void loop(){
  receiver_input_channel_3 = convert_receiver_channel(3);    //Convert the actual receiver signals for throttle to the standard 1000 - 2000us

  ///////////////////////////////////////////////////////////////////////////////////////////
  // AA
  // hover mode throttle
  // Added in place of (A)
  //
  // throttle_stick position at 1500 means no change in current thottle
  // throttle_stick > 1500 means increase the throttle
  // throttle_stick < 1500 means decrease the throttle
  // final throttle value is between 1000 and 2000
       if( receiver_input_channel_3 < 1050  && throttle > 1400 ) { throttle -= 50; } // AA fast decrease in throttle
  else if( receiver_input_channel_3 < 1200  && throttle > 1010 ) { throttle -= 10; } // AA medium decrease in throttle
  else if( receiver_input_channel_3 < 1400  && throttle > 1001 ) { throttle -= 1;  } // AA slow decrease in throttle   
  else if( receiver_input_channel_3 > 1600  && throttle < 2000 ) { throttle += 1;  } // AA slow increase in throttle
  else if( receiver_input_channel_3 > 1800  && throttle < 2000 ) { throttle += 10; } // AA medium increase in throttle   
  else if( receiver_input_channel_3 < 1400 ) { throttle = 1000;  }   
  // Serial.println( throttle ); 
  // hover mode throttle
  // AA
  ///////////////////////////////////////////////////////////////////////////////////////////
    
  while(zero_timer + 4000 > micros());                       //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
  PORTD |= B11110000;                                        //Set port 4, 5, 6 and 7 high at once
  timer_channel_1 = throttle + zero_timer;   //Calculate the time when digital port 4 is set low
  timer_channel_2 = throttle + zero_timer;   //Calculate the time when digital port 5 is set low
  timer_channel_3 = throttle + zero_timer;   //Calculate the time when digital port 6 is set low
  timer_channel_4 = throttle + zero_timer;   //Calculate the time when digital port 7 is set low
  
  while(PORTD >= 16){                                        //Execute the loop until digital port 4 to 7 is low
    esc_loop_timer = micros();                               //Check the current time
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low
  }
  
}

//This routine is called every time input 8, 9, 10 or 11 changed state
//ISR(PCINT0_vect){
//  current_time = micros();
//  //Channel 1=========================================
//  if(PINB & B00000001){                                        //Is input 8 high?
//    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
//      last_channel_1 = 1;                                      //Remember current input state
//      timer_1 = current_time;                                  //Set timer_1 to current_time
//    }
//  }
//  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
//    last_channel_1 = 0;                                        //Remember current input state
//    receiver_input[1] = current_time - timer_1;                 //Channel 1 is current_time - timer_1
//  }
//  //Channel 2=========================================
//  if(PINB & B00000010 ){                                       //Is input 9 high?
//    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
//      last_channel_2 = 1;                                      //Remember current input state
//      timer_2 = current_time;                                  //Set timer_2 to current_time
//    }
//  }
//  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
//    last_channel_2 = 0;                                        //Remember current input state
//    receiver_input[2] = current_time - timer_2;                 //Channel 2 is current_time - timer_2
//  }
//  //Channel 3=========================================
//  if(PINB & B00000100 ){                                       //Is input 10 high?
//    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
//      last_channel_3 = 1;                                      //Remember current input state
//      timer_3 = current_time;                                  //Set timer_3 to current_time
//    }
//  }
//  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
//    last_channel_3 = 0;                                        //Remember current input state
//    receiver_input[3] = current_time - timer_3;                 //Channel 3 is current_time - timer_3
//  }
//  //Channel 4=========================================
//  if(PINB & B00001000 ){                                       //Is input 11 high?
//    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
//      last_channel_4 = 1;                                      //Remember current input state
//      timer_4 = current_time;                                  //Set timer_4 to current_time
//    }
//  }
//  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
//    last_channel_4 = 0;                                        //Remember current input state
//    receiver_input[4] = current_time - timer_4;                 //Channel 4 is current_time - timer_4
//  }
//}

//Checck if the receiver values are valid within 10 seconds
void wait_for_receiver(){
  byte zero = 0;                                                                //Set all bits in the variable zero to 0
  while(zero < 15){                                                             //Stay in this loop until the 4 lowest bits are set

    cli();  
      receiver_input[1] = ppm_channels[1] ;
      receiver_input[2] = ppm_channels[2] ;
      receiver_input[3] = ppm_channels[3] ;
      receiver_input[4] = ppm_channels[4] ;      
    sei();
    
    if(receiver_input[1] < 2100 && receiver_input[1] > 900)zero |= 0b00000001;  //Set bit 0 if the receiver pulse 1 is within the 900 - 2100 range
    if(receiver_input[2] < 2100 && receiver_input[2] > 900)zero |= 0b00000010;  //Set bit 1 if the receiver pulse 2 is within the 900 - 2100 range
    if(receiver_input[3] < 2100 && receiver_input[3] > 900)zero |= 0b00000100;  //Set bit 2 if the receiver pulse 3 is within the 900 - 2100 range
    if(receiver_input[4] < 2100 && receiver_input[4] > 900)zero |= 0b00001000;  //Set bit 3 if the receiver pulse 4 is within the 900 - 2100 range
    delay(500);                                                                 //Wait 500 milliseconds
  }
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;
  
  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  // Original PWM input
  // actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function

  // PPM input
  //while(1) {
  //  if( ppm_read && ppm_sync ) {
      cli();  
        actual = ppm_channels[channel] ;
      sei();
  //    break;
  //  }
  //}
  // PPM
  
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel
  
  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

