// OSH orbital shaker: www.black-glass.com

// TYVM Mayhew Labs: www.mayhewlabs.com/products/rotary-encoder-led-ring  
// TYVM Mike McCauley: www.open.com.au/mikem/arduino/AccelStepper/

#include <AccelStepper.h>
          
// ####### Encoder Variables #######
    //These are the pins that will talk to the shift register through SPI
  #define SDI    11    // Data
  #define CLK    12    // Clock
  #define LE     13    // Latch
    //These are the rotary encoder pins A, B, and switch
  #define ENC_A    8
  #define ENC_B    9
  #define ENC_SW   10
  #define ENC_PORT PINB   //The port that the rotary encoder is on (see http://www.arduino.cc/en/Reference/PortManipulation)
    // Global variables
  int scaledCounter = 0;  // The LED output is based on a scaled veryson of the rotary encoder counter
  int sequenceNumber = 0; // The output sequence for the LEDs
  int incomingByte = 0;   // Serial input to select LED output sequence
  int switch_spool = 0;   // Counts the loops where switch is in LOW state ... used to power off machine
  int power_state = LOW;    // Stores current power state
    // 16 LEDs on the ring.... each line has 16 bits with the 1 being on and 0 being off ... for that LED
  unsigned int sequence[16] = {
    0b1000000000000000, // This has a single LED turned on
    0b1000000000000001, // This has the fist and last LEDs on and the rest off.  
    0b1000000000000011,
    0b1000000000000111,
    0b1000000000001111,
    0b1000000000011111,
    0b1000000000111111,
    0b1000000001111111,
    0b1000000011111111,
    0b1000000111111111,
    0b1000001111111111,
    0b1000011111111111,
    0b1000111111111111,
    0b1001111111111111,
    0b1011111111111111,
    0b1111111111111111 // This is all LEDs turned on
  };
  
// ####### Stepper Driver Variables #######
  AccelStepper myStepper( 1, 6, 7 ); // initialize AccelStepper in step/dir mode on pins 8 and 9
  const int STEPPER_ENERGIZED_PIN = 5;  // Controls the powered or unpowered state of stepper coils 
  int targetFreq = 0; // What frequency we'd like to accel or deccel to
  int tempFreq = 0; // used to ensure the user has settled on a chosen RPM
  int previousFreq = 0; // TODO: rename this "currentFreq"
  int buttonState = 0;  // variable for storing the RUN_PIN status
    // counts the number of cycles for reading inputs
    // starts at 10000 so all inputs are read on first cycle
  int run_cycle_count = 100000;




void setup() {
// ####### Encoder Setup #######
    //Set SPI pins to output
  pinMode(SDI, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(LE, OUTPUT);
    //Set encoder pins to input, turn internal pull-ups on
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  pinMode(ENC_SW, INPUT);
  digitalWrite(ENC_SW, HIGH);

// ####### Stepper Driver Setup #######
    // initialize input pins
  pinMode( STEPPER_ENERGIZED_PIN, OUTPUT );
  myStepper.setAcceleration( 200.0 );
  
// ####### General Setup #######
  Serial.begin( 115200 );
    // Ensure LEDs are off by default
  push_to_led_encoder( 0b0000000000000000 );
}


void loop()
{
  power_state = read_encoder_button ( power_state, previousFreq );
  static uint8_t counter = 0;      // this variable will be changed by encoder input
  int8_t tmpdata;
  tmpdata = read_encoder();
  
  // If the encoder has moved, modify the RPMs
  // And reflect in the LED encoder
  if(tmpdata && power_state == HIGH ) {
    counter += tmpdata * 4;      
    scaledCounter = map(counter, 0, 255, 0, 16);
    
    // Light up LEDs based on encoder position and movement along the sequence array
    push_to_led_encoder( sequence[scaledCounter] );
    
    // targetFreq starts at zero
    // NOTE: @ targetFreq == 750rpm we begin to miss steps even w/o load
    targetFreq = scaledCounter * 50 ;
    
    print_formatted_output ( "Target Freq: ", targetFreq );    
  }
  

  
  if ( power_state == HIGH ) {
    ////// handle frequency of input pin reads
    digitalWrite( STEPPER_ENERGIZED_PIN, HIGH );
    if ( run_cycle_count == 10000 ) {
     
            ////// handle change in shake frequency and speed maintain
      // If targetFreq has remained the same for the last 4 passes ( at 10000 flops / pass )
      // we know the user has settled on a given RPM
      if ( targetFreq == tempFreq ){
        // if the frequency has changed, slow it to zero
        // store it in the variables
        if ( previousFreq != targetFreq ) {
          Serial.println( "Running slow_to_zero bc freq change" );
          // This prevents bashing up our shaken samples with twitchy changes in speed
          slow_to_zero(previousFreq);
          myStepper.setMaxSpeed( targetFreq );
          previousFreq = targetFreq;
        } else if ( targetFreq > 0 && previousFreq == targetFreq ) {
          Serial.println("adding steps");
          myStepper.move( ( targetFreq * 5 ) ); 
        }
        
      } else if ( targetFreq != tempFreq ){
        tempFreq = targetFreq;
      }

      run_cycle_count = 0;
    }
    
    myStepper.run();
    run_cycle_count++;
  } 
  
}


// ################## Functions ######################

void push_to_led_encoder ( int led_binary_sequence ){
  digitalWrite(LE,LOW);
  shiftOut(SDI,CLK,MSBFIRST,( led_binary_sequence >> 8));    // High byte first
  shiftOut(SDI,CLK,MSBFIRST, led_binary_sequence );          // Low byte second
  digitalWrite(LE,HIGH);
}

void print_formatted_output ( String attrib , int num ){
  Serial.print( attrib );
  Serial.print( num );
  Serial.print( "\n" );
}

// Check the switch a number of times to ensure signal from noise ( caused by momentary switch bounce )
int read_encoder_button( int ps, int pf ) {
  if ( digitalRead( ENC_SW ) == LOW ){
    if ( switch_spool == 4000 ){
      if ( ps == 1 ){
        Serial.println(" POWERING OFF ");
        push_to_led_encoder( 0b0000000000000000 ); // Extinguish all LEDs
        slow_to_zero( pf );
        targetFreq = 0;
        scaledCounter = 0;
        ps = 0;
      } else if ( ps == 0 ){
        Serial.println(" POWERING UP ");
        push_to_led_encoder( 0b1000000000000000 ); // Light up power LED
        ps = 1;
      }
    }
    switch_spool++;
    //Serial.println( switch_spool );
  } else if ( digitalRead( ENC_SW ) == HIGH ) {
    switch_spool = 0;
  }
  return ps;
}

// Returns change in encoder state (-1,0,1)
// www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino
int8_t read_encoder() {
  int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

// Slows the stepper and powers off the coils
void slow_to_zero( int pf ) {
  if ( myStepper.speed() > 0 ) {
    myStepper.moveTo( myStepper.currentPosition() + ( pf * 2 ) );
    // Blocks while it slows to zero
    while ( myStepper.speed() > 0 )
      myStepper.run();
    Serial.println("stopped");
    digitalWrite( STEPPER_ENERGIZED_PIN, LOW );
    delay( 1000 );
  }
}

void stepper_status ( int f ) {
  Serial.print( f);
  Serial.print( " " );
  Serial.print( myStepper.distanceToGo() );
  Serial.print("\n");
}

