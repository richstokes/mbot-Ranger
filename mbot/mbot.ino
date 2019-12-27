#include "MeAuriga.h"

// Scheduler stuff
#define INTERVAL_MESSAGE1 10
#define INTERVAL_MESSAGE2 50
#define INTERVAL_MESSAGE3 15000
#define INTERVAL_MESSAGE4 20000
unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;

void print_time(unsigned long time_millis);

// LED setup
#define TOTALLEDS  12
#define RINGALLLEDS        0

#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring( 0, 12 );
#endif

int led_rgb[] = {0, 0, 0}; // Start with LEDs off
int sensorState = 0; // initial state of line following sensor
int previousState = 1; // previous state of line sensor

// Sensors
MeOnBoardTemp OnBoardTemp(PORT_13);
MeUltrasonicSensor ultraSensor(PORT_7);
MeLineFollower lineFinder(PORT_8);

// Motors
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

// Buzzer
MeBuzzer buzzer;

void setup() 
{
  Serial.begin(115200);
  #ifdef MeAuriga_H
    // 12 LED Ring controller is on Auriga D44/PWM
    led_ring.setpin( 44 );
    // Set all LEDs off
    led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
    led_ring.show();
  #endif

//  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
//  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  // The problem is interrupt happening ALL the time - want it to only do stuff every now and then
  // so that rest of program can run

  // Encoder motor settings
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

//  Encoder_1.setPulse(9);
//  Encoder_2.setPulse(9);
//  Encoder_1.setRatio(39.267);
//  Encoder_2.setRatio(39.267);
//  Encoder_1.setPosPid(0.18,0,0);
//  Encoder_2.setPosPid(0.18,0,0);
//  Encoder_1.setSpeedPid(0.18,0,0);
//  Encoder_2.setSpeedPid(0.18,0,0);

  buzzer.setpin(45); // Buzzer is pin 45 on mBot Ranger
}

void loop() 
{
    // Scheduler run "process 1"
      if(millis() > time_1 + INTERVAL_MESSAGE1){
        time_1 = millis();
        print_time(time_1);
        Serial.println("Process 1!");
        int d = get_distance(); // issue where distance is inacurate if motors running?
//        Serial.print("Distance Value is: ");
//        Serial.println(d);
        led_light(d / 2); // Set LEDs based on distance
        
        // Update sensorState from line follower
        sensorState = read_line();
//        Serial.println(sensorState);
    }

   // Scheduler run "process 2"
    if(millis() > time_2 + INTERVAL_MESSAGE2){
        time_2 = millis();
        print_time(time_2);
        Serial.println("Process 2!");
        // Apply latest settings to motor
        if(sensorState != previousState){
          Serial.println("Line sensor change detected - updating motors!");
          follow_line(sensorState);
          previousState = sensorState;
        }
        
//        Encoder_1.loop();
//        Encoder_2.loop();
//        isr_process_encoder1();
//        isr_process_encoder2();
    }
}


int read_line()
  {
    int sensorState = lineFinder.readSensors();
    return sensorState;
  }

  
void follow_line(int sensorState) // move robot
{
//  int sensorState = lineFinder.readSensors();
  switch(sensorState)
  {
    case S1_IN_S2_IN: 
//      Serial.println("Sensor 1 and 2 are inside of black line - moving FORWARD"); // move forward
      led_green();
      Encoder_1.setMotorPwm(-180);
      Encoder_2.setMotorPwm(180);
      break;
    case S1_IN_S2_OUT: 
//      Serial.println("Sensor 2 is outside of black line - moving LEFT"); // move left
      led_blue();
      Encoder_1.setMotorPwm(-180);
      Encoder_2.setMotorPwm(-160);
      break;
    case S1_OUT_S2_IN: 
//      Serial.println("Sensor 1 is outside of black line - moving RIGHT"); // move right
      led_yellow();
      Encoder_1.setMotorPwm(160);
      Encoder_2.setMotorPwm(180);
      break;
    case S1_OUT_S2_OUT: 
//      Serial.println("Sensor 1 and 2 are outside of black line - STOPPING"); // We're lost
      Encoder_1.setMotorPwm(0);
      Encoder_2.setMotorPwm(0);
      led_red(); led_light(12);
      error_beep();
      break;
    default: 
      Serial.println("Missing data from line sensor - STOPPING");
      Encoder_1.setMotorPwm(0);
      Encoder_2.setMotorPwm(0);
      break;
  }

//  Serial.print("Speed 1:");
//  Serial.print(Encoder_1.getCurrentSpeed());
//  Serial.print(" , Speed 2:");
//  Serial.println(Encoder_2.getCurrentSpeed());
}


int get_temp()
{
  int t;
  t = OnBoardTemp.readValue();
  //  Serial.print("Temp Analog Value is: ");
  //  Serial.println(OnBoardTemp.readAnalog());
  //  Serial.print("Temp Value is: ");
  //  Serial.println(get_temp());
  return t;
}

int get_distance()
{
  int d;
  d = ultraSensor.distanceCm();
  return d;
}

void led_light(int LEDCOUNT) // Light up specified number of LEDs on the ring
{
  int i;
//  Serial.print("LEDs to light up: ");
//  Serial.println(LEDCOUNT);

    // Set all LEDs off
    led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
//    led_ring.show();
        
    for ( i = 1; i <= LEDCOUNT; i++ )
    {
        led_ring.setColor( i, led_rgb[0], led_rgb[1], led_rgb[2]);
        led_ring.show();
    }
}


// Helpers
void isr_process_encoder1(void)
{
  Serial.println("Started encoder 1 ISR");
  
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }

  Serial.println("FINISHED encoder 1 ISR");
}

void isr_process_encoder2(void)
{
  Serial.println("Started encoder 2 ISR");
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
  
  Serial.println("FINISHED encoder 2 ISR");
}

void error_beep()
{
    buzzer.tone(392, 250);
    buzzer.tone(1600, 125);
}

void led_red()
{
//  Serial.println("Setting LED to red");
  led_rgb[0] = 255; 
  led_rgb[1] = 0; 
  led_rgb[2] = 0; // turn LEDs red
}

void led_green()
{
//  Serial.println("Setting LED to green");
  led_rgb[0] = 10; 
  led_rgb[1] = 255; 
  led_rgb[2] = 40; // turn LEDs bright green
}

void led_blue()
{
//  Serial.println("Setting LED to blue");
  led_rgb[0] = 10; 
  led_rgb[1] = 40; 
  led_rgb[2] = 255; // turn LEDs bright green
}

void led_yellow()
{
//  Serial.println("Setting LED to yellow");
  led_rgb[0] = 255; 
  led_rgb[1] = 255; 
  led_rgb[2] = 0; // turn LEDs bright green
}

void print_time(unsigned long time_millis){
    Serial.print("Time: ");
    Serial.print(time_millis/1000);
    Serial.print("s - ");
}
