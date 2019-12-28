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
MeRGBLed led_ring( 0, TOTALLEDS );
#endif

int led_rgb[] = {0, 0, 0}; // Start with LEDs off
float j, f, k; // used for LED patterns
int sensorState = 0; // initial state of line following sensor
int previousState = 1; // previous state of line sensor
int d = 12; // holds distance, default value of 12 means we start with LEDs on


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
#endif

  randomSeed(analogRead(2));
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
  startup_colors(); // play RBG LED pattern after all setup done OK
  // Set all LEDs off
  led_light(12); // Light up all LEDs
}

void loop()
{
  // Scheduler run "process 1" -- read line sensor and update LEDs
  if (millis() > time_1 + INTERVAL_MESSAGE1) {
    time_1 = millis();
    print_time(time_1);
//    Serial.println("Running Process 1!");
    // Update sensorState from line follower
    sensorState = read_line();
    // Serial.println(sensorState);
    led_light(d / 2); // Set LEDs based on distance
//    led_light(12); // Light up all LEDs
//    led_light(random(1, 12)); // Light up random LEDs
  }

  // Scheduler run "process 2" -- apply direction changes to motors
  if (millis() > time_2 + INTERVAL_MESSAGE2) {
    time_2 = millis();
    print_time(time_2);
//    Serial.println("Running Process 2!");
    // Apply latest settings to motor
    if (sensorState != previousState) {
      Serial.println("Line sensor change detected - updating motors!");
      follow_line(sensorState);
      previousState = sensorState;
    }
  }

  // Scheduler run "process 3" -- get distance from ultrasonic sensor
  if (millis() > time_3 + INTERVAL_MESSAGE3) {
    time_3 = millis();
    print_time(time_3);
    Serial.println("Running Process 3!");
    d = get_distance(); // issue where reading distance sensor slows down program
//    Serial.print("Distance Value is: ");
//    Serial.println(d);
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
  switch (sensorState)
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

//void speedo(){ // Get current speed
//    Serial.print("Speed 1:");
//    Serial.print(Encoder_1.getCurrentSpeed());
//    Serial.print(" , Speed 2:");
//    Serial.println(Encoder_2.getCurrentSpeed());
//}

void led_light(int LEDCOUNT) // Light up specified number of LEDs on the ring
{
  int i;
    Serial.print("LEDs to light up: ");
    Serial.println(LEDCOUNT);

  // Set all LEDs off
  led_ring.setColor( RINGALLLEDS, 0, 0, 0 );
  //    led_ring.show();

  for ( i = 1; i <= LEDCOUNT; i++ )
  {
    led_ring.setColor( i, led_rgb[0], led_rgb[1], led_rgb[2]);
    led_ring.show();
  }
}


// Helper functions
//void isr_process_encoder1(void)
//{
//  Serial.println("Started encoder 1 ISR");
//
//  if (digitalRead(Encoder_1.getPortB()) == 0)
//  {
//    Encoder_1.pulsePosMinus();
//  }
//  else
//  {
//    Encoder_1.pulsePosPlus();;
//  }
//
//  Serial.println("FINISHED encoder 1 ISR");
//}
//
//void isr_process_encoder2(void)
//{
//  Serial.println("Started encoder 2 ISR");
//  if (digitalRead(Encoder_2.getPortB()) == 0)
//  {
//    Encoder_2.pulsePosMinus();
//  }
//  else
//  {
//    Encoder_2.pulsePosPlus();
//  }
//
//  Serial.println("FINISHED encoder 2 ISR");
//}

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

void color_loop(){ // spin random colours around the LED ring
  for (uint8_t t = 0; t < TOTALLEDS; t++ )
  {
    uint8_t red  = 64 * (1 + sin(t / 2.0 + j / 4.0) );
    uint8_t green = 64 * (1 + sin(t / 1.0 + f / 9.0 + 2.1) );
    uint8_t blue = 64 * (1 + sin(t / 3.0 + k / 14.0 + 4.2) );
    led_ring.setColorAt( t, red, green, blue );
  }
  led_ring.show();
 
  j += random(1, 6) / 6.0;
  f += random(1, 6) / 6.0;
  k += random(1, 6) / 6.0;
}

void startup_colors(){ // spin a few random colours on startup
  for ( int i = 1; i <= random(200, 800); i++ ){
  color_loop();
  }
}

void print_time(unsigned long time_millis) {
//  Serial.print("Time: ");
//  Serial.println(time_millis / 1000);
////  Serial.println("s - ");
}
