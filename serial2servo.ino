/*
   Created by Suraj Chakravarthi Raja at the Valero Lab to drive hand gantry servos.

   CODE ADAPTED FROM FOLLOWING SOURCES:
   - Non-blocking serial reads: https://forum.arduino.cc/t/serial-input-basics/278284/2
   - A strtok example: https://www.best-microcontroller-projects.com/arduino-strtok.html
   - Adafruit servo driver tutorial: https://learn.adafruit.com/adafruit-16-channel-pwm-slash-servo-shield/using-the-adafruit-library
   - Tower Pro SG51R sub-micro servo: https://www.adafruit.com/product/2201
                                      https://www.towerpro.com.tw/product/sg51r/
   - Actuonix L16-R linear servo: https://www.actuonix.com/L16-R-Miniature-Linear-Servo-For-RC-p/l16-r.htm
                     - Datasheet: https://s3.amazonaws.com/actuonix/Actuonix+L16+Datasheet.pdf
   Created 2022-03-02, 16:01 PST.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>

//*****************
// Initializiation
//*****************
// Serial UART setup
//-------------------
#define baudrate      500000

// Servo driver setup
//--------------------
// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVO_FREQ    50 // Analog servos run at ~50 Hz updates
#define MAX_SERVO_ID  15 //
// For Actuonix L16-R linear servosr
#define L16_R_SERVO_ID     15 // Pin/channel number of linear servo (Arduino Pin)

#define L16_R_OFF_PULSE    0 // This is the 'maximum' pulse length count (out of 4096)
#define L16_R_MIN_PULSE  204 // This is the 'minimum' pulse length count (out of 4096)
#define L16_R_MAX_PULSE  386 // This is the 'maximum' pulse length count (out of 4096)

#define L16_R_MIN_LENGTH  15 // when pulse length = 1 ms (fully retracted)
#define L16_R_MAX_LENGTH 140 // when pulse length = 2 ms (fully extended)

// For Tower Pro SG51R servos
#define SG51R_OFF_PULSE    0 // This is the 'maximum' pulse length count (out of 4096)
#define SG51R_MIN_PULSE  130 // This is the 'minimum' pulse length count (out of 4096)
#define SG51R_MAX_PULSE  468 // This is the 'maximum' pulse length count (out of 4096)

#define SG51R_MIN_ANGLE  -90 // when pulse length = ~0.75ms
#define SG51R_REF_ANGLE    0 // when pulse length = ~1.50ms
#define SG51R_MAX_ANGLE   90 // when pulse length = ~2.25ms

// For setting PWM by microseconds using the servoDriver.writeMicroseconds(channel, microsecs) function
// #define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
// #define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40, Wire);
//uint8_t                 servonum  = 0; // our servo # counter

// Non-blocking serial read
//--------------------------
const byte  numChars                    = 32;
char        receivedChars[numChars];
boolean     newData                     = false;

// Parsing string to servo data
//------------------------------
uint8_t       servoID     = 0;
int           servoValue  = 0;

// Setup Arduino
//---------------
void setup() {
  // Initialize Adafruit servo driver library
  servoDriver.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  servoDriver.setOscillatorFrequency(27000000);
  servoDriver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(500);

  // Power off all servos upon reboot
  for (unsigned i = 0; i <= MAX_SERVO_ID; i++) {
    servoDriver.setPWM(i, 0, 0);
  }

  // Initialize serial interface
  Serial.begin(baudrate);
  delay(10);
  Serial.println("<Servos ready>");
}

// Serial to servo updates
//-------------------------
void loop() {
  // Receive string
  serial_read_delimited();

  // Parse string to float data
  // float servoAngle = atof(receivedChars);
  if (newData == true) {
    str2ServoValue();
    if  (servoID == L16_R_SERVO_ID) { // move hand gantry linear actuator servo
      servoDriver.setPWM(servoID, 0, servoValue);
    } else { // move Tower Pro SG51R angular servo
      servoDriver.setPWM(servoID, 0, servoValue);
    }
  }
}

// Support function definitions
//------------------------------
void serial_read_delimited()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc == startMarker) {
      ndx = 0;
      recvInProgress = true;
    }
    else if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) { // if too many characters input, perform truncation
          ndx = numChars - 1;
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          newData = true;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        newData = true;
        //Serial.println(receivedChars);
      }
    } // end "receive in progress is true" if block
  } // end while loop
}

void str2ServoValue()
{
  char        *token;
  const char  *delimiter = ",";
  unsigned    tokenCount;
  int         temp;
  unsigned    servo_power;

  Serial.print("<"); 
  //Serial.print(receivedChars);
  for (token = strtok(receivedChars, delimiter), tokenCount = 0; token != NULL && tokenCount < 2; token = strtok(NULL, delimiter), tokenCount++) {
    //Serial.print("Token "); Serial.print(tokenCount); Serial.print(": "); Serial.println(token);

    temp = atoi(token);
    switch (tokenCount) {
      case 0:
        servoID = ((temp > MAX_SERVO_ID) ? MAX_SERVO_ID : ((temp < 0) ? 0 : temp)); // limit servo IDs betwen 0 and 15
        Serial.print(servoID);
        Serial.print(","); 
        break;
      
      case 1:
        servo_power = strcmp(token, "x");
        if (servoID == L16_R_SERVO_ID) { // input validation for linear servo in channel 0
          if (servo_power == 0) {
            servoValue = L16_R_OFF_PULSE;
          } else {
            servoValue = (temp > L16_R_MAX_LENGTH) ? L16_R_MAX_LENGTH : ((temp < L16_R_MIN_LENGTH) ? L16_R_MIN_LENGTH : temp);
            Serial.print(servoValue);
            servoValue = map(servoValue, L16_R_MIN_LENGTH, L16_R_MAX_LENGTH, L16_R_MIN_PULSE, L16_R_MAX_PULSE);
          }
        } else { // input validation for angular servos
          if (servo_power == 0) {
            servoValue = SG51R_OFF_PULSE;
          } else {
            servoValue = (temp > SG51R_MAX_ANGLE) ? SG51R_MAX_ANGLE : ((temp < SG51R_MIN_ANGLE) ? SG51R_MIN_ANGLE : temp);
            Serial.print(servoValue);
            servoValue = map(servoValue, SG51R_MIN_ANGLE , SG51R_MAX_ANGLE , SG51R_MIN_PULSE, SG51R_MAX_PULSE);
          }
        }
        break;
      
      default:
        break;
    }
  }
  newData = false;
  Serial.print(">");
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  servoDriver.setPWM(n, 0, pulse);
}
