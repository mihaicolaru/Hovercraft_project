//  BOOMBA 2.0
//  hovercraft code
//  ENGR 290
//  Mihai Olaru
//  Abhinav Batra
//  Argiro Skokos
//  Jonathan Tcharfajian
//  Hugh McKenzie

#include "MPU9250.h"

#define AHRS false  // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

//====================I2C interface====================
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // I2C address

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

//==================== Pin definitions ====================
// com and led
int intPin = 13;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int LED  = 12;  // Set up pin 13 led for toggling

// sensors
int _INT0 = 2; // INT0 pin 2
int _INT1 = 3; // INT1 pin 3

// fans
int thrust_left = 6; //PWM0 pin 9 (thrust 1)
int thrust_right = 9; //PWM1 pin 10 (thrust 2)
int lift = 7;  //PWM0 ON/OFF (lift)

// servo
int servo = 10;

//==================== Variables ====================
// fan speed
int left_speed, right_speed, init_left = 205, init_right = 220;
int _min = 0; //minimum fan speed
int _max = 255; //maximum fan speed

// sensor distances
unsigned long pulse_length;
int distance;//sensor distance
int avg_val = 0; //sensor distance average value

int dist_count = 0; //sensor trigger counter

// servo angle
int angle = 127;

// turn logic
bool turn_side = true;
int turn_count = 0;

// control loop start
bool start_it = true;

//==================== Setup (runs once) ====================
void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(9600);

  while (!Serial) {};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // set up interrupt pin for sensors
  pinMode(_INT0, INPUT);  // INT0 set as input
  digitalWrite(_INT0, LOW);
  pinMode(_INT1, INPUT);  // INT1 set as input
  digitalWrite(_INT1, LOW);

  // set up interrupt pin for fans
  pinMode(thrust_left, OUTPUT);
  analogWrite(thrust_left, 0);
  pinMode(thrust_right, OUTPUT);
  analogWrite(thrust_right, 0);
  pinMode(lift, OUTPUT);
  digitalWrite(lift, LOW);

  // set up interrupt pin for servo
  pinMode(servo, OUTPUT);
  analogWrite(servo, 170);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 detected"));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");
    delay(1000);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    // Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("MPU-9250 not detected, abort!"));
      Serial.flush();
      abort();
    }

    // Get gyro resolution, only need to do this once
    myIMU.getGres();
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

//==================== Update IMU gyro values ====================
void update_IMU() {
  // If intPin goes high, all data registers have new data on interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's this depends on scale being set
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if (SerialDebug)
      {
        Serial.println();
        // Print gyro values in degree/sec
        Serial.print("gyro: "); Serial.println(myIMU.gz);

      }
      myIMU.count = millis();
      digitalWrite(LED, !digitalRead(LED));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
  } // if (AHRS)
}

//==================== Control Script ====================
void loop()
{

  if (start_it) {
    turn_on();
    start_it = false;
  }

  stabilize();

  read_sensor();

  if (distance < 50) {
    dist_count++;
    if (dist_count > 3) {
      if (turn_side) {
        turn_count++;
        turn_servo(turn_side);
        turn_side = false;
      }
      else {
        turn_count++;
        turn_servo(turn_side);
        turn_side = true;
      }
      dist_count = 0;
    }
  }

}

//==================== Turn ====================
void turn_servo(bool side)
{
  if (side) {
    turn_off();
    delay(700);

    //start turning
    analogWrite(servo, 250);
    left_speed = 250;
    right_speed = 160;
    advance();
    hover();

    //check distance
    while ((distance < 150) || (distance > 200)) {
      read_sensor();
    }
    turn_off();

    //compensate
    analogWrite(servo, 20);
    left_speed = 80;
    right_speed = 250;
    advance();
    hover();
    delay(1500);

    turn_off();
    delay(700);

    //go straight
    analogWrite(servo, 170);
    turn_on();
  }
  else {
    turn_off();
    delay(700);

    //start turning
    analogWrite(servo, 20);
    left_speed = 150;
    right_speed = 250;
    advance();
    hover();

    //check distance
    while ((distance < 170) || (distance > 190)) {
      read_sensor();
    }

    turn_off();
    delay(700);

    //compensate
    analogWrite(servo, 250);
    left_speed = 250;
    right_speed = 80;
    advance();
    hover();
    delay(1500);

    turn_off();
    delay(500);

    //go straight
    analogWrite(servo, 170);
    turn_on();
  }

}

//==================== Sensor distance ====================
void read_sensor() {
  pulse_length = pulseIn(_INT0, HIGH);
  distance = pulse_length * 0.017;

  Serial.print("distance: ");
  Serial.println(distance);

}

//==================== Turn on Lift ====================
void hover() {
  digitalWrite(lift, HIGH);
}

//==================== Turn off Thrust ====================
void stop_thrust() {
  analogWrite(thrust_right, 0);
  analogWrite(thrust_left, 0);
}

//==================== Send initial values to Lift and Thrust ====================
void turn_on() {
  hover();
  left_speed = init_left;
  right_speed = init_right;
  advance();
}

//==================== Turn off Lift and Thrust ====================
void turn_off() {
  digitalWrite(lift, LOW);
  analogWrite(thrust_left, 0);
  analogWrite(thrust_right, 0);
  analogWrite(servo, 170);
}

//==================== Send Thrust speeds ====================
void advance() {
  analogWrite(thrust_right, right_speed);
  analogWrite(thrust_left, left_speed);
}

//==================== Upper/lower fan speed limit ====================
void min_max() {
  if (left_speed > _max) {
    left_speed = _max;
  }
  else if (left_speed < _min) {
    left_speed = _min;
  }

  if (right_speed > _max) {
    right_speed = _max;
  }
  else if (right_speed < _min) {
    right_speed = _min;
  }
}

//==================== Straight line stabilization ====================
void stabilize() {

  update_IMU();

  if (myIMU.gz > 2) {
    angle -= 5;
    if (angle < 5)
    {
      angle = 5;
    }
    analogWrite(servo, angle);
    //delay(5);
    /*
        Serial.print("Positive change in gyro, angle: ");
        Serial.println(angle);
    */
  }
  else if (myIMU.gz < -2) {
    angle += 5;
    if (angle > 250)
    {
      angle = 250;
    }
    analogWrite(servo, angle);
    //delay(5);
    /*
        Serial.print("Negative change in gyro, angle: ");
        Serial.println(angle);
    */
  }
}
