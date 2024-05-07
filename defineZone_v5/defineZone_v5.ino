#include <Wire.h>
//#include <MPU6050.h>
#include <math.h>

#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// LED pin for indicating work status
const int WORK_LED = 2; 

// LED pin for indicating brake status
const int BRAKE_LED = 3; 

// LED pin for indicating setting status
const int SETTING_LED = 4; 

// Button pin for resetting the system
const int RESET_BUTTON = 5; 

// Button pin for setting the system
const int SET_BUTTON = 6; 

// Alpha value for complementary filter
double ALPHA = 0.75; 

// Accelerometer angles in X and Y directions
double accAngleX, accAngleY; 

// Gyroscope angles in X, Y and Z directions
double gyroAngleX, gyroAngleY, gyroAngleZ; 

// Roll, pitch and yaw angles and their zero values
double roll, pitch, rollZero, pitchZero, yaw, yawZero;

// Combined value of roll, pitch and yaw
double combined;

// Reset value for yaw angle
double yawReset=0;

const double  offsetPitch=0; //not calculated data
const double  offsetRoll=0;    //not calculated data 
const double  offsetYaw=0;     //not calculated data

// Preset maximum and minimum values for pitch, roll, and yaw
const double PRESET_PITCH_MAX=80;
const double PRESET_PITCH_MIN=-80;
const double PRESET_ROLL_MAX=80;
const double PRESET_ROLL_MIN=-80;
const double PRESET_YAW_MAX=80;
const double PRESET_YAW_MIN=-80;

// Preset maximum combined value
const double PRESET_COMBINED_MAX=85;

// Final values for pitch, roll, and yaw after calculations
double pitch_final, roll_final, yaw_final;

// Maximum and minimum values for pitch, roll, and yaw after adjustments
double PITCH_MAX, ROLL_MAX, YAW_MAX;
double PITCH_MIN, ROLL_MIN, YAW_MIN;

// Maximum combined value after adjustments
double COMBINED_MAX;

// Variables to hold time values for calculations
double previousTime;
double currentTime; 
double elapsedTime;

// Boolean variable to check if the chainsaw should be running
bool chainsawRunning;

enum State {
  NORMAL,
  RESET,
  SET,
  EMERGENCY,
  STOP
};

void updateState(State newState) {
  currentState = newState;
}

void setup()
 {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
//
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-156);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(-14);
    mpu.setXAccelOffset(-3699);
    mpu.setYAccelOffset(-2519);
    mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

    }
  // Setze die Pins für die LEDs als Ausgänge
  pinMode(WORK_LED, OUTPUT);
  pinMode(BRAKE_LED, OUTPUT);
  pinMode(SETTING_LED, OUTPUT);

  // Setze die Pins für die Taster als Eingänge
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(SET_BUTTON, INPUT_PULLUP);

  updateState(NORMAL);
  blink();
  chainsawRunning=false;
}

void checkAndUpdate(float &currentMax, float &currentMin, float finalValue) {
  if(finalValue > currentMax) {
    currentMax = finalValue;
  }
  if(finalValue < currentMin) {
    currentMin = finalValue;
  }
  COMBINED_MAX = abs(pitch_final) + abs(yaw_final) + 1.5 * abs(roll_final);
}

void checkValues() {
  checkAndUpdate(PITCH_MAX, PITCH_MIN, pitch_final);
  checkAndUpdate(ROLL_MAX, ROLL_MIN, roll_final);
  checkAndUpdate(YAW_MAX, YAW_MIN, yaw_final);
}

void checkButtonAndSetState(int button, int state) {
  if(digitalRead(button) == LOW) {
    updateState(state);
  }
}

void checkValueAndSetEmergency(float value, float min, float max, const char* message) {
  if(chainsawRunning && (value < min || value > max)) {
    Serial.println(message);
    updateState(EMERGENCY);
  }
}

void loop()
{

  checkButtonAndSetState(RESET_BUTTON, RESET);
  checkButtonAndSetState(SET_BUTTON, SET);

  checkValueAndSetEmergency(pitch_final, PITCH_MIN, PITCH_MAX, "PITCH_OUT_OF_RANGE");
  checkValueAndSetEmergency(roll_final, ROLL_MIN, ROLL_MAX, "ROLL_OUT_OF_RANGE");
  checkValueAndSetEmergency(yaw_final, YAW_MIN, YAW_MAX, "YAW_OUT_OF_RANGE");

  if(chainsawRunning && combined > COMBINED_MAX) {
    Serial.println("COMBINED_MAX");
    updateState(EMERGENCY);
  }


  switch(currentState)
  {
    case NORMAL:   //normal use
      chainsawRunning=true;
      digitalWrite(WORK_LED, HIGH);
      digitalWrite(BRAKE_LED, LOW);
      digitalWrite(SETTING_LED, LOW);
      setAngles();
      printAngles();
      break;

    case RESET: //reset Button - reset YAW & goes to normal use
    chainsawRunning=false;
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, HIGH);
      digitalWrite(SETTING_LED, HIGH);
      //resetAllValues();
      yawReset=0;
      setAngles();
      yawReset=yaw_final;
      setAngles();
      setMaxValues(PRESET_PITCH_MAX, PRESET_ROLL_MAX,PRESET_YAW_MAX);
      setMinValues(PRESET_PITCH_MIN, PRESET_ROLL_MIN,PRESET_YAW_MIN);
      COMBINED_MAX=PRESET_COMBINED_MAX;
      updateState(NORMAL);
      break;

    case SET: //Set-Button - defining ZONE
      chainsawRunning=false;
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, LOW);
      digitalWrite(SETTING_LED, HIGH);

      if(PITCH_MAX==PRESET_PITCH_MAX && ROLL_MAX==PRESET_ROLL_MAX && YAW_MAX==PRESET_YAW_MAX && PITCH_MIN==PRESET_PITCH_MIN && ROLL_MIN==PRESET_ROLL_MIN && YAW_MIN==PRESET_YAW_MIN)
      {
        setMaxValues(5, 5,5);
        setMinValues(-5, -5, -5);
      }

      setAngles();

      checkValues();

      printAngles();

      updateState(NORMAL);
      break;

    case EMERGENCY: //Emergency-STOP
      chainsawRunning=false;
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, HIGH);
      digitalWrite(SETTING_LED, LOW);
      updateState(STOP);
      break;

  }
}

void setLEDs(int state) {
  digitalWrite(WORK_LED, state);
  digitalWrite(BRAKE_LED, state);
  digitalWrite(SETTING_LED, state);
}

void blink() {
  for(int i = 0; i < 2; i++) {
    setLEDs(HIGH);
    delay(200);
    setLEDs(LOW);
    delay(200);
  }
}

void setAngles()
{
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
{ 

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetGyro(&gg, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw_final=(ypr[0] * RAD_TO_DEG) -yawReset;
        pitch_final=ypr[1] * RAD_TO_DEG;
        roll_final=ypr[2] * RAD_TO_DEG;

}
}

void printWithTab(const String& text, float value) {
  Serial.print(text);
  Serial.print(value);
  Serial.print("\t");
}

void printAngles() {
  printWithTab("Pitch:", pitch_final);
  printWithTab("Roll:", roll_final);
  printWithTab("YAW:", yaw_final);
  printWithTab("COMBINED:", combined);

  // Uncomment the following lines if you want to print MAX and MIN values
  /*
  printWithTab("Pitch_Max:", PITCH_MAX);
  printWithTab("Roll_MAX:", ROLL_MAX);
  printWithTab("YAW_Max:", YAW_MAX);
  printWithTab("Combined_Max:", COMBINED_MAX);
  printWithTab("Pitch_Min:", PITCH_MIN);
  printWithTab("Roll_Min:", ROLL_MIN);
  printWithTab("YAW_Min:", YAW_MIN);
  */
}

void setMaxValues(double pitch_max, double roll_max, double yaw_max)
{
  PITCH_MAX=pitch_max;
  ROLL_MAX=roll_max;
  YAW_MAX=yaw_max;

}

void setMinValues(double pitch_min, double roll_min, double yaw_min)
{
  PITCH_MIN=pitch_min;
  ROLL_MIN=roll_min;
  YAW_MIN=yaw_min;

}

void resetAllValues()
{
   mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-156);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(-14);
    mpu.setXAccelOffset(-3699);
    mpu.setYAccelOffset(-2519);
    mpu.setZAccelOffset(1391);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    blink();
}