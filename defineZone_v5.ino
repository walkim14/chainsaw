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

//Inputs and Outputs
const int WORK_LED = 2; // Pin für die WORK LED
const int BRAKE_LED = 3; // Pin für die BRAKE LED
const int SETTING_LED = 4; // Pin für die Settings LED
const int RESET_BUTTON = 5; // Pin für den Reset-Taster
const int SET_BUTTON = 6; // Pin für den Set-Taster

double ALPHA = 0.75; // Gewichtungsfaktor für den Komplementärfilter
double accAngleX, accAngleY; // Beschleunigungsmesser-Winkel
double gyroAngleX, gyroAngleY, gyroAngleZ; // Gyroskop-Winkel
double roll, pitch, rollZero, pitchZero, yaw, yawZero; // Roll- und Pitch-Winkel
double combined;
double yawReset=0;

const double  offsetPitch=0; //not calculated data
const double  offsetRoll=0;    //not calculated data 
const double  offsetYaw=0;     //not calculated data

const double PRESET_PITCH_MAX=80;
const double PRESET_PITCH_MIN=-80;
const double PRESET_ROLL_MAX=80;
const double PRESET_ROLL_MIN=-80;
const double PRESET_YAW_MAX=80;
const double PRESET_YAW_MIN=-80;
const double PRESET_COMBINED_MAX=85;


double pitch_final, roll_final, yaw_final;
double PITCH_MAX, ROLL_MAX, YAW_MAX;
double PITCH_MIN, ROLL_MIN, YAW_MIN;
double COMBINED_MAX;


double previousTime;
double currentTime; 
double elapsedTime;

bool chainsawRunning;
int currentState=0;

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

  currentState=4; //default
  blink();
  chainsawRunning=false;
}

void loop()
{

  if(digitalRead(RESET_BUTTON)==LOW)
  {
    currentState=1;
  }

  if(digitalRead(SET_BUTTON)==LOW)
  {
    currentState=2;
  }

  if(chainsawRunning && pitch_final>PITCH_MAX)
  {
    Serial.println("PITCH_MAX");
    currentState=3;
  }

  if(chainsawRunning && pitch_final<PITCH_MIN)
  {
    Serial.println("PITCH_MIN");
    currentState=3;
  }

  if(chainsawRunning && roll_final>ROLL_MAX)
  {
    Serial.println("ROLL_MAX");
    currentState=3;
  }

  if(chainsawRunning && roll_final<ROLL_MIN)
  {
    Serial.println("ROLL_MIN");
    currentState=3;
  }

  if(chainsawRunning && yaw_final>YAW_MAX)
  {
    Serial.println("YAW_MAX");
    currentState=3;
  }

  if(chainsawRunning && yaw_final<YAW_MIN)
  {
    Serial.println("YAW_MIN");
    currentState=3;
  }
  if (chainsawRunning && combined>COMBINED_MAX)
  {
    Serial.println("COMBINED_MAX");
    currentState=3;
  }


  switch(currentState)
  {
    case 0:   //normal use
      chainsawRunning=true;
      digitalWrite(WORK_LED, HIGH);
      digitalWrite(BRAKE_LED, LOW);
      digitalWrite(SETTING_LED, LOW);
      setAngles();
      printAngles();
      break;

    case 1: //reset Button - reset YAW & goes to normal use
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
      currentState=0;
      break;

    case 2: //Set-Button - defining ZONE
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

      if(pitch_final>PITCH_MAX)
        {
          PITCH_MAX=pitch_final;
          COMBINED_MAX=(abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final));
        }
      if(roll_final>ROLL_MAX)
        {
          ROLL_MAX=roll_final;
  	      COMBINED_MAX=(abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final));
        }

      if(yaw_final>YAW_MAX)
        {
          YAW_MAX=yaw_final;
          COMBINED_MAX=(abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final));
        }


      if(pitch_final<PITCH_MIN)
        {
          PITCH_MIN=pitch_final;
          COMBINED_MAX=(abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final));
        }

      if(roll_final<ROLL_MIN)
        {
          ROLL_MIN=roll_final;
          COMBINED_MAX=(abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final));
        }

      if(yaw_final<YAW_MIN)
        {
          YAW_MIN=yaw_final;
          COMBINED_MAX=(abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final));
        }
      printAngles();

      currentState=0;
      break;

    case 3: //Emergency-STOP
      chainsawRunning=false;
      digitalWrite(WORK_LED, LOW);
      digitalWrite(BRAKE_LED, HIGH);
      digitalWrite(SETTING_LED, LOW);
      currentState=4;
      break;

  }
}

void blink ()
{
  digitalWrite(WORK_LED, HIGH);
  digitalWrite(BRAKE_LED, HIGH);
  digitalWrite(SETTING_LED, HIGH);
  delay(200);
  digitalWrite(WORK_LED, LOW);
  digitalWrite(BRAKE_LED, LOW);
  digitalWrite(SETTING_LED, LOW);
  delay(200);
  digitalWrite(WORK_LED, HIGH);
  digitalWrite(BRAKE_LED, HIGH);
  digitalWrite(SETTING_LED, HIGH);
  delay(200);
  digitalWrite(WORK_LED, LOW);
  digitalWrite(BRAKE_LED, LOW);
  digitalWrite(SETTING_LED, LOW);
  delay(200);
}

void setAngles()
{
/*
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax-=-337;
    ay-=-295;
    az-=1375.5;
    gx-=19.5;
    gy-=-33.5;
    gz-=4.5;

  // Berechne Beschleunigungsmesser-Winkel (Roll und Pitch)
    accAngleX=atan2(ay,sqrt(pow(ax,2)+pow(az,2)))*180.0/M_PI;
    accAngleY=atan2(ax,sqrt(pow(ay,2)+pow(az,2)))*180.0/M_PI;
    
  //Time calc
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  // Berechne Gyroskop-Winkel
    double gyroRateX = (double)gx / 131.0; // 131 LSB/°/s für Gyroskop
    double gyroRateY = (double)gy / 131.0;
    double gyroRateZ = (double)gz / 131.0;
    gyroAngleX += gyroRateX *(double)0.01; // 0.01 Sekunden Probenahmezeit, bzw elapsed Time
    gyroAngleY += gyroRateY *(double)0.01;
    gyroAngleZ += gyroRateZ *(double)0.01;

  // Berechne Roll- und Pitch-Winkel durch Kombination von Beschleunigungsmesser- und Gyroskop-Winkeln
    roll = ALPHA * (roll + gyroRateX *(double)0.01) + (1 - ALPHA) * accAngleX;
    pitch = (ALPHA * (pitch + gyroRateY *(double)0.01) + (1 - ALPHA) * accAngleY);

    rollZero=-roll+offsetRoll;      //Roll starts at +90deg
    pitchZero=(-pitch+offsetPitch); //Pitch starts at +90deg

  //Berechnung von YAW
    yaw+=gyroRateZ*elapsedTime;
    yawZero=-yaw+offsetYaw;

  //setting new Values
    pitch_final=pitchZero;
    roll_final=rollZero;
    yaw_final=yawZero;
    
  //setting combined value
   // combined=((pow(tan(pitch_final-90),2)/pow(tan(PITCH_MAX-90),2))+pow(tan(yaw_final-90),2)/pow(tan(YAW_MAX-90),2));
  combined=abs(pitch_final)+abs(yaw_final)+1.5*abs(roll_final);*/

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

void printAngles()
{
    Serial.print("Pitch:");
    Serial.println(pitch_final); Serial.print("\t");
    Serial.print("Roll:");
    Serial.print(roll_final); Serial.print("\t");
    Serial.print("YAW:");
    Serial.println(yaw_final); Serial.print("\t");
    Serial.print("COMBINED:");
    Serial.println(combined); Serial.print("\t");

//MAX:
  /*  Serial.print("Pitch_Max:");
    Serial.println(PITCH_MAX); Serial.print("\t");
    Serial.print("Roll_MAX:");
    Serial.print(ROLL_MAX); Serial.print("\t");
    Serial.print("YAW_Max:");
    Serial.println(YAW_MAX); Serial.print("\t");
    Serial.print("Combined_Max:");
    Serial.println(COMBINED_MAX); Serial.print("\t");
//MIN:
    Serial.print("Pitch_Min:");
    Serial.println(PITCH_MIN); Serial.print("\t");
    Serial.print("Roll_Min:");
    Serial.print(ROLL_MIN); Serial.print("\t");
    Serial.print("YAW_Min:");
    Serial.println(YAW_MIN); Serial.print("\t");
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

    blink();
}