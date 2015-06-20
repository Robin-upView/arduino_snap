#include <Arduino.h>

#include "Wire.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include "Servo.h"

#include "dcm.h"

void imu_Valget ();
void calcInput();
void calib_gyro();
void fast_loop();


#ifdef	__cplusplus
extern "C" {
#endif

#include "gru_quadcl.h"

#ifdef	__cplusplus
}
#endif

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int xoffset = 12;
int yoffset = -122;
int zoffset = -13;

int16_t mx, my, mz;     //To store magnetometer readings

volatile unsigned long startPeriod; // set in the interrupt
volatile int rc[7];

#define CH1  3  // Pin numbers //av gauche
#define CH2  5  //ar droit
#define CH3  6  //ar gauche
#define CH4  7  //av droit

// I2C address 0x69 could be 0x68 depending on setup??.
int MPU9150_I2C_ADDRESS = 0x68;

////COMPASS
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07

int led = 13;

long timer=0; //general purpose timer 
long timer_old;

Servo Servo_1;
Servo Servo_2;
Servo Servo_3;
Servo Servo_4;

int main(void)
{
	init();
	setup();
	for (;;)
		loop();

	return 0; // must NEVER be reached
}


void setup() {   
    
  pinMode(CH1,OUTPUT);
  pinMode(CH2,OUTPUT);
  pinMode(CH3,OUTPUT);
  pinMode(CH4,OUTPUT);
  
  Servo_1.attach (CH1, 1000,1900);  //av gauche
  Servo_2.attach (CH2, 1000,1900);  //ar droit
  Servo_3.attach (CH3, 1000,1900);  //ar gauche
  Servo_4.attach (CH4, 1000,1900);  //av droit
  
  Servo_1.writeMicroseconds(1000);//arrière droit
  Servo_2.writeMicroseconds(1000);//avant gauche
  Servo_3.writeMicroseconds(1000);//arrière gauche
  Servo_4.writeMicroseconds(1000);


  attachInterrupt(0,calcInput,FALLING);
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Wire.begin(0);
  
  pinMode(led, OUTPUT);  
  
  accelgyro.setSleepEnabled(false);
  
  accelgyro.setFullScaleGyroRange(3); //Gyro scale 2000deg/s
  delay(1);
  accelgyro.setFullScaleAccelRange(1);//Accel scale 4g
  delay(1);
  accelgyro.setClockSource(3);// Select GyroZ clock
  delay(1);
  accelgyro.setDLPFMode(4);// set bandwidth of both gyro and accelerometer to ~20 Hz
  delay(1);
  
  //IMU calibration
  calib_gyro(); //Bias computed once and values stored in program
  
  // Magnetometer configuration

  accelgyro.setI2CMasterModeEnabled(0);
  accelgyro.setI2CBypassEnabled(1);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x02); 
  Wire.write(0x00);  // Set continuous mode
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x00);
  Wire.write(B00011000);  // 75Hz
  Wire.endTransmission();
  delay(5);

  accelgyro.setI2CBypassEnabled(0);

  // X axis word
  accelgyro.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
  accelgyro.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
  accelgyro.setSlaveEnabled(0, true);
  accelgyro.setSlaveWordByteSwap(0, false);
  accelgyro.setSlaveWriteMode(0, false);
  accelgyro.setSlaveWordGroupOffset(0, false);
  accelgyro.setSlaveDataLength(0, 2);

  // Y axis word
  accelgyro.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
  accelgyro.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
  accelgyro.setSlaveEnabled(1, true);
  accelgyro.setSlaveWordByteSwap(1, false);
  accelgyro.setSlaveWriteMode(1, false);
  accelgyro.setSlaveWordGroupOffset(1, false);
  accelgyro.setSlaveDataLength(1, 2);

  // Z axis word
  accelgyro.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
  accelgyro.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
  accelgyro.setSlaveEnabled(2, true);
  accelgyro.setSlaveWordByteSwap(2, false);
  accelgyro.setSlaveWriteMode(2, false);
  accelgyro.setSlaveWordGroupOffset(2, false);
  accelgyro.setSlaveDataLength(2, 2);

  accelgyro.setI2CMasterModeEnabled(1);
  
  timer = micros();
 
  delay(20);
  }

void loop()
{
// Execute the fast loop
  if((micros()-timer)>=10000)   // 10ms => 100 Hz loop rate 
  { 
    timer_old = timer;
    timer=micros();
    G_Dt = (timer-timer_old)/1000000.0;      // Real time of loop run 

    fast_loop();
  }
}

void fast_loop() {
             
  //read sensors
  imu_Valget (); 
    

  //IMU Computation
  Matrix_update(); 
  Renormalization();
  Drift_cancellation();
  Euler_angles();
  
  
  //External parameters
  gru_quadcl_U.rx[0]=rc[2]*10;
  gru_quadcl_U.rx[2]=rc[0]*10;
  gru_quadcl_U.rx[3]=rc[1]*10;
  gru_quadcl_U.rx[4]=rc[3]*10;
  

  gru_quadcl_U.rates[0] = Omega[1];
  gru_quadcl_U.rates[1] = Omega[0];
  gru_quadcl_U.rates[2] = -Omega[2];
  
  gru_quadcl_U.ahrs[0] = roll;//roll
  gru_quadcl_U.ahrs[1] = pitch;//pitch
          
  gru_quadcl_U.extparams[0] = 0.058;//p_p //0.06
  gru_quadcl_U.extparams[10] = 0.075;//p_i
  gru_quadcl_U.extparams[11] = 0.004;//p_d //0.001
  
  gru_quadcl_U.extparams[15] = 0.090;//q_p //P.12
  gru_quadcl_U.extparams[16] = 0.100;//q_i
  gru_quadcl_U.extparams[17] = 0.006;//q_d //0.002
  
  gru_quadcl_U.extparams[1] = 1.2;//r_p
  
  gru_quadcl_U.extparams[14] = 1.0;//attitude_mode 
  
  gru_quadcl_U.extparams[6] = 40.0;//phi_scale theta_scale
  gru_quadcl_U.extparams[2] = 180.0;//p_scale q_scqle
  gru_quadcl_U.extparams[3] = 80.0;//p_scale q_scqle
  
  gru_quadcl_U.extparams[4] = 5.9000;//phi_p theta_p //4.5
  gru_quadcl_U.extparams[5] = 0.0000;//phi_i theta_i //1.0
  
  //Control law
  gru_quadcl_step();
  
 
  //Motors values
  Servo_2.writeMicroseconds(constrain(gru_quadcl_Y.servos[2]/10,1000,1900));//arrière droit
  Servo_1.writeMicroseconds(constrain(gru_quadcl_Y.servos[0]/10,1000,1900));//avant gauche
  Servo_3.writeMicroseconds(constrain(gru_quadcl_Y.servos[3]/10,1000,1900));//arrière gauche
  Servo_4.writeMicroseconds(constrain(gru_quadcl_Y.servos[1]/10,1000,1900));//avant droit
  
  
  //Logger
  Serial.print(MAG_Heading);
  Serial.print(" ");
  Serial.print(yaw);
  Serial.println(" ");
  Serial1.print(G_Dt,3);
  Serial1.println(" ");
   
 /*
  Serial.print(gru_quadcl_Y.addlog[19],3);
  Serial.print(" ");
  Serial.print(gru_quadcl_Y.addlog[20],3);
  Serial.print(" ");
  Serial.print(gru_quadcl_Y.addlog[21],3);
  Serial.print(" ");
  Serial.print(gru_quadcl_Y.addlog[22],3);
  Serial.print(" ");
  Serial.print(gru_quadcl_Y.addlog[23],3);
  Serial.print(" ");
  Serial.print(gru_quadcl_Y.addlog[24],3);
  Serial.print(" ");
  Serial.print(gru_quadcl_Y.addlog[25],3);
  Serial.println(" ");
*/
  
}


void calcInput()
{
  //static variables are not reset when we exit the function
  static unsigned int pulseIn;
  static int channel;
  
      //length of current pulse
      pulseIn = (int)(micros() - startPeriod);
      
      //remember the time for next loop
      startPeriod = micros();

      //channel detector
      if(pulseIn >2000){
        channel = 0;
      }
      //store value
      else
      {
        rc[channel]=pulseIn;
        channel++; //increment channel for next time
      }
}

void imu_Valget ()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //we store gyro and accel values in an array
  AN[0] = gx;   
  AN[1] = gy;
  AN[2] = gz;
  AN[3] = ax;
  AN[4] = ay;
  AN[5] = az;
  
    //Read magnetometer measures
    mx=accelgyro.getExternalSensorWord(0);
    my=accelgyro.getExternalSensorWord(2);
    mz=accelgyro.getExternalSensorWord(4);

  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(pitch);  // Optimizacion, se puede sacar esto de la matriz DCM?
  sin_roll = sin(pitch);
  cos_pitch = cos(-roll);
  sin_pitch = sin(-roll);
  // Tilt compensated Magnetic filed X:
  MAG_X = (mx-xoffset)*cos_pitch+(my-yoffset)*sin_roll*sin_pitch+(mz-zoffset)*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = (my-yoffset)*cos_roll-(mz-zoffset)*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(MAG_Y,MAG_X);
    //if(MAG_Heading < 0) MAG_Heading += 2 * M_PI;

}





void calib_gyro()
{

   for(int c=0; c<10; c++)
   { 
     digitalWrite(led, HIGH);
      delay(200);
      imu_Valget ();
      digitalWrite(led, LOW); 
      delay(200);
   }
  
   imu_Valget ();
   delay(20);
   imu_Valget ();
   for(int y=0; y<=2; y++)   // Read first initial ADC values for offset.
      AN_OFFSET[y]=AN[y];

   for(int i=0;i<100;i++)    // We take some readings...
   {
      imu_Valget ();
      for(int y=0; y<=2; y++)   // Read initial ADC values for offset (averaging).
         AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
      delay(20);

        }
  
   //AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
 
}


