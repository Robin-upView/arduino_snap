#include <Arduino.h>

#include "Wire.h"

#include "MPU6050.h"
#include "I2Cdev.h"
#include "MS561101BA.h"
#include "Servo.h"


//#include "dcm.h"

void imu_Valget ();
void calcInput();
void calib_gyro();
void fast_loop();
void updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt);
float invSqrt(float x);


#ifdef	__cplusplus
extern "C" {
#endif

#include "gru_quadcl.h"

#ifdef	__cplusplus
}
#endif

MPU6050 accelgyro;
int16_t a_x, a_y, a_z;
int16_t g_x, g_y, g_z;
int16_t m_x, m_y, m_z;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float q0, q1, q2, q3;
float gyroOffset[3];
float twoKi;
float twoKp;
float integralFBx, integralFBy, integralFBz;


float pitch, roll, yaw;

float gyro_offset_x, gyro_offset_y, gyro_offset_z;
float G_Dt=0.02;

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
  accelgyro.setRate(1);
  delay(1);
  accelgyro.setDLPFMode(4);// set bandwidth of both gyro and accelerometer to ~20 Hz
  delay(1);
  
  //IMU calibration
  calib_gyro(); //Bias computed once and values stored in program
  
  
  gru_quadcl_initialize();
  
  gru_quadcl_U.extparams[0] = 0.150;//p_p //0.12
  gru_quadcl_U.extparams[10] = 0.000;//0.09;//p_i
  gru_quadcl_U.extparams[11] = 0.004;//p_d //0.0025 steps
  
  gru_quadcl_U.extparams[15] = 0.150;//q_p //0.08
  gru_quadcl_U.extparams[16] = 0.000;//0.09;//q_i
  gru_quadcl_U.extparams[17] = 0.004;//q_d //0.0025 steps
  
  gru_quadcl_U.extparams[1] = 1.2;//r_p
  gru_quadcl_U.extparams[13] = 0.2;//r_breakout
  gru_quadcl_U.extparams[12] = -1.0;//head_p
  
  gru_quadcl_U.extparams[14] = 1.0;//attitude_mode 
  
  gru_quadcl_U.extparams[6] = 45.0;//phi_scale theta_scale
  gru_quadcl_U.extparams[2] = 180.0;//p_scale q_scqle
  gru_quadcl_U.extparams[3] = 150.0;//r_scale
  
  gru_quadcl_U.extparams[4] = 6.500;//phi_p theta_p //7.0
  gru_quadcl_U.extparams[5] = 1.000;//0.500;//phi_i theta_i //1.0//0.5
  
  q0=1;
  q1=0;
  q2=0;
  q3=0;
  twoKi = 0.001;
  twoKp = 1.5;

  
  timer = micros();
 
  delay(20);
  }

void loop()
{
// Execute the fast loop
  if((micros()-timer)>=5000)   // 10ms => 100 Hz loop rate 
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
    
  updateIMU(ax,ay,az,gx,gy,gz,G_Dt);
   
  pitch = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  roll = -asin(2*(q0*q2-q3*q1));
  yaw = -atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));

  //IMU Computation
  //Matrix_update(); 
  //Renormalization();
  //Drift_cancellation();
  //Euler_angles();
  
  
  //External parameters
  gru_quadcl_U.rx[0]=rc[2]*10;
  gru_quadcl_U.rx[2]=rc[0]*10;
  gru_quadcl_U.rx[3]=rc[1]*10;
  gru_quadcl_U.rx[4]=rc[3]*10;
  gru_quadcl_U.rx[5]=rc[4]*10;

  gru_quadcl_U.rates[0] =-(gy-gyro_offset_y);
  gru_quadcl_U.rates[1] = gx-gyro_offset_x;
  gru_quadcl_U.rates[2] = gz-gyro_offset_z;
  
  gru_quadcl_U.ahrs[0] = roll;//roll
  gru_quadcl_U.ahrs[1] = pitch;//pitch
  gru_quadcl_U.ahrs[2] = yaw;//yaw
          
  
  //Control law
  gru_quadcl_step();
  
 
  //Motors values
  Servo_2.writeMicroseconds(constrain(gru_quadcl_Y.servos[2]/10,1000,1900));//arrière droit
  Servo_1.writeMicroseconds(constrain(gru_quadcl_Y.servos[0]/10,1000,1900));//avant gauche
  Servo_3.writeMicroseconds(constrain(gru_quadcl_Y.servos[3]/10,1000,1900));//arrière gauche
  Servo_4.writeMicroseconds(constrain(gru_quadcl_Y.servos[1]/10,1000,1900));//avant droit

  
  //Serial.print(roll);
  //Serial.print(" ");
  //Serial.print(pitch);
  //Serial.print(" ");
  //Serial.println(yaw);
  //Serial.print(" ");
  //Serial.print(gy-gyro_offset_y);
  //Serial.print(" ");
  //Serial.print(gx-gyro_offset_x);
  //Serial.print(" ");
  //Serial.print(gz-gyro_offset_z);
  //Serial.print(" ");
  //Serial.println(G_Dt*10000);
  //Serial.println(" ");
   
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[30],3);
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[31],3);
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[29],3);
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[19],3);
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[23],3);
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[24],3);
  //Serial.print(" ");
  //Serial.print(gru_quadcl_Y.addlog[25],3);
  //Serial.println(" ");

  
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
   //read int16_t
  accelgyro.getMotion6(&a_x, &a_y, &a_z, &g_x, &g_y, &g_z);
  
  //conversion in float
  ax=a_x;
  ay=a_y;
  az=a_z;
  gx=g_x;
  gy=g_y;
  gz=g_z;
  
  //conversion in rad/s
  gx = gx*2000.0/32768.0*0.0175;
  gy = gy*2000.0/32768.0*0.0175;
  gz = gz*2000.0/32768.0*0.0175; 
      
}





void calib_gyro()
{

   for(int c=0; c<10; c++)
   { 
     digitalWrite(led, HIGH);
      delay(200);
      imu_Valget();
      digitalWrite(led, LOW); 
      delay(200);
   }

	for(int i = 0; i<100; i++)
	{
		imu_Valget();
		gyro_offset_x += gx;
		gyro_offset_y += gy;
		gyro_offset_z += gz;
		delay(10);
	}
	gyro_offset_x/=100.0;
	gyro_offset_y/=100.0;
	gyro_offset_z/=100.0;
}

void updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx -= gyro_offset_x;
  gy -= gyro_offset_y;
  gz -= gyro_offset_z;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;	// apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;	// prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);		// pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
 
}


float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}