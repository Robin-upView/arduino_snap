/* 
 * File:   dcm.h
 * Author: Robin
 *
 * Created on 3 mai 2015, 19:55
 */


#ifndef DCM_H
#define	DCM_H

float MAG_Heading;
float mag_heading_x;
float mag_heading_y;

#define GRAVITY 8192
//#define Kp_ROLLPITCH 0.015
//#define Ki_ROLLPITCH 0.000010

#define Kp_ROLLPITCH 1.515/GRAVITY
#define Ki_ROLLPITCH 0.00101/GRAVITY

#define Kp_YAW 1.0 // Yaw Porportional Gain  
#define Ki_YAW 0.00005 // Yaw Integrator Gain
float errorYaw[3]= {0,0,0};

#define Gyro_Gain_X 0.0609
#define Gyro_Gain_Y 0.0609
#define Gyro_Gain_Z 0.0609
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

 ///////////////// DCM Variables /////////////////
//will be used for the computation
float DCM_Matrix[3][3]= {
  {1,0,0}
  ,{0,1,0}
  ,{0,0,1}
};


//will be used for the computation
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}};


//will be used for the computation
float Temporary_Matrix[3][3]={
  {0,0,0}
  ,{0,0,0}
  ,{0,0,0}
};

float Accel_Vector[3]= {0,0,0}; //accel values in m/s-2

float Gyro_Vector[3]= {0,0,0};//gyro values in rad/s

float Omega_Vector[3]= {
  0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {
  0,0,0};//Omega Proportional correction
float Omega_I[3]= {
  0,0,0};//Omega Integrator
float Omega[3]= {
  0,0,0};//Omega

float G_Dt=0.02;    // Integration time for the gyros (DCM algorithm)

float errorRollPitch[3]= {
  0,0,0};
float roll=0;
float pitch=0;
float yaw=0;

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

uint8_t sensors[6] = {0,1,2,3,4,5};  
int SENSOR_SIGN[] = {1,-1,-1,-1,1,1,-1,1,-1};


///////////////// End DCM Variables /////////////////



float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
void Euler_angles(void);
void Matrix_update(void);
void Drift_cancellation(void);
void Renormalization(void);
float read_adc(int select);
#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

 

void Renormalization(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}


void Drift_cancellation(void)
{
  //Compensation the Roll, Pitch and Yaw drift. 
  float errorCourse;
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  
  // Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  Accel_weight = constrain(1 - 4*abs(1 - Accel_magnitude),0,1);

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
  
    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse= (DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  
  
}


void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(read_adc(0)); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(read_adc(1)); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(read_adc(2)); //gyro Z yaw
  
  // Low pass filter on accelerometer data (to filter vibrations)
  //Accel_Vector[0]=Accel_Vector[0]*0.5 + read_adc(3)*0.5; // acc x
  //Accel_Vector[1]=Accel_Vector[1]*0.5 + read_adc(4)*0.5; // acc y
  //Accel_Vector[2]=Accel_Vector[2]*0.5 + read_adc(5)*0.5; // acc z

  Accel_Vector[0]=read_adc(3); // acc x
  Accel_Vector[1]=read_adc(4); // acc y
  Accel_Vector[2]=read_adc(5); // acc z
  
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);//adding integrator
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]);//adding proportional
    

  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;



  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++)  //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  // Euler angles from DCM matrix
    roll = asin(-DCM_Matrix[2][0]);
    pitch = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

//Computes the dot product of two 3x1 matrixs -> |A|.|B|.cos(alpha)
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;

  for(int c=0; c<3; c++)
  {
    op+=vector1[c]*vector2[c];
  }

  return op; 
}

//Computes the cross product of two 3x1 matrixs -> |A|.|B|.sin(alpha) perpenducular vector
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply a 3x1 matrix by a scalar
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
    vectorOut[c]=vectorIn[c]*scale2; 
  }
}

//Add two 3x1 matrixs
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
    vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}


//Multiply two 3x3 matrixs. 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3]; 
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
        op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];

      
    }
  }
}

float read_adc(int select)
{
  if (SENSOR_SIGN[select]<0) {
    return (AN_OFFSET[select]-AN[select]);
  }
  else {
    return (AN[select]-AN_OFFSET[select]);
  }
}



#endif	/* DCM_H */

