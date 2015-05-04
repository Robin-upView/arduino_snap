/* 
 * File:   dcm.h
 * Author: Robin
 *
 * Created on 3 mai 2015, 19:55
 */


#ifndef DCM_H
#define	DCM_H






float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
void Euler_angles(void);
void Matrix_update(void);
void Drift_cancellation(void);
void Renormalization(void);

#endif	/* DCM_H */

