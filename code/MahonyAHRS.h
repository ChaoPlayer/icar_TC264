#ifndef CODE_MAHONYAHRS_H_
#define CODE_MAHONYAHRS_H_

#include "Ifx_Cf32.h"
#include "Ifx_LutAtan2F32.h"

void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Mahony_Init(float sampleFrequency);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_computeAngles(void);
void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz);
float getRoll(void);
float getPitch(void);
float getYaw(void);
float getRollRadians(void);
float getPitchRadians(void);
float getYawRadians(void);

#endif
