#ifndef __DSP__H__
#define __DSP__H__

#include "main.h"







typedef struct
{
    float k_P;
    float k_Int;
    float period;

    float In;
    float Fb;
    float d;
    float Out;

    float Int;
    float P;

    float OutMax;
    float OutMin;

    float IntMax;
    float IntMin;

} dsp_regulator_typedef;


typedef struct 
{       
    float in;
    float step;
    float out;
    float period;
    float settings;
}dsp_intensSetter_typedef;

void dsp_intensSetterSetup(dsp_intensSetter_typedef* intens, float setting, float period);
void dsp_intensSetterUpProcess(dsp_intensSetter_typedef* intens);
void dsp_intensSetterReset(dsp_intensSetter_typedef* intens);


typedef struct
{
	float x;
	float y;
}dsp_point_typedef;

float dsp_regulatorProcess(dsp_regulator_typedef *r);
void dsp_regulatorReset(dsp_regulator_typedef *r);

float dsp_lineApprox(const dsp_point_typedef* points, uint8_t count, float input);











#endif