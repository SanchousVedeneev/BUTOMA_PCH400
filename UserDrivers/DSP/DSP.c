#include "DSP.h"

float dsp_regulatorProcess(dsp_regulator_typedef *r)
{
    float outTmp = 0.0f;
    float intTmp = 0.0f;
    if (r == NULL)
    {
        return 0.0f;
    }
    // Ошибка
    r->d = r->In - r->Fb;
    // Интегральная часть Int = delta x K_i + Int
    intTmp = r->d * r->period * r->k_Int + r->Int;

    /* Ограничение интеграла */
    if (intTmp < r->IntMin)
        intTmp = r->IntMin;
    if (intTmp > r->IntMax)
        intTmp = r->IntMax;
    r->Int = intTmp;

    // Пропорциональная часть
    r->P = r->k_P * r->d;
    /* результат */
    outTmp = r->Int + r->P;
    // Ограничение
    if (outTmp < r->OutMin)
        outTmp = r->OutMin;
    else if (outTmp > r->OutMax)
        outTmp = r->OutMax;
    r->Out = outTmp;
    return outTmp;
}

void dsp_regulatorReset(dsp_regulator_typedef *r)
{
  r->In = 0.0f;
  r->Fb = 0.0f;
  r->d = 0.0f;
  r->P = 0.0f;
  r->Int = r->IntMin;
  r->Out = r->OutMin;
  return;
}

void dsp_intensSetterSetup(dsp_intensSetter_typedef* intens, float setting, float period)
{

    intens->settings = setting;
    intens->period = period;
    intens->step = setting*period;
    return;
}

void dsp_intensSetterUpProcess(dsp_intensSetter_typedef* intens)
{
  if(intens->in > intens->out) 
  {
    intens->out += intens->step;
  }
  else if(intens->in < intens->out)
  {
    intens->out = intens->in;
  }
  return;
}

void dsp_intensSetterReset(dsp_intensSetter_typedef* intens)
{
  intens->in = 0.0f;
  intens->out = 0.0f;
}

float dsp_lineApprox(const dsp_point_typedef* points, uint8_t count, float input)
{
    //in range
    if(input < points[0].x){
      return points[0].y;
    }
    if(input > points[count-1].x){
      return points[count-1].y;
    }

    if(count < 2){
      return 0.0f;
    }

    //find line
    uint8_t idx = 0;
    for (int i = 1; i < count; i++)
    {
      if (input < points[i].x)
      {
        idx = i;
        break;
      }
    }
    
    if(idx == 0) return 0.0f;

    return (input - points[idx-1].x)*((points[idx].y - points[idx-1].y)/(points[idx].x-points[idx-1].x)) + points[idx-1].y;
}