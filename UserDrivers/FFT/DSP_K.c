/*
 * FFT.c
 *
 *  Created on: Jan 13, 2021
 *      Author: Development
 */
//
// NAME:          FFT.
// PURPOSE:       Быстрое преобразование Фурье: Комплексный сигнал в комплексный спектр и обратно.
//                В случае действительного сигнала в мнимую часть (Idat) записываются нули.
//                Количество отсчетов - кратно 2**К - т.е. 2, 4, 8, 16, ... (см. комментарии ниже).
//
//
// PARAMETERS:
//
//    float *Rdat    [in, out] - Real part of Input and Output Data (Signal or Spectrum)
//    float *Idat    [in, out] - Imaginary part of Input and Output Data (Signal or Spectrum)
//    int    N       [in]      - Input and Output Data length (Number of samples in arrays)
//    int    LogN    [in]      - Logarithm2(N)
//    int    Ft_Flag [in]      - Ft_Flag = FT_ERR_DIRECT  (i.e. -1) - Direct  FFT  (Signal to Spectrum)
//		                 Ft_Flag = FT_ERR_INVERSE (i.e.  1) - Inverse FFT  (Spectrum to Signal)
//
// RETURN VALUE: false on parameter error
//_________________________________________________________________________________________
//
// NOTE: In this algorithm N and LogN can be only:
//       N    = 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384;
//       LogN = 2, 3,  4,  5,  6,   7,   8,   9,   10,   11,   12,   13,    14;
//_________________________________________________________________________________________
//_________________________________________________________________________________________

// #include <math.h>
#include "DSP_K.h"
#include <stddef.h>
// #include "cmsis_gcc.h"

#define  NUMBER_IS_2_POW_K(x)   ((!((x)&((x)-1)))&&((x)>1))  // x is pow(2, k), k=1,2, ...
#define  FT_DIRECT         (-1)    // Direct transform.
#define  FT_INVERSE        (1)    // Inverse transform.
#define true 1
#define false 0


int FFTx(float *Rdat, float *Idat, int N, int LogN, int Ft_Flag)
{
  // parameters error check:
  if((Rdat == NULL) || (Idat == NULL))                  return false;
  if((N > 16384) || (N < 1))                            return false;
  if(!NUMBER_IS_2_POW_K(N))                             return false;
  if((LogN < 2) || (LogN > 14))                         return false;
  if((Ft_Flag != FT_DIRECT) && (Ft_Flag != FT_INVERSE)) return false;

  register int  i, j, n, k, io, ie, in, nn;
  float         ru, iu, rtp, itp, rtq, itq, rw, iw, sr;

  static const float Rcoef[14] =
  {  -1.0000000000000000F,  0.0000000000000000F,  0.7071067811865475F,
      0.9238795325112867F,  0.9807852804032304F,  0.9951847266721969F,
      0.9987954562051724F,  0.9996988186962042F,  0.9999247018391445F,
      0.9999811752826011F,  0.9999952938095761F,  0.9999988234517018F,
      0.9999997058628822F,  0.9999999264657178F
  };
  static const float Icoef[14] =
  {   0.0000000000000000F, -1.0000000000000000F, -0.7071067811865474F,
     -0.3826834323650897F, -0.1950903220161282F, -0.0980171403295606F,
     -0.0490676743274180F, -0.0245412285229122F, -0.0122715382857199F,
     -0.0061358846491544F, -0.0030679567629659F, -0.0015339801862847F,
     -0.0007669903187427F, -0.0003834951875714F
  };

  nn = N >> 1;
  ie = N;
  for(n=1; n<=LogN; n++)
  {
    rw = Rcoef[LogN - n];
    iw = Icoef[LogN - n];
    if(Ft_Flag == FT_INVERSE) iw = -iw;
    in = ie >> 1;
    ru = 1.0F;
    iu = 0.0F;
    for(j=0; j<in; j++)
    {
      for(i=j; i<N; i+=ie)
      {
        io       = i + in;
        rtp      = Rdat[i]  + Rdat[io];
        itp      = Idat[i]  + Idat[io];
        rtq      = Rdat[i]  - Rdat[io];
        itq      = Idat[i]  - Idat[io];
        Rdat[io] = rtq * ru - itq * iu;
        Idat[io] = itq * ru + rtq * iu;
        Rdat[i]  = rtp;
        Idat[i]  = itp;
      }

      sr = ru;
      ru = ru * rw - iu * iw;
      iu = iu * rw + sr * iw;
    }

    ie >>= 1;
  }

  for(j=i=1; i<N; i++)
  {
    if(i < j)
    {
      io       = i - 1;
      in       = j - 1;
      rtp      = Rdat[in];
      itp      = Idat[in];
      Rdat[in] = Rdat[io];
      Idat[in] = Idat[io];
      Rdat[io] = rtp;
      Idat[io] = itp;
    }

    k = nn;

    while(k < j)
    {
      j   = j - k;
      k >>= 1;
    }

    j = j + k;
  }

  if(Ft_Flag == FT_DIRECT) return true;

  rw = 1.0F / N;

  for(i=0; i<N/2; i++)
  {
    Rdat[i] *= rw;
    Idat[i] *= rw;

  }


  return true;
}


// -----------------  Atan2 -----------------------------//
#define M_PI ((float)3.141592653589793)
#define M_PI12 (M_PI/12.F)
#define M_PI6 (M_PI/6.F)
#define M_PI2 (M_PI/2.F)
#define M_PI3 (M_PI/3.F)
/* square root of 3 */
#define SQRT3 ((float)1.732050807569)
/*
DSP_K_Atan2
*/
float DSP_K_Atan2(float cos, float sin)
{

  float  x,x2,a;
	int sta=0,sp=0;
	x = sin/cos;
  /* check up the sign change */
  if(x<0.F) {x=-x;sta|=1;}
  /* check up the invertation */
  if(x>1.F) {x=1.F/x;sta|=2;}
  /* process shrinking the domain until x<PI/12 */
  while(x>M_PI12) {
    sp++; a=x+SQRT3;
		a=1.F/a;
		x*=SQRT3;
		x-=1.F; x*=a;}
  /* calculation core */
  x2=x*x;
	a=x2+1.4087812F;
	a=0.55913709F/a;
	a+=0.60310579F;
  a-=0.05160454F*x2;
	a*=x;
  /* process until sp=0 */
  while(sp>0) {a+=M_PI6;sp--;}
  /* invertation took place */
  if(sta&2) a=M_PI2-a;
  /* sign change took place */
  if(sta&1) a=-a;
	
		if(cos == 0.0f) {
		if(sin > 0.0f){a =  M_PI2;}
		if(sin < 0.0f){a = -M_PI2;}
    }else if(cos<0) {
		if(sin >= 0.0f){a += M_PI;}
		if(sin < 0.0f){a -= M_PI;}
    }
  return (a + M_PI)/M_PI*180.0f;
}





