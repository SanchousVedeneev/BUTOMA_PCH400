#ifndef BUF2_H
#define BUF2_H

#include <stdint.h>

#define DSP_T float

#define BUF_PUSH_FAIL   ((uint8_t)0)
#define BUF_PUSH_OK     ((uint8_t)1)
#define BUF_BLOCK       ((uint8_t)1)

typedef struct Buf_16_struct
{
    DSP_T *dataBuf;
    uint16_t size;
    uint16_t _counter;
    uint8_t _block;
} Buf_16_typedef;

uint8_t Buf_16_push(Buf_16_typedef* buf, DSP_T* data);

void Buf_16_release_buf(Buf_16_typedef* buf);

#endif