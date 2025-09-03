
#include "Buf2.h"

#include <stddef.h>
#include "cmsis_gcc.h"

__INLINE uint8_t Buf_16_push(Buf_16_typedef *buf, DSP_T *data)
{
    if(buf->_block){
        return BUF_PUSH_FAIL;
    }

    *(buf->dataBuf + buf->_counter++) = *data;

    if(buf->_counter==buf->size){
       buf->_counter=0;
       buf->_block= BUF_BLOCK;
    }
    return BUF_PUSH_OK;
}

void Buf_16_release_buf(Buf_16_typedef* buf)
{
    buf->_block = 0;
}