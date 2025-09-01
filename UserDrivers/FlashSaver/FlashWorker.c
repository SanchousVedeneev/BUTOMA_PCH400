
#include "FlashWorker.h"

static uint32_t GetBank(uint32_t Addr);
static uint32_t getPageNumber(uint32_t Addr);

#define EMPTY_VALUE ((uint64_t)0xFFFFFFFFFFFFFFFF)
#define DWORDS_COUNT (FLASH_WORKER_SIZE_AREA / 8)


typedef union {
    uint8_t bytes[8];
    uint64_t dword;
}packDword_typedef;

int16_t FlashWorker_getAvailableRecords(uint16_t sizeByte)
{
    packDword_typedef pack64;
    uint8_t* loadArea = NULL;
    int16_t cnt = -1;
    uint32_t dWordStructSize = sizeByte / 8 + 1;
    if (sizeByte % 8 != 0)
    {
        dWordStructSize += 1;
    }
    for (int w64 = 0; w64 < DWORDS_COUNT; w64++)
    {
        loadArea = (uint8_t *)((FLASH_WORKER_START_AREA) + w64 * 8);
        pack64.dword = *(uint64_t *)(loadArea);
        if (pack64.dword == EMPTY_VALUE)
        {
            pack64.dword = *(uint64_t *)(loadArea - 8);
            for (int i = 0; i < 8; i++)
            {
                if (pack64.bytes[i] != (uint8_t)(sizeByte))
                {
                    return -1;
                }
            }
            asm("NOP");
            cnt =  (FLASH_WORKER_SIZE_AREA - ((uint32_t)loadArea - FLASH_WORKER_START_AREA))/(dWordStructSize*8);
            asm("NOP");
            return cnt;
        }
    }
    return -1;
}

void FlashWorker_save(void *s, uint16_t sizeByte)
{
    static uint8_t flag = 0;
    uint64_t w64Value = 0;
    uint64_t writeZone = 0;
    packDword_typedef packDword;
    uint32_t dWordStructSize = sizeByte / 8;
    if (sizeByte % 8 != 0)
    {
        dWordStructSize += 1;
    }

    for (int w64 = 0; w64 < DWORDS_COUNT; w64++)
    {
        w64Value = *(uint64_t *)(FLASH_WORKER_START_AREA + w64 * 8);
        if (w64Value == EMPTY_VALUE)
        {
            if (DWORDS_COUNT - w64 >= dWordStructSize+1)
            {
                writeZone = FLASH_WORKER_START_AREA + w64 * 8;
                break;
            }
        }
    }
    if (writeZone != 0)
    {
        /* Unlock the Flash to enable the flash control register access *************/
        HAL_FLASH_Unlock();
        /* Clear OPTVERR bit set on virgin samples */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
        for (int w64 = 0; ; )
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, writeZone + 8 * w64, *((uint64_t *)(s) + w64)) != HAL_OK)
            {
                Error_Handler(); // err
            }
            w64++;
            if (w64 == dWordStructSize)
            {
                for (int i = 0; i < 8; i++)
                {
                    packDword.bytes[i] = (uint8_t)(sizeByte);
                }
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, writeZone + 8 * w64, packDword.dword) != HAL_OK)
                {
                    Error_Handler(); // err
                }
                break;
            }
        }

        HAL_FLASH_Lock();
        flag = 0;
        return;
    }

    if(flag){
        Error_Handler();
    }

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    /* Get the bank */
    uint32_t PAGEError;

    /* Fill EraseInit structure*/
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages = 1;
    EraseInitStruct.Banks = GetBank(FLASH_WORKER_START_AREA);
    EraseInitStruct.Page = getPageNumber(FLASH_WORKER_START_AREA);

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        /*
          Error occurred while sector erase.
          User can add here some code to deal with this error.
          SECTORError will contain the faulty sector and then to know the code error on this sector,
          user can call function 'HAL_FLASH_GetError()'
         */
        Error_Handler(); // err
    }
    HAL_FLASH_Lock();
    flag = 1;
    FlashWorker_save(s,sizeByte);
}


uint8_t FlashWorker_load(void *s, uint16_t sizeByte)
{
    packDword_typedef pack64;
    uint8_t* loadArea = NULL;

    for (int w64 = 0; w64 < DWORDS_COUNT; w64++)
    {
        loadArea = (uint8_t *)((FLASH_WORKER_START_AREA) + w64 * 8);
        pack64.dword = *(uint64_t *)(loadArea);
        if (pack64.dword == EMPTY_VALUE)
        {
            pack64.dword = *(uint64_t *)(loadArea-8);
            for (int i = 0; i < 8; i++)
            {
                if(pack64.bytes[i] != (uint8_t)(sizeByte)){
                    return 0;
                }
            }

            uint32_t dWordStructSize = sizeByte / 8;
            if (sizeByte % 8 != 0)
            {
                dWordStructSize += 1;
            }
            loadArea-=(dWordStructSize+1)*8;

            for (int i = 0; i < sizeByte; i++)
            {
                *((uint8_t *)(s) + i) = *(loadArea+i);
            }
            return 1;
        }
    }
    return 0;
}

static uint32_t GetBank(uint32_t Addr)
{
    uint32_t bank = 0;
    if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
    {
        /* No Bank swap */
        if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
        {
            bank = FLASH_BANK_1;
        }
        else
        {
            bank = FLASH_BANK_2;
        }
    }
    else
    {
        /* Bank swap */
        if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
        {
            bank = FLASH_BANK_2;
        }
        else
        {
            bank = FLASH_BANK_1;
        }
    }
    return bank;
}

static uint32_t getPageNumber(uint32_t Addr)
{
    return (FLASH_WORKER_START_AREA - FLASH_BASE) / FLASH_PAGE_SIZE / 2;
}







