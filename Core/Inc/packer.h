#pragma once

#include <stddef.h>
#include <stdint.h>

/// поддерживаемые типы данных
///
/// **** знаковые signed *****
/// int8_t
/// int16_t
/// int32_t
/// float
///
/// **** беззнаковые unsigned ****
/// uint8_t
/// uint16_t
/// uint32_t
/// ufloat
typedef float ufloat;
typedef enum 
{
  INT8, INT16, INT32, FLOAT,
  UINT8, UINT16, UINT32, UFLOAT,
} support_type_e;
/// **** атрибуты данных ****
///
/// добавляются к данным ввиде коментария //%
/// прммер:
///    //% kdiv=1000
///    //% kmul=128
///    //% bits=7
///    ufloat anydata; //0..1000 => 0..127
///
/// bits - число бит на передачу
/// delta,kmul,kdiv - масштабирование
/// поддерживаемые атрибуты данных значения по умолчанию:
/// float delta=0
/// float kmul=1
/// float kdiv=1
/// uint8_t bits=8
///
/// альтернативный подход зная from,to,bits получить delta, kamp=kmul/kdiv
/// float from =---
/// float to   =--- Обязательный атрибут >0 !!! по умолчанию (unsigned) from = 0 или (signed) from = -to
/// 
/// атрибут //% send - определяет основную структуру для передачи
/// //% send
/// typedef struct
/// {.....


/// ******* преобразование данных *********
///
/// масштабирование
/// float fdata_corr = ((float)data + delta)*kamp;
/// int32_t s32Data = Round(fdata_corr)
///
/// обрезка 
/// bits=N
/// **** знаковые signed *****
/// min = -(2^(N-1))
/// max =   2^(N-1)-1
/// **** беззнаковые unsigned ****
/// min = 0
/// max = 2^N-1
///
/// if (s32Data < smin) s32Data = smin
/// else if (s32Data > smax) s32Data = smax


// генератором метаданных h2send.exe datain.h metadataout.h NameMetaData 
//      создается хедер файл metadataout.h:
// определяется длина входного и выходного пакета данных, и число неиспользованных битов вконце: 
//      #define CNT_UNUSEDBITS x (нужно подогнать bits чтобы был 0)
//      #define LEN_ARRAY5BIT xx
//      #define LEN_ARRAY_GENERATED xx
// создается массив структур:  
//      const GeneratedItem_t NameMetaData[LEN_ARRAY_GENERATED] = {.....}; 
typedef struct 
{       
    support_type_e type;     // тип данных
    uint_fast16_t offset;         // смещение в структуре данных для передачи
    uint_fast8_t bits;            // атрибут bits число бит для передачи (точность)
    uint_fast8_t N5bit;           // сколько 5bit данных в массиве занимает величина
    uint_fast8_t OffsetInArray;   // начиная с какого элемента в 5bit массиве 
    uint_fast8_t OffsetInBit;     // начиная с какого бита первого элемента в 5bit массиве  0..4
    float delta;             // атрибут delta
    float kamp;              // атрибуты kmul/kdiv
} GeneratedItem_t;

#define N5bit(bits,OffsetInBit) \
       ({ uint8_t _nbf = (5-OffsetInBit) % 5; \
          uint8_t _nb = bits-_nbf; \
          uint8_t _nbl = _nb % 5; \
         ((_nbf?1:0) + (_nb/5) + (_nbl?1:0)); })

 void inner_Encode(const void* inData, uint8_t* outData, const GeneratedItem_t* metadata, uint_fast8_t lenmetadata);
 void inner_Decode(const uint8_t* inData, void* outData, const GeneratedItem_t* metadata, uint_fast8_t lenmetadata);
 uint8_t inner_Quality(size_t idx, uint8_t* inQuality, const GeneratedItem_t* metadata, uint_fast8_t lenmetadata);
