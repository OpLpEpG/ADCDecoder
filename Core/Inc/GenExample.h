#pragma once

#include "packer.h"

#define LEN_ARRAY_GENERATED 14
#define LEN_ARRAY5BIT 28
#define CNT_UNUSEDBITS 0

/// @brief  упаковка данных по 5 бит
/// @param inData указатель на структуру данных для передачи
/// @param outData указатель на массив выходных данных uint8_t TRANS_5BitData[1+4+LEN_ARRAY5BIT+1]; &TRANS_5BitData[5]
#define Encode(inData, outData) inner_Encode(inData, outData, GenData,LEN_ARRAY_GENERATED)

/// @brief распаковка данных по 5 бит
/// @param inData указатель на массив принятых данных uint8_t DECODE_5BitData[LEN_ARRAY5BIT]; &DECODE_5BitData[0]
/// @param outData указатель на структуру данных для передачи
#define Decode(inData, outData) inner_Decode(inData, outData, GenData,LEN_ARRAY_GENERATED)

/// @brief возвращает достоверность члена данных структуры
/// @param GenData_field имя из GenData_fields
/// @param inQuality указатель на массив качества принятых данных в процентах
/// @return качество данных в процентах
#define Quality(GenData_field, inQuality) inner_Quality(GenData_field,inQuality, GenData, LEN_ARRAY_GENERATED)

enum GenData_fields {
flag,v_bat,mah,GKup,count_kadr,ZU,OBOR,NAGR,TEMP,v_trans,tok_trans,MOMENTtr,DAVLtr,GKdn
};

const GeneratedItem_t GenData[LEN_ARRAY_GENERATED] = {
// TYPE |OFF|BTS|5BL|5BI|BI|      delta           |        amp
{UINT8  ,  0,  1,  1,  0,0,  0.00000000000000E+000, 1.00000000000000E+000}, // flag
{UFLOAT ,  1,  6,  2,  0,1, -1.10000000000000E+000, 1.00000000000000E+001}, // v_bat
{UFLOAT ,  5,  8,  2,  1,2,  0.00000000000000E+000, 1.00000000000000E+001}, // mah
{UFLOAT ,  9, 10,  2,  3,0,  0.00000000000000E+000, 1.00000000000000E+001}, // GKup
{UINT32 , 13, 20,  4,  5,0,  0.00000000000000E+000, 1.00000000000000E+000}, // count_kadr
{UFLOAT , 17, 11,  3,  9,0,  0.00000000000000E+000, 1.00000000000000E+001}, // ZU
{UFLOAT , 21,  9,  2, 11,1,  0.00000000000000E+000, 1.00000000000000E+000}, // OBOR
{FLOAT  , 25,  9,  2, 13,0,  0.00000000000000E+000, 1.00000000000000E+001}, // NAGR
{UFLOAT , 29, 11,  3, 14,4,  0.00000000000000E+000, 1.00000000000000E+001}, // TEMP
{UFLOAT , 33, 15,  3, 17,0,  0.00000000000000E+000, 1.00000000000000E+003}, // v_trans
{UFLOAT , 37, 12,  3, 20,0,  0.00000000000000E+000, 1.00000000000000E+000}, // tok_trans
{UFLOAT , 41,  9,  3, 22,2,  0.00000000000000E+000, 1.00000000000000E+001}, // MOMENTtr
{INT16  , 45,  9,  2, 24,1,  0.00000000000000E+000, 1.00000000000000E+000}, // DAVLtr
{UFLOAT , 47, 10,  2, 26,0,  0.00000000000000E+000, 1.00000000000000E+001}, // GKdn
};


