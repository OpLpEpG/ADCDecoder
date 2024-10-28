/*
 * adecoder.h
 *
 *  Created on: Oct 3, 2024
 *      Author: User
 */

#ifndef SRC_ADECODER_H_
#define SRC_ADECODER_H_

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <main.h>
#include <stm32g4xx_hal.h>

#define INLN __attribute__((__always_inline__)) inline

#define POROG_SP 70
#define REGARD_SP 50

#define FIND_SP (-1)
#define FIND_CODE (0)

template <uint_fast8_t SMP_PER_SYM, uint_fast8_t STEP_LEN, uint_fast8_t CODELEN>
class adecoder_t
{
public:

  static const uint_fast16_t splen = 32 * 4 * SMP_PER_SYM; // in bits
  static const uint_fast16_t codlen = 32 * SMP_PER_SYM;    // in bits
  static const uint_fast16_t bits = round(log2(splen * 3));
  static const uint_fast16_t buflen = 1 << bits;
  static const uint_fast16_t buflenMask = buflen - 1;
  static const uint_fast16_t dmalen = buflen / 16;
  static const uint_fast16_t buflenEx = dmalen * (1 + splen / dmalen);
  static const uint_fast16_t porogADC_max = 1 << 11; // 12 bit adc-sign
  static const uint_fast16_t delayTest = (SMP_PER_SYM/2/STEP_LEN)*STEP_LEN;
  static const int32_t porogsp = porogADC_max * POROG_SP / 100;
  typedef struct
  {
    uint32_t index : bits;
  } buff_index_t;

  typedef struct
  {
    buff_index_t ptr;
    int32_t maxCorr;
    /// @brief задержка 
    uint32_t delay;
  }SP_max_t;
  typedef struct
  {
    buff_index_t ptr;
    int32_t maxCorr;
    int_fast8_t BestIdx;
    uint_fast8_t Qamp;
    uint_fast8_t Qregard;
    SP_max_t loMax;
    SP_max_t hiMax;
  } SP_result_t;

  uint8_t Code[CODELEN];
  uint8_t CodeQ[CODELEN];
  bool CodeReadyFlag;

  INLN uint32_t NextDma(void)
  {
    uint32_t res = ptr_dma.index;
    if (res < buflenEx) 
    {
      dma_status = 0;
      bufExPtr = (uint32_t)&buf[res+buflen];
    }
    else
    {
      dma_status = 1;
    }    
    ptr_dma.index += dmalen;
    return (uint32_t)&buf[res];
  }
  INLN uint32_t NextDmaEx(void)
  {
    return bufExPtr;
  }
  INLN void NextDataIRQ(void) 
  {
    dmaCnt += dmalen;
  }
  INLN void NextDataPRG(void) 
  {
    if (state < 0)
      nextSp();
    if (state >= 0)
      nextCod();
  }
  int_fast8_t state = FIND_SP;
  SP_result_t SpRes;
  uint_fast8_t dma_status;

private:
  typedef int32_t (*decodefunc_t)(int16_t *buf);

  volatile uint32_t dmaCnt;
  uint32_t SpCnt;
  buff_index_t ptr_dma;
  buff_index_t ptr;
  uint32_t Dtakt;
  uint32_t DtaktCod;
  int16_t buf[buflen + buflenEx];
  uint32_t bufExPtr;  
  SP_max_t spMaxHi;
  SP_max_t spMaxLo;  
  const decodefunc_t decodeN[32] = {
        decode0,decode1,decode2,decode3,decode4,decode5,decode6,decode7,decode8,decode9,
        decode10,decode11,decode12,decode13,decode14,decode15,decode16,decode17,decode18,decode19,
        decode20,decode21,decode22,decode23,decode24,decode25,decode26,decode27,decode28,decode29,decode30,decode31};

  INLN void nextSp(void) //__attribute__((optimize("-Ofast")))
  {
    while (dmaCnt >= splen + STEP_LEN)//SMP_PER_SYM / 2)
    {
      uint32_t takt = TIM1->CNT;
      
      int32_t rescur = decodeSp(&buf[ptr.index]);

      if (rescur >= spMaxLo.maxCorr)
      {
        spMaxLo.maxCorr = rescur;
        spMaxLo.ptr = ptr;
        spMaxLo.delay = 0;
      }
      else if (spMaxLo.delay == delayTest)
      {
        if (spMaxLo.maxCorr >= spMaxHi.maxCorr)
        {
          // допустим это СП
          spMaxHi = spMaxLo;
          // ищем вторичные максимумы  после СП
          spMaxLo.maxCorr = 0;// spMaxHi.maxCorr * ((REGARD_SP-1)/100);
        }
      }
      else if (spMaxHi.delay >= buflen - splen - dmalen)
      {
    	  //uint_fast8_t po = 100 *(1 - (float)spMaxLo.maxCorr / spMaxHi.maxCorr);
    	  uint_fast8_t po = 100 - 100 * spMaxLo.maxCorr / spMaxHi.maxCorr;
                                
        if (po >= REGARD_SP)
        {
          SpRes.Qregard = po;
          const int_fast8_t Delta = STEP_LEN;// SMP_PER_SYM / 2;
          SpRes.maxCorr = 0;

          int_fast8_t BestIdx = -Delta;
          buff_index_t p = spMaxHi.ptr;
          p.index -= Delta;
          for (int_fast8_t i = -Delta; i < Delta; i++)
          {
            int32_t r = decodeSp(&buf[p.index]);
            if (SpRes.maxCorr <= r)
            {
              SpRes.maxCorr = r;
              SpRes.ptr = p;
              BestIdx = i;
            }
            p.index++;
          }
          SpRes.Qamp = SpRes.maxCorr / 32 / 4 * 100 / porogADC_max;
          SpRes.BestIdx = BestIdx;
          SpRes.loMax = spMaxLo;
          SpRes.hiMax = spMaxHi;

          // correct ptr and coundt to best Q, skip SP
          ptr.index = SpRes.ptr.index + splen;
          int32_t deltaDma = splen + BestIdx - spMaxHi.delay;
          DMA_CRITICAL(dmaCnt -= deltaDma);

          state = FIND_CODE;

          return;
        }
        spMaxLo.maxCorr = 0;
        spMaxHi.maxCorr = 0;
        spMaxHi.delay = 0;
      }//*/

     /* if (rescur >= porogsp * 32 * 4)
      {
        const int_fast8_t Delta = STEP_LEN;//SMP_PER_SYM/2;
        SpRes.maxCorr = 0;

        int_fast8_t BestIdx = -Delta;
        buff_index_t p = ptr;
        p.index -= Delta;
        for (int_fast8_t i = -Delta; i < Delta; i++)
        {
          int32_t r = decodeSp(&buf[p.index]);
          if (SpRes.maxCorr <= r)
          {
            SpRes.maxCorr = r;
            SpRes.ptr = p;
            BestIdx = i;
          }
          p.index++;
        }
        SpRes.Qamp = SpRes.maxCorr / 32 / 4 * 100 / porogADC_max;
        SpRes.BestIdx = BestIdx;
        
        // correct ptr and coundt to best Q, skip SP        
        ptr.index = SpRes.ptr.index + splen;        
        DMA_CRITICAL(dmaCnt -= splen + BestIdx);
        
        state = FIND_CODE;
        return;
      }//*/ 

      ptr.index += STEP_LEN;
      DMA_CRITICAL(dmaCnt -= STEP_LEN);
      SpCnt += 1;
      spMaxHi.delay += STEP_LEN;
      spMaxLo.delay += STEP_LEN;
      Dtakt = TIM1->CNT - takt;
    }
  }

#pragma GCC push_options
#pragma GCC optimize ("-Oz")
  void nextCod(void)
  {
    while (dmaCnt >= codlen)// + SMP_PER_SYM / 2)
    {

      uint32_t takt = TIM1->CNT;

      int32_t BestRes = 0;
      uint_fast8_t BestCode = 0;
      for (uint_fast8_t i = 0; i < 32; i++)
      {
        int32_t r = decodeN[i](&buf[ptr.index]);
        if (r > BestRes)
        {
          BestRes = r;
          BestCode = i;
        }
      }
      Code[state] = BestCode;
      CodeQ[state] = BestRes / 32 * 100 / porogADC_max;

      ptr.index += codlen;
      DMA_CRITICAL(dmaCnt -= codlen);

      DtaktCod = TIM1->CNT - takt;

      if (++state == CODELEN)
      {
        state = FIND_SP;
        spMaxLo.maxCorr = 0;
        spMaxHi.maxCorr = 0;
        spMaxHi.delay = 0;
        SpCnt = 0;
        CodeReadyFlag = true;
        break;
      }
    }
  }
#pragma GCC pop_options

  // INLN float decodeSp(buff_index_t p)
  INLN int32_t decodeSp(int16_t *buf) // __attribute__((section (".ccmram")))
  {
    int32_t res = 0;
      // step:0
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:1
      res += *buf;
      buf += SMP_PER_SYM;
      // step:2
      res += *buf;
      buf += SMP_PER_SYM;
      // step:3
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:4
      res += *buf;
      buf += SMP_PER_SYM;
      // step:5
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:6
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:7
      res += *buf;
      buf += SMP_PER_SYM;
      // step:8
      res += *buf;
      buf += SMP_PER_SYM;
      // step:9
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:10
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:11
      res += *buf;
      buf += SMP_PER_SYM;
      // step:12
      res += *buf;
      buf += SMP_PER_SYM;
      // step:13
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:14
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:15
      res += *buf;
      buf += SMP_PER_SYM;
      // step:16
      res += *buf;
      buf += SMP_PER_SYM;
      // step:17
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:18
      res += *buf;
      buf += SMP_PER_SYM;
      // step:19
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:20
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:21
      res += *buf;
      buf += SMP_PER_SYM;
      // step:22
      res += *buf;
      buf += SMP_PER_SYM;
      // step:23
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:24
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:25
      res += *buf;
      buf += SMP_PER_SYM;
      // step:26
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:27
      res += *buf;
      buf += SMP_PER_SYM;
      // step:28
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:29
      res += *buf;
      buf += SMP_PER_SYM;
      // step:30
      res += *buf;
      buf += SMP_PER_SYM;
      // step:31
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:32
      res += *buf;
      buf += SMP_PER_SYM;
      // step:33
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:34
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:35
      res += *buf;
      buf += SMP_PER_SYM;
      // step:36
      res += *buf;
      buf += SMP_PER_SYM;
      // step:37
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:38
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:39
      res += *buf;
      buf += SMP_PER_SYM;
      // step:40
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:41
      res += *buf;
      buf += SMP_PER_SYM;
      // step:42
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:43
      res += *buf;
      buf += SMP_PER_SYM;
      // step:44
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:45
      res += *buf;
      buf += SMP_PER_SYM;
      // step:46
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:47
      res += *buf;
      buf += SMP_PER_SYM;
      // step:48
      res += *buf;
      buf += SMP_PER_SYM;
      // step:49
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:50
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:51
      res += *buf;
      buf += SMP_PER_SYM;
      // step:52
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:53
      res += *buf;
      buf += SMP_PER_SYM;
      // step:54
      res += *buf;
      buf += SMP_PER_SYM;
      // step:55
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:56
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:57
      res += *buf;
      buf += SMP_PER_SYM;
      // step:58
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:59
      res += *buf;
      buf += SMP_PER_SYM;
      // step:60
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:61
      res += *buf;
      buf += SMP_PER_SYM;
      // step:62
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:63
      res += *buf;
      buf += SMP_PER_SYM;
      // step:64
      res += *buf;
      buf += SMP_PER_SYM;
      // step:65
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:66
      res += *buf;
      buf += SMP_PER_SYM;
      // step:67
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:68
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:69
      res += *buf;
      buf += SMP_PER_SYM;
      // step:70
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:71
      res += *buf;
      buf += SMP_PER_SYM;
      // step:72
      res += *buf;
      buf += SMP_PER_SYM;
      // step:73
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:74
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:75
      res += *buf;
      buf += SMP_PER_SYM;
      // step:76
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:77
      res += *buf;
      buf += SMP_PER_SYM;
      // step:78
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:79
      res += *buf;
      buf += SMP_PER_SYM;
      // step:80
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:81
      res += *buf;
      buf += SMP_PER_SYM;
      // step:82
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:83
      res += *buf;
      buf += SMP_PER_SYM;
      // step:84
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:85
      res += *buf;
      buf += SMP_PER_SYM;
      // step:86
      res += *buf;
      buf += SMP_PER_SYM;
      // step:87
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:88
      res += *buf;
      buf += SMP_PER_SYM;
      // step:89
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:90
      res += *buf;
      buf += SMP_PER_SYM;
      // step:91
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:92
      res += *buf;
      buf += SMP_PER_SYM;
      // step:93
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:94
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:95
      res += *buf;
      buf += SMP_PER_SYM;
      // step:96
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:97
      res += *buf;
      buf += SMP_PER_SYM;
      // step:98
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:99
      res += *buf;
      buf += SMP_PER_SYM;
      // step:100
      res += *buf;
      buf += SMP_PER_SYM;
      // step:101
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:102
      res += *buf;
      buf += SMP_PER_SYM;
      // step:103
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:104
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:105
      res += *buf;
      buf += SMP_PER_SYM;
      // step:106
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:107
      res += *buf;
      buf += SMP_PER_SYM;
      // step:108
      res += *buf;
      buf += SMP_PER_SYM;
      // step:109
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:110
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:111
      res += *buf;
      buf += SMP_PER_SYM;
      // step:112
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:113
      res += *buf;
      buf += SMP_PER_SYM;
      // step:114
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:115
      res += *buf;
      buf += SMP_PER_SYM;
      // step:116
      res += *buf;
      buf += SMP_PER_SYM;
      // step:117
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:118
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:119
      res += *buf;
      buf += SMP_PER_SYM;
      // step:120
      res += *buf;
      buf += SMP_PER_SYM;
      // step:121
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:122
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:123
      res += *buf;
      buf += SMP_PER_SYM;
      // step:124
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:125
      res += *buf;
      buf += SMP_PER_SYM;
      // step:126
      res -= *buf;
      buf += SMP_PER_SYM;
      // step:127
      res += *buf;
      buf += SMP_PER_SYM;
    
    return res;
  }

  static int32_t decode0(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode1(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode2(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode3(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode4(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode5(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode6(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode7(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode8(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode9(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode10(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode11(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode12(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode13(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode14(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode15(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode16(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode17(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode18(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode19(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode20(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode21(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode22(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:1
    res += *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode23(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res += *buf;
    buf += SMP_PER_SYM;
    // step:17
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode24(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode25(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode26(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode27(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res += *buf;
    buf += SMP_PER_SYM;
    // step:3
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res += *buf;
    buf += SMP_PER_SYM;
    // step:19
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode28(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode29(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res += *buf;
    buf += SMP_PER_SYM;
    // step:5
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:6
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:7
    res += *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:21
    res += *buf;
    buf += SMP_PER_SYM;
    // step:22
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:23
    res += *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode30(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res += *buf;
    buf += SMP_PER_SYM;
    // step:9
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:10
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:11
    res += *buf;
    buf += SMP_PER_SYM;
    // step:12
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:13
    res += *buf;
    buf += SMP_PER_SYM;
    // step:14
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:15
    res += *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:25
    res += *buf;
    buf += SMP_PER_SYM;
    // step:26
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:27
    res += *buf;
    buf += SMP_PER_SYM;
    // step:28
    res += *buf;
    buf += SMP_PER_SYM;
    // step:29
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:30
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:31
    res += *buf;
    buf += SMP_PER_SYM;
    return res;
  }
  static int32_t decode31(int16_t *buf)
  {
    int32_t res = 0;
    // step:0
    res += *buf;
    buf += SMP_PER_SYM;
    // step:1
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:2
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:3
    res += *buf;
    buf += SMP_PER_SYM;
    // step:4
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:5
    res += *buf;
    buf += SMP_PER_SYM;
    // step:6
    res += *buf;
    buf += SMP_PER_SYM;
    // step:7
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:8
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:9
    res += *buf;
    buf += SMP_PER_SYM;
    // step:10
    res += *buf;
    buf += SMP_PER_SYM;
    // step:11
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:12
    res += *buf;
    buf += SMP_PER_SYM;
    // step:13
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:14
    res += *buf;
    buf += SMP_PER_SYM;
    // step:15
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:16
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:17
    res += *buf;
    buf += SMP_PER_SYM;
    // step:18
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:19
    res += *buf;
    buf += SMP_PER_SYM;
    // step:20
    res += *buf;
    buf += SMP_PER_SYM;
    // step:21
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:22
    res += *buf;
    buf += SMP_PER_SYM;
    // step:23
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:24
    res += *buf;
    buf += SMP_PER_SYM;
    // step:25
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:26
    res += *buf;
    buf += SMP_PER_SYM;
    // step:27
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:28
    res -= *buf;
    buf += SMP_PER_SYM;
    // step:29
    res += *buf;
    buf += SMP_PER_SYM;
    // step:30
    res += *buf;
    buf += SMP_PER_SYM;
    // step:31
    res -= *buf;
    buf += SMP_PER_SYM;
    return res;
  }
};

#endif /* SRC_ADECODER_H_ */
