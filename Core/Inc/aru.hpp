/*
 * aru.h
 *
 *  Created on: Nov 1, 2024
 *      Author: User
 */

#ifndef ARU_H_
#define ARU_H_

#include <stddef.h>
#include <stdint.h>
#include "maincpp.h"
#include "adecoder.hpp"

enum AruMmode
{
    NOISE,
    DATA,
};

class aru_t
{
private:
public:
   
   static const uint32_t TIMER_ONE_SECOND_TAKTS = 1000000;
   static const uint32_t DATA_IN_TAKTS = decinst_t::splen + (decinst_t::codlen)*(decinst_t::codesCount+3);
   static const uint32_t ONE_CODE_TICS = (float) decinst_t::codlen*TIMER_ONE_SECOND_TAKTS/decinst_t::TIMER_ONE_SECOND_TAKTS;

   static const uint32_t TIMER_DATA_TICKS = (float) DATA_IN_TAKTS * TIMER_ONE_SECOND_TAKTS/decinst_t::TIMER_ONE_SECOND_TAKTS;
   static const uint32_t TIMER_NOISE_TICKS = TIMER_ONE_SECOND_TAKTS - TIMER_DATA_TICKS;

   uint32_t Sync;
   uint32_t DataGuardCnt = 10000;
   uint32_t NoiseGuardCnt = 10000;
   uint32_t Gain;
   uint_fast8_t spQamp;
   uint_fast8_t codesQavg;
   AruMmode aruMmode;
   uint32_t timer_ARR;
   INLN void OnCodeEnd(decinst_t& decoder)
   {
        spQamp = decoder.SpRes.Qamp;
        Sync++;
        aruMmode = DATA;
        timer_ARR = TIMER_DATA_TICKS;
        uint32_t sumQ = 0;
        for (size_t i = 0; i < decoder.codesCount; i++)
        {
            sumQ += decoder.CodeQ[i];
        }
        codesQavg = sumQ/decoder.codesCount;
   }
   INLN void OnTimer(uint32_t guardCnt)
   {
        if (aruMmode == NOISE)
        {
            NoiseGuardCnt = guardCnt;
            aruMmode = DATA;
            timer_ARR = TIMER_DATA_TICKS;
        }
        else
        {
            DataGuardCnt = guardCnt;
            aruMmode = NOISE;
            timer_ARR = TIMER_NOISE_TICKS;

            if (DataGuardCnt == 0 && NoiseGuardCnt == 0)
            {
                Gain = OpampPGA(1);     
            }

            if (Sync)
            {
                if (NoiseGuardCnt >= 2000 || DataGuardCnt >= 1000)
                {
                    Gain = OpampPGA(-1);
                }
            }
            else
            {
                if (NoiseGuardCnt + DataGuardCnt >= 4000)
                {
                    Gain = OpampPGA(-1);
                }
            }
        }
   }
   // void UpdateARU(uint32_t guardCnt)
   // {
   //    gurdcnt = guardCnt;
   //    // if (gurdcnt >) 
   // }
};

// extern aru_t aru;


#endif /* ARU_H_ */
