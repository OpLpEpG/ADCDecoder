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

#define INLN __attribute__ ((__always_inline__)) inline

#define POROG_SP 70

#define FIND_SP  (-1)

template<uint_fast8_t SMP_PER_SYM, uint_fast8_t STEP_LEN, uint_fast8_t CODELEN>
class adecoder_t {
public:
		 static const uint_fast16_t splen = 32 *4 * SMP_PER_SYM;	//in bits
		 static const uint_fast16_t codlen = 32  *  SMP_PER_SYM;	//in bits
		 static const uint_fast16_t bits = round(log2(splen*2));
		 static const uint_fast16_t buflen = 1 << bits;
		 static	const uint_fast16_t dmalen = buflen / 16;

		 static const uint_fast16_t porogsp_max = 1<<11;//12 bit adc-sign
		 static constexpr float porogsp = porogsp_max*POROG_SP/100;

		 typedef struct
		 {
		 	   uint32_t index: bits;
		 } buff_index_t;

		 typedef struct
		 {
			buff_index_t ptr;
		 	float maxCorr;
		 } result_t;

		 int16_t buf[buflen];

private:
		 size_t dmaCnt;
		 size_t SpCnt;
		 bool flagCurSp;
		 result_t res;
		 buff_index_t ptr_dma;
		 buff_index_t ptr;
		 int8_t state = FIND_SP;
		 //float decodeCod(uint32_t code) __attribute__((section (".ccmram")));
		 void nextSp(void) __attribute__((section (".ccmram")));
		 INLN float decodeSp(void)
		 {
				buff_index_t p = ptr;
				float res = 0;
				decode(0x6999a656, p, res);
				decode(0x99559655, p, res);
				decode(0xa59556a9, p, res);
				decode(0x5a595995, p, res);
				return res;
		 }
		 INLN void decode(uint32_t code, buff_index_t& p, float& res)
		 {
			for(uint_fast8_t i =0; i<32; i++)
			{
				(code & 0x80000000) ? res +=buf[p.index] : res -= buf[p.index];
				p.index += SMP_PER_SYM;
				code <<= 1;
			}
		 }

};

#endif /* SRC_ADECODER_H_ */
