/*
 * adecoder.cpp
 *
 *  Created on: Oct 3, 2024
 *      Author: User
 */

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "adecoder.h"


//template<uint_fast8_t SMP_PER_SYM, uint_fast8_t CODELEN>
//float adecoder_t<SMP_PER_SYM,  CODELEN>::decodeCod(uint32_t code)
//{
//
//}

//template<uint_fast8_t SMP_PER_SYM, uint_fast8_t CODELEN>
//float adecoder_t<SMP_PER_SYM,  CODELEN>::decodeSp(void)
//{
//	buff_index_t p = ptr;
//	float res = 0;
//	decode(0x6999a656, p, res);
//	decode(0x99559655, p, res);
//	decode(0xa59556a9, p, res);
//	decode(0x5a595995, p, res);
/*	uint32_t c = 0x6999a656;//, 0x99559655, 0xa59556a9, 0x5a595995, //SP
	for(uint_fast8_t i =0; i<32; i++)
	{
		(c & 0x80000000) ? res +=buf[p.index] : res -= buf[p.index];
		p.index += SMP_PER_SYM;
		c <<= 1;
	}
	c = 0x99559655;
	for(uint_fast8_t i =0; i<32; i++)
	{
		(c & 0x80000000) ? res +=buf[p.index] : res -= buf[p.index];
		p.index += SMP_PER_SYM;
		c <<= 1;
	}
	c = 0xa59556a9;
	for(uint_fast8_t i =0; i<32; i++)
	{
		(c & 0x80000000) ? res +=buf[p.index] : res -= buf[p.index];
		p.index += SMP_PER_SYM;
		c <<= 1;
	}
	c = 0x5a595995; //SP
	for(uint_fast8_t i =0; i<32; i++)
	{
		(c & 0x80000000) ? res +=buf[p.index] : res -= buf[p.index];
		p.index += SMP_PER_SYM;
		c <<= 1;
	}*/
//	return res/128;
//}

template<uint_fast8_t SMP_PER_SYM, uint_fast8_t STEP_LEN, uint_fast8_t CODELEN>
void adecoder_t<SMP_PER_SYM, STEP_LEN,  CODELEN>::nextSp(void)
{
	while (dmaCnt >= splen+SMP_PER_SYM/2)
	{
		float rescur = decodeSp();

		if (rescur >= porogsp)
		{
			result_t rz;
			buff_index_t p;
			ptr.index -= SMP_PER_SYM/2;
			for (int_fast8_t i =0; i < SMP_PER_SYM; i++)
			{
				float r = decodeSp();
				if (rz.maxCorr <= r)
				{
					rz.maxCorr = r;
					rz.ptr = ptr;
				}
			}
			res = rz;
			state = 0;
			return;
		}

	ptr.index += STEP_LEN;
	dmaCnt -= STEP_LEN;
	SpCnt += STEP_LEN;
}
}
