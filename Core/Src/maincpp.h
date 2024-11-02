/*
 * maincpp.h
 *
 *  Created on: Sep 30, 2024
 *      Author: User
 */

#ifndef SRC_MAINCPP_H_
#define SRC_MAINCPP_H_
#include "main.h"

void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac) __attribute__((section (".ccmram")));
class maincpp {
};

#endif /* SRC_MAINCPP_H_ */
