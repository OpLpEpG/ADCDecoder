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
#include <main.h>

class aru_t
{
private:
 uint32_t gurdcnt;
public:
 void UpdateARU(uint32_t guardCnt)
 {
    gurdcnt = guardCnt;
    // if (gurdcnt >) 
 }
};


#endif /* ARU_H_ */
