/*
 * efficient_10log10.h
 *
 *  Created on: Oct 22, 2024
 *      Author: reicherr
 */

#ifndef INC_EFFICIENT_10LOG10_H_
#define INC_EFFICIENT_10LOG10_H_

#include <stdint.h>

void efficient10log10(uint64_t P, int32_t * integerPart, int32_t * fractionalPart);
void correctIntFracNumber(int32_t * intPart, int32_t * fracPart);


#endif /* INC_EFFICIENT_10LOG10_H_ */
