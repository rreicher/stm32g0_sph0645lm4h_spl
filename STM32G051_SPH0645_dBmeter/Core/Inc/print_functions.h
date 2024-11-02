/*
 * print_functions.h
 *
 *  Created on: Oct 22, 2024
 *      Author: reicherr
 */

#ifndef INC_PRINT_FUNCTIONS_H_
#define INC_PRINT_FUNCTIONS_H_

// Functions for printing text/data to an unspecified output device.

#include <stdint.h>
#include <stdbool.h>

void print(const char* format, ...);
void print64hex(int64_t n);
void printU64hex(uint64_t x);


#endif /* INC_PRINT_FUNCTIONS_H_ */
