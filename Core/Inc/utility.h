#ifndef UTILITY_H
#define UTILITY_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


void LedSet(uint8_t led_no, uint8_t state);
uint8_t InputRead(uint8_t input_no);

uint8_t SafetyStateRead();
void FaultOutputSet(uint8_t state);

void AIR_MINUS_Set(uint8_t state);
void AIR_PLUS_Set(uint8_t state);
void AIR_PRECHARGE_Set(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif
