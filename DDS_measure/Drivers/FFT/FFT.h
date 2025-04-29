#ifndef _FFT_H
#define _FFT_H

#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_SIZE 4096					// 采样数量

void FFT_start(void);
void FFT_getValue(void);

#endif
