#pragma once

#include <Arduino.h>

typedef struct {
  int C1;
  int C2;
  volatile long pos;
  portMUX_TYPE encMux;
} Encoder;

static void IRAM_ATTR Encoder_C1_ISR(void* arg);
static void IRAM_ATTR Encoder_C2_ISR(void* arg);
void Encoder_begin(Encoder *enc);
long Encoder_getPos(Encoder *enc);