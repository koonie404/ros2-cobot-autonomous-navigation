#include "Encoder.h"

static void IRAM_ATTR Encoder_C1_ISR(void* arg) {
  Encoder* enc=(Encoder*)arg;  
  byte val1=digitalRead(enc->C1);
  byte val2=digitalRead(enc->C2);
  if(val1==val2) enc->pos++;
  else enc->pos--;
}

static void IRAM_ATTR Encoder_C2_ISR(void* arg) {
  Encoder* enc=(Encoder*)arg;
  byte val1=digitalRead(enc->C1);
  byte val2=digitalRead(enc->C2);
  if(val1!=val2) enc->pos++;
  else enc->pos--;
}

void Encoder_begin(Encoder *enc) {
  pinMode(enc->C1, INPUT);
  pinMode(enc->C2, INPUT);
  attachInterruptArg(enc->C1, Encoder_C1_ISR, enc, CHANGE);
  attachInterruptArg(enc->C2, Encoder_C2_ISR, enc, CHANGE);    
}

long Encoder_getPos(Encoder *enc) {
  portENTER_CRITICAL(&enc->encMux);
  long tmp_pos=enc->pos;
  portEXIT_CRITICAL(&enc->encMux);
  return tmp_pos;
}