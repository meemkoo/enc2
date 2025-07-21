#include <Arduino.h>
#include <panic.h>
#include <string.h>
#include "encoder.h"



Encoder* encoder_new(int pwmPin) {
  Encoder* new_encoder = (Encoder* )malloc(sizeof(Encoder));
  new_encoder->angle = 0;
  new_encoder->raw_angle = 0;
  new_encoder->offset = 0;
  new_encoder->period = 0;

  new_encoder->pwmPin = pwmPin;

  return new_encoder;
}

void encoder_update(Encoder* encoder) {
    unsigned long highTime;
    static unsigned long i = 0;

    // disabling inturrupts makes timing more accurate
    cli();
        highTime = pulseIn(encoder->pwmPin, HIGH);
    sei();

    encoder->raw_angle = (signed short)highTime;



    // An unsigned long because the average calculation grows above 65K
    unsigned long inter = 0;
    signed short slope = 0;

    slope = encoder->buffer[0]-encoder->buffer[BUFFER_THROW_DEPTH-1];
    slope = abs(slope);

    if (slope > (signed short)2) {
        encoder->moving = true;
        encoder->filtered_raw_angle = encoder->raw_angle;
    } else {
        encoder->moving = false;
        for(i=0; i<BUFFERSIZE; i++) {
            inter += encoder->buffer[i];
        }
        encoder->filtered_raw_angle = inter / (unsigned long)BUFFERSIZE;
    }

    // WARNING: After the memmove the array's first element is duplicated
    encoder->buffer[0] = encoder->raw_angle;
    memmove(encoder->buffer+1, encoder->buffer, (BUFFERSIZE-1)*sizeof(signed short));


    // I am not good enough at C++ to understand why modulo 
    //  would not work for wrapping the value 
    //  but this is probably faster anyway
    encoder->angle = (encoder->filtered_raw_angle - encoder->offset);
    if (encoder->angle < 0) {
        encoder->angle = 1024 + encoder->angle;
    }
}

void encoder_zero(Encoder* encoder) {
    encoder->offset = encoder->raw_angle;
}
