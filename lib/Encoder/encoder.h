#ifndef encoder_h
#define encoder_h

#define BUFFERSIZE 50
#define BUFFER_THROW_DEPTH BUFFERSIZE

typedef signed short PulseS;
typedef unsigned short Pulse;

typedef struct {
    bool moving;
    signed short slope;
    signed short angle;
    signed short filtered_raw_angle;
    signed short raw_angle;
    signed short offset;

    signed short period;

    int pwmPin;

    signed short buffer[BUFFERSIZE];
} Encoder;

Encoder*          encoder_new      (int pwmPin);
void              encoder_update   (Encoder* encoder);
void              encoder_zero     (Encoder* encoder);

#endif