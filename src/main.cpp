#include <Arduino.h>
// #include <encoder.h>
// #include <PID_v1_bc.h>

// #define PIN_LIMIT_0 7
// #define PIN_ENCODER_0 2
// #define PIN_LED_0 3

// unsigned long highTime, lowTime, period;
// float dutyCycle;

// volatile bool tz = false;

// Encoder* flipper_encoder;

// void zint() {
//   if (!tz) {
//     encoder_zero(flipper_encoder);
//     tz = true;
//   } else if (tz) {
//     tz = false;
//   }
// }


// // PIND@0=A , PIND@4=B


// // Define PID parameters
// double Kp = 2, Ki = 0, Kd = 0;
// double Setpoint, Input, Output;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// void setup() {
//   Serial.begin(115200);
//   pinMode(PIN_ENCODER_0, INPUT);
//   pinMode(PIN_LIMIT_0, INPUT_PULLUP);

//   flipper_encoder = encoder_new(PIN_ENCODER_0);

//   attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_0), zint, CHANGE);

//   myPID.SetMode(AUTOMATIC);
//   myPID.SetSampleTime(50); // milliseconds
//   myPID.SetOutputLimits(0, 255);
// }

// uint8_t i21 = 0;
// int pos = 0;

// void loop() {
//   encoder_update(flipper_encoder);
//   Serial.print(flipper_encoder->angle);
//   Serial.print("\t");
//   // Read input (e.g., from a sensor)
//   Input = map(analogRead(A5), 665, 675, 255, 0);
//   Serial.print(analogRead(A5));
//   Serial.print("\t");
//   Serial.print((long)Input);
//   Serial.print("\t");
//   // Input = (double)flipper_encoder->angle;

//   // Set the setpoint (e.g., from a potentiometer)
//   Setpoint = map(analogRead(A0), 0, 1023, 0, 255);
//   Serial.print(Setpoint);
//   Serial.print("\t");
//   // Setpoint = 25;//map(500, 0, 1023, 0, 255);

//   // Compute the PID output
//   myPID.Compute();
//   Output;
//   Serial.print("\t");
//   Serial.print(Output);
//   Serial.print("\n");

//   // Apply the output to the system (e.g., control an LED)
//   analogWrite(PIN_LED_0, Output);
// }

// #include <Arduino.h>

// #define BUFFERSIZE2 100
// #define KP 1
// #define KI 0
// #define KD 0

// int sensor;
// int setpoint;
// int error;
// int output;
// int preverror;
// int ledstate;

// int min, max, in, ctl, avg;

// int buffer2[BUFFERSIZE2];

// void setup() {
//   Serial.begin(11200);
//   min = analogRead(A5);
//   max = analogRead(A5);
//   in = 0;
//   ctl = 0;
//   avg = 0;
//   ledstate = 0;
// }

// void loop() {
//   unsigned long inter = 0;
//   ctl = map(map(analogRead(A0), 0, 1023, 0, 613), 0, 613, 0, 255);

//   in = ledstate; analogRead(A5);

//   for(i21=0; i21<BUFFERSIZE2; i21++) {
//     inter += buffer2[i21];
//   }
//   avg = inter / (unsigned long)BUFFERSIZE2;

//   buffer2[0] = in;
//   memmove(buffer2+1, buffer2, (BUFFERSIZE2-1)*sizeof(int));

//   bool flag = false;
//   for(i21=0; i21<BUFFERSIZE2; i21++) {
//     if (buffer2[i21]==0) {
//       flag = true;
//     };
//   }

//   if (!flag) {
//     if (avg < min) {
//       min = avg;
//     } else if (avg > max) {
//       max = avg;
//     }
//   }

//   sensor = map(avg, min, max, 0, 255);
//   setpoint = ctl;
//   error = setpoint - sensor;
//   output = (KP*error);
//   ledstate += output;


//   Serial.print(ctl);
//   Serial.print("\t");
//   Serial.print(min);
//   Serial.print("\t");
//   Serial.print(max);
//   Serial.print("\t");
//   Serial.print(in);
//   Serial.print("\t");
//   Serial.print(avg);
//   Serial.print("\t");
//   Serial.print(sensor);
//   Serial.print("\t");
//   Serial.print(error);
//   Serial.print("\t");
//   Serial.print(ledstate);
//   Serial.print("\t");
//   Serial.print(output);
//   Serial.print("\n");

  
//   analogWrite(PIN_LED_0, ledstate);
// }

#include <Arduino.h>
#include <encoder.h>
#include <Servo.h>

#define encoder 5
#define limit 7
#define motor 3
#define signal 6

#define st Serial.print("\t")
#define sn Serial.print("\n")
#define sp(x) Serial.print(x)
#define spt(x) Serial.print(x);Serial.print("\t")

Encoder* flipper_encoder;
bool motor_kill;
bool limit_hit;
int motorin;
unsigned long signal_in;




Servo Smotor;
const int minMicros = 1100; // Minimum pulse width in microseconds
const int maxMicros = 1900; // Maximum pulse width in microseconds
const int halfSpeedMicros = (maxMicros + minMicros) / 2; // Halfway between min and max




void zint() {

}

void setup() {
  interrupts();
  motor_kill = true;
  limit_hit = false;

  flipper_encoder = encoder_new(encoder);
  pinMode(signal, INPUT);
  pinMode(limit, INPUT_PULLUP);

  pinMode(motor, OUTPUT);
  pinMode(A0, INPUT);
  Smotor.attach(motor);

  Serial.begin(11200);
}


void loop() {
  encoder_update(flipper_encoder);
  if (!digitalRead(limit)) {
    limit_hit = true;
  } else {
    limit_hit = false;
  }

  if (limit_hit) {
    encoder_zero(flipper_encoder);
  }

  if ((150 < flipper_encoder->angle) && (flipper_encoder->angle < 360)) {
    motor_kill = false;
  } else {
    motor_kill = true;
  }

  signal_in = pulseIn(signal, HIGH);


  //if (!motor_kill) {
    // motorin = pulseIn(A0, HIGH);
    // pinMode(motor, OUTPUT);
    // Smotor.attach(motor);
    Smotor.writeMicroseconds(signal_in);
  // }
  // else {
  //   // Smotor.detach();
  //   pinMode(motor, INPUT);
  //   // Smotor.writeMicroseconds(0);
  // }


  spt(flipper_encoder->angle);
  spt(signal_in);
  spt(halfSpeedMicros-map(signal_in, 980, 1980, 0, 255)-(255/2));
  spt(limit_hit);
  spt(motor_kill);
  spt(motorin);
  sn;
}
