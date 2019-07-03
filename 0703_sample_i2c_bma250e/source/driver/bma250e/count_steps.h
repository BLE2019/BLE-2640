#ifndef count_steps_h
#define count_steps_h
#include "stdint.h"

#define SAMPLING_RATE           15                       //20 hz sampling rate
#define NUM_TUPLES              60                       //80 sets of accelerometer readings (so in other words, 80*3 = 240 samples)
#define WINDOW_LENGTH           NUM_TUPLES/SAMPLING_RATE //window length in seconds

#define NUM_SAMPLES_IN_CSV_FILE 400
uint8_t count_steps(uint16_t *data);
uint16_t count_acc_step(uint16_t* acc);
#endif /* count_steps_h */