#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    double accumulator;
    double last;
    bool was_last;
    char pad[7];
} Pid;

/// Call to initialize the pid before sending a stream of feedbacks.
void pid_start(Pid *pid);
/// Call with the error from the sensor to produce a new output.
double pid_output(Pid *pid, double error, double kp, double ki, double kd);

#endif // PID_H

