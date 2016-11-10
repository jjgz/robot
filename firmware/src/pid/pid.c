#include "pid.h"

void pid_start(Pid *pid) {
    pid->accumulator = 0.0;
    pid->was_last = false;
}

double pid_output(Pid *pid, double error, double kp, double ki, double kd) {
    pid->accumulator += error;
    if (pid->was_last) {
        double out = error * kp + pid->accumulator * ki + (error - pid->last) * kd;
        pid->last = error;
        return out;
    } else {
        double out = error * kp + pid->accumulator * ki;
        pid->last = error;
        pid->was_last = true;
        return out;
    }
}
