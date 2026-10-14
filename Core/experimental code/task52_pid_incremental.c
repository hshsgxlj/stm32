#include "task52_pid_incremental.h"

static Task52_PID_TypeDef task52_config;
static float task52_error_1;
static float task52_error_2;

void Task52_PID_IncrementalInit(float kp, float ki, float kd)
{
    task52_config.kp = kp;
    task52_config.ki = ki;
    task52_config.kd = kd;
    task52_error_1 = 0.0f;
    task52_error_2 = 0.0f;
}

float Task52_PID_IncrementalUpdate(float target, float actual)
{
    float error = target - actual;
    float delta = (task52_config.kp * (error - task52_error_1)) +
                  (task52_config.ki * error) +
                  (task52_config.kd * (error - (2.0f * task52_error_1) +
                                       task52_error_2));
    task52_error_2 = task52_error_1;
    task52_error_1 = error;
    return delta;
}
