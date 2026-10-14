#include "task51_pid_position.h"

static float task51_integral;
static float task51_previous_error;
static Task51_PID_TypeDef task51_config;

void Task51_PID_PositionInit(float kp, float ki, float kd)
{
    task51_config.kp = kp;
    task51_config.ki = ki;
    task51_config.kd = kd;
    task51_integral = 0.0f;
    task51_previous_error = 0.0f;
}

float Task51_PID_PositionUpdate(float target, float actual, float dt)
{
    float error = target - actual;
    float derivative;

    if (dt <= 0.0f) {
        dt = 0.001f;
    }
    task51_integral += error * dt;
    derivative = (error - task51_previous_error) / dt;
    task51_previous_error = error;
    return (task51_config.kp * error) +
           (task51_config.ki * task51_integral) +
           (task51_config.kd * derivative);
}
