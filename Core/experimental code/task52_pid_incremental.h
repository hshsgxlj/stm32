#ifndef TASK52_PID_INCREMENTAL_H
#define TASK52_PID_INCREMENTAL_H

typedef struct {
    float kp;
    float ki;
    float kd;
} Task52_PID_TypeDef;

/* 实验5任务2：增量式 PID，返回本次 PWM 增量。 */
void Task52_PID_IncrementalInit(float kp, float ki, float kd);
float Task52_PID_IncrementalUpdate(float target, float actual);

#endif /* TASK52_PID_INCREMENTAL_H */
