#ifndef TASK51_PID_POSITION_H
#define TASK51_PID_POSITION_H

typedef struct {
    float kp;
    float ki;
    float kd;
} Task51_PID_TypeDef;

/* 实验5任务1：位置式 PID。dt 单位为秒。 */
void Task51_PID_PositionInit(float kp, float ki, float kd);
float Task51_PID_PositionUpdate(float target, float actual, float dt);

#endif /* TASK51_PID_POSITION_H */
