
#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void pid_init(pid_struct_t *pid, fp32 PID[3], fp32 max_out, fp32 max_iout) // pid结构体初始化，后两个参数分别设置输出最大值，积分最大值
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }

    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 pid_calc(pid_struct_t *pid, fp32 fdb, fp32 set) // 第二个参数为反馈值，第三个参数为目标值
{
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    pid->error[0] = set - fdb;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

fp32 gimbal_Yaw_PID_calc(pid_struct_t *pid, fp32 fdb, fp32 set) // 第二个参数为反馈值，第三个参数为目标值
{
    fp32 err;
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    err = set - fdb;

    if (err > 180)
    {
        err -= 360;
    }
    else if (err < -180)
    {
        err += 360;
    }

    pid->error[0] = err;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

fp32 gimbal_Pitch_PID_cal(pid_struct_t *pid, fp32 fdb, fp32 set) // 第二个参数为反馈值，第三个参数为目标值
{
    fp32 err;
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    err = set - fdb;

    if (err > 8191 / 2)
    {
        err -= 8191;
    }
    else if (err < -8191 / 2)
    {
        err += 8191;
    }

    pid->error[0] = err;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}
fp32 pid_calc_trigger(pid_struct_t *pid, fp32 ref, fp32 fdb) // ref是目标值,fdb是电机解码的速度返回值
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->error[1] = pid->error[0];         // err[1]是上一次计算出来的差值
    pid->error[0] = pid->ref - pid->fdb; // err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的

    if (pid->error[0] > 4096)
        pid->error[0] -= 8191;
    if (pid->error[0] < -4096)
        pid->error[0] += 8191;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);
    LimitMax(pid->Iout, pid->max_iout); // 防止越界

    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out); // 防止越界
    return pid->out;                  // 电机返回的报文有转速和转矩电流，但是只能发电压值(-30000至30000)
}