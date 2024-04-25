#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "bsp_dwt.h"
#include <stdbool.h>
#include <pid.h>
shooter_t shooter; // 发射机构信息结构体
// 电机0为拨盘电机，电机1为弹舱盖电机，电机2、3为摩擦轮电机
#define TRIGGER_SINGLE_ANGLE 1620 // 36*360/8

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体
bool is_angle_control = false;//单发
float current_time = 0;
float last_time = 0;
bool flag=1;
static void Shooter_Inint();         // 发射机构的初始化
static void model_choice();          // 模式选择
static void dial_control();          // 拨盘电机控制
static void friction_control();      // 摩擦轮电机控制
static void bay_control();           // 弹舱电机控制
static void shooter_current_given(); // 给电流
// 拨盘旋转固定角度
static void trigger_single_angle_move();
static void shoot_reverse();

void Shoot_task(void const *pvParameters)
{
    Shooter_Inint();
    for (;;)
    {
        model_choice();
        shooter_current_given();
        osDelay(1);
    }
}

// 发射机构的初始化
static void Shooter_Inint(void)
{
    // 初始化pid参数
    shooter.pid_dial_para[0] = 20, shooter.pid_dial_para[1] = 0, shooter.pid_dial_para[2] = 0;
    shooter.pid_friction_para[0] = 30, shooter.pid_friction_para[1] = 0, shooter.pid_friction_para[2] = 0;
    shooter.pid_bay_para[0] = 10, shooter.pid_bay_para[1] = 0, shooter.pid_bay_para[2] = 0;

    shooter.pid_angle_value[0] = 10;
    shooter.pid_angle_value[1] = 0.05;
    shooter.pid_angle_value[2] = 500;
    // 初始化pid结构体
    pid_init(&shooter.pid_dial, shooter.pid_dial_para, 10000, 10000);
    pid_init(&shooter.pid_angle, shooter.pid_angle_value, 20000, 30000); // trigger_angle

    pid_init(&shooter.pid_friction, shooter.pid_friction_para, 20000, 20000);
    pid_init(&shooter.pid_bay, shooter.pid_bay_para, 10000, 10000);

    // 初始化速度目标
    shooter.dial_speed_target = 0;
    shooter.target_angle = shooter.motor_info[0].total_angle;

    shooter.friction_speed_target[0] = 0, shooter.friction_speed_target[1] = 0;//两个摩擦轮速度
    shooter.bay_speed_target = 0;
}

// 模式选择
static void model_choice(void)
{
    // 取消注释开始发射
    bay_control();
    // 取消注释开始发射
    if (rc_ctrl.rc.s[1] == 3 || rc_ctrl.rc.s[1] == 1)
    {
        // 发射
        friction_control();
        // dial_control();
        // 右拨杆中，键鼠控制
        if (rc_ctrl.rc.s[0] == 3)
        {
            // 单发，鼠标控制
            if (flag)
            {
                // HAL_Delay(5000);
                is_angle_control = true;
                trigger_single_angle_move();
                flag=0;
            }
            
            // else if (z_flag)
            // {
            //     is_angle_control = false;
            //     shoot_reverse();
            // }
        }

        // 右拨杆下，遥控器控制
        // else
        else if (rc_ctrl.rc.s[1] ==1)
        {
            dial_control();
            is_angle_control = false;
        }
        else if (rc_ctrl.rc.s[0] == 2)
        {
            flag=1;
        //     // 左拨杆上，电机启动
        //     if (rc_ctrl.rc.s[1] == 1)
        //     {
                // is_angle_control = false;
        //         // shoot_start();
        //         shooter.dial_speed_target = 2000;
        //     }
        //     else
        //     {
        //         // shoot_stop();
        //         shooter.dial_speed_target = 0;
        //     }
        }
    }
    else
    {
        shooter.dial_speed_target = 0;
        shooter.motor_info[0].set_current=0;
            shooter.bay_speed_target = 0;
        // 停止
        shooter.friction_speed_target[0] = 0;
        shooter.friction_speed_target[1] = 0;
    }
}

// 拨盘电机控制
static void dial_control(void)
{
    if (rc_ctrl.rc.s[1] == 1)
    {
        LEDR_OFF();
        shooter.dial_speed_target = 2000;
    }
    else
    {
        shooter.dial_speed_target = 0;
    }

    // if (rc_ctrl.rc.s[0] == 3)
    // {
        // 单发，鼠标控制
    // if (rc_ctrl.rc.s[0] == 3) // press_left
    // {
    //     is_angle_control = true;
    //     trigger_single_angle_move();
    //     HAL_Delay(5000);
    //     }
    //     else if (z_flag)
    //     {
    //         is_angle_control = false;
    //         shoot_reverse();
    //     }
    // }
}

// 摩擦轮电机控制
static void friction_control(void)
{

    shooter.friction_speed_target[0] = -8000;
    shooter.friction_speed_target[1] = 8000;
}

// 弹舱电机控制
static void bay_control(void)
{
    // 暂留
    shooter.bay_speed_target = 0;
}

// 给电流
static void shooter_current_given(void)
{
    // shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // 拨盘电机
    if (is_angle_control)
        shooter.motor_info[0].set_current = pid_calc_trigger(&shooter.pid_angle, shooter.target_angle, shooter.motor_info[0].total_angle); // 拨盘电机
    else
        shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // 拨盘电机

    shooter.motor_info[1].set_current = pid_calc(&shooter.pid_bay, shooter.motor_info[1].rotor_speed, shooter.bay_speed_target);          // 弹舱电机
    shooter.motor_info[2].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[2].rotor_speed, shooter.friction_speed_target[0]); // 摩擦轮电机
    shooter.motor_info[3].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[3].rotor_speed, shooter.friction_speed_target[1]); // 摩擦轮电机
    set_motor_current_shoot(1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
    // set_curruent(MOTOR_3508_1, hcan1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
}

/*************拨盘旋转固定角度***********/
static void trigger_single_angle_move()
{
    current_time = DWT_GetTimeline_ms();
    // 判断两次发射时间间隔，避免双发
    if (current_time - last_time > 1000)
    {
        last_time = DWT_GetTimeline_ms();
        shooter.target_angle = shooter.motor_info[0].total_angle + TRIGGER_SINGLE_ANGLE;
    }
}
/*****************反转******************/
static void shoot_reverse()///nrh瞎写的 没啥用
{
    shooter.dial_speed_target = 250;
}
