#include "pid.h"

#define PID_INTEGRAL_OUTPUT_LIMIT 2200.0f

static float PID_Clamp(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;

    pid->output_min = out_min;
    pid->output_max = out_max;
}

float PID_Update(PID_t *pid, float target, float feedback, float dt)
{
    float derivative;
    float next_integral;
    float unclamped_output;
    float integral_min;
    float integral_max;
    float temp;

    if (dt <= 0.0f)
    {
        return pid->output;
    }

    pid->error = target - feedback;

    pid->p_out = pid->kp * pid->error;

    next_integral = pid->integral + pid->error * dt;
    if (pid->ki != 0.0f)
    {
        integral_min = pid->output_min / pid->ki;
        integral_max = pid->output_max / pid->ki;
        if (integral_min < -PID_INTEGRAL_OUTPUT_LIMIT / pid->ki)
        {
            integral_min = -PID_INTEGRAL_OUTPUT_LIMIT / pid->ki;
        }
        if (integral_max > PID_INTEGRAL_OUTPUT_LIMIT / pid->ki)
        {
            integral_max = PID_INTEGRAL_OUTPUT_LIMIT / pid->ki;
        }
        if (integral_min > integral_max)
        {
            temp = integral_min;
            integral_min = integral_max;
            integral_max = temp;
        }
        next_integral = PID_Clamp(next_integral, integral_min, integral_max);
    }
    pid->i_out = pid->ki * next_integral;

    derivative = (pid->error - pid->last_error) / dt;
    pid->d_out = pid->kd * derivative;

    unclamped_output = pid->p_out + pid->i_out + pid->d_out;
    if ((unclamped_output > pid->output_max && pid->error > 0.0f) ||
        (unclamped_output < pid->output_min && pid->error < 0.0f))
    {
        pid->i_out = pid->ki * pid->integral;
        unclamped_output = pid->p_out + pid->i_out + pid->d_out;
    }
    else
    {
        pid->integral = next_integral;
    }

    pid->output = PID_Clamp(unclamped_output, pid->output_min, pid->output_max);

    pid->last_error = pid->error;

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
}
