#include <math.h>
class PID
{
private:
    struct heading_param
    {
        float kp;
        float ki;
        float kd;
    }headingParams;

    struct base_param
    {
        float kp;
        float ki;
        float kd;
    } baseParams;

    struct error
    {
        float proportional;
        float integral;
        float derivative;
        float previous;
    } err;

    struct velocity
    {
        float linear;
        float angular;
        float angular_filter;
    } vel;

public:
    void setBaseParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };


    void setHeadingParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };

    float control_base(float error, float speed, int condition, float deltaT)
    {
        if (condition)
        {
            err.proportional = error;
            if (err.proportional > 180)
            {
                err.proportional -= 360;
            }
            else if (err.proportional < -180)
            {
                err.proportional += 360;
            }
        }
        else
        {
            err.proportional = error;
        }
        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous) / deltaT;
        
        err.previous = err.proportional;
        
        float u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;
        float uT = headingParams.kp * err.proportional + headingParams.ki * err.integral + headingParams.kd * err.derivative;
        return condition ? fmax(-speed,fmin(speed,uT)) : fmax(-speed,fmin(speed,u));
    }
    float control_base(float error, float speed, int condition)
    {
        if (condition)
        {
            err.proportional = error;
            if (err.proportional > 180)
            {
                err.proportional -= 360;
            }
            else if (err.proportional < -180)
            {
                err.proportional += 360;
            }
        }
        else
        {
            err.proportional = error;
        }
        err.integral += err.proportional;

        err.derivative = (err.proportional - err.previous);
        
        err.previous = err.proportional;
        
        float u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;
        float uT = headingParams.kp * err.proportional + headingParams.ki * err.integral + headingParams.kd * err.derivative;
        return condition ? fmax(-speed,fmin(speed,uT)) : fmax(-speed,fmin(speed,u));
    }
};
