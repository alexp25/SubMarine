#include <stdlib.h>
#include <string.h>
#include "pid.h"

float kp = 0.5, ki = 0.5, kd = 0.5;

struct pid_context* create_pid_contex(int dt)
{
    struct pid_context* context = (struct pid_context *) calloc(1,sizeof(struct pid_context)); 
    context->dt = dt;
    return context;
}

void load_weights(struct pid_context * context,float new_kp, float new_ki, float new_kd)
{
    context->kp = new_kp;
    context->ki = new_ki;
    context->kd = new_kd;
}

void change_target(struct pid_context * context, float new_target)
{
    float dt = context->dt;
    memset(context,0,sizeof(struct pid_context));
    context->yaw_target = new_target;
    context->dt = dt;
}

//update a context
int update(struct pid_context * context, float yaw){
    char dir = 0;
    float st,dr;
    st = context->yaw_target - yaw;

    if( st == 0) return 0;

    if( st >= 0) {
        dr = 360 - st;
    } else {
        dr = st;
        st = 360 + st;
    }

    if( st > dr)
    {
        context->epsilon = dr; 
        dir = DREAPTA;
    }
    else
    {
        context->epsilon = st;
        dir = STANGA;
    }

    st = context->dt * context->epsilon * dir;
    context->acc += st;
    if( context->acc > MAX_ACC)
        context->acc =  MAX_ACC;
    if( context->acc < -MAX_ACC)
        context->acc = MAX_ACC;

    context->der = 0.95f * context->der + 0.05f * context->epsilon / context->dt;

    return dir * ( context->kp * context->epsilon + context->ki * context->acc + context->kd * context->der);
}