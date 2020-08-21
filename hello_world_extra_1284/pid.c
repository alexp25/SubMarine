#include <stdlib.h>
#include <string.h>
#include "pid.h"

void initialize_pid_contex(struct pid_context* context, float dt, float target)
{
    context->dt = dt;
    context->yaw_target = target;
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
// [AP] poate fi redenumita eventual pt ca e doar pt carma, nu e chiar generic in forma actuala (atunci poti ignora comentariile de mai jos, mai putin primul)
int update_pid(struct pid_context * context, float yaw){
    char dir = 0;
    float st,dr;
    st = context->yaw_target - yaw;

    // [AP] nu neaparat, comanda poate fi != 0 oricum pt ca mai e integrala si derivata
    if( st == 0) return 0;

    // [AP] e putin ambiguu aici, ca ai combinat partea de busola cu pid-ul
    // functia asta ar trb sa fie mai generica, nu doar pt carma dupa busola

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

    // [AP] MAX_ACC e specific pt carma, poate fi ca parametru (in context)
    if( context->acc > MAX_ACC)
        context->acc =  MAX_ACC;
    if( context->acc < -MAX_ACC)
        context->acc = MAX_ACC;

    // [AP] alpha dat ca parametru, la fel ca si kp, ki, kd (in context)
    context->der = 0.95f * context->der + 0.05f * context->epsilon / context->dt;

    // [AP] MAX_STEER e specific pt carma, poate fi ca parametru (in context)
    float steering_value =   context->kp * context->epsilon + context->ki * context->acc + context->kd * context->der;
    if(steering_value > MAX_STEER)
        steering_value = MAX_STEER;
    if( steering_value < -MAX_STEER)
        steering_value = MAX_STEER;

    return dir * steering_value;
}