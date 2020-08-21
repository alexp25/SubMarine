#define MAX_ACC 180
#define MAX_STEER 100
#define MAX_ANGLE 45
#define STANGA 1
#define DREAPTA -1
#define WRAP_AROUND(x) (x < 0) ? x + 360 : x

#define UPDATE_PID_PARAM(x,y) x = ((float)y)/10000;

struct pid_context {
    float epsilon;
    float acc;
    float der;
    float kp,ki,kd;
    float yaw_target;
    float dt;
};

//create a new context
void initialize_pid_contex(struct pid_context* context, float dt, float target);

//changes the values for the parameters kp,ki and kd
void load_weights(struct pid_context * context,float new_kp, float new_ki, float new_kd);

//changes the target direction for a pid context
void change_target(struct pid_context * context, float new_target);

//update a context
int update_pid(struct pid_context * context, float yaw);