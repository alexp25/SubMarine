#define MAX_ACC 180
#define MAX_ANGLE 45
#define STANGA 1
#define DREAPTA -1
#define WRAP_AROUND(x) (x < 0) ? x + 360 : x

extern float kp, ki, kd;

struct pid_context {
    float epsilon;
    float acc;
    float der;
    float kp,ki,kd;
    float yaw_target;
    int dt;
};


//create a new context
struct pid_context* create_pid_contex(int dt);

//changes the values for the parameters kp,ki and kd
void load_weights(struct pid_context * context,float new_kp, float new_ki, float new_kd);

//changes the target direction for a pid context
void change_target(struct pid_context * context, float new_target);

//update a context
int update(struct pid_context * context, float yaw);