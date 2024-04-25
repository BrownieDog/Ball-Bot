#define ROLL 0
#define PITCH 1
#define YAW 2

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2


struct Euler {
    float roll;
    float pitch;
    float yaw;
};

class QuickSilver {
public:
    void update_estimate(float acc[], float gyro[], float dt);
    float vertical_acceleration_from_acc(float acc[]);
    void initialize(float b);
    float* getGravityVector();
    Euler getEuler();
private:
    float gravity_vector[3];
    float beta; // filter gain used to fuse acc into the estimate should be a fairly small value
    Euler eulerAngles;
};