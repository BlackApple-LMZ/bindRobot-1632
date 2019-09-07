#ifndef CONTROLBEANEX_H
#define CONTROLBEANEX_H


class ControlBeanEx
{
private:
    void* bean;
    float private_V[8];

public:

    float x, y, z, goal_angle1, goal_angle2, rotation;
    bool communicate_success, initial_finish, servo_off_flag, move_flag;
    float angle1_after_judge, angle2_after_judge;
    bool isReach_after_judge;
    float real[6];

    ControlBeanEx(void *b);

    ~ControlBeanEx();


    int initial(int generation, float z_travel);

    void get_scara_param();

    void set_arm_length(float l1, float l2);

    int unlock_position();

    bool is_connected();

    int get_joint_state(int joint_num);

    bool set_drag_teach(bool state);

    bool get_drag_teach();

    bool set_cooperation_fun_state(bool state);

    bool get_cooperation_fun_state();

    bool is_collision();

    int set_position_move(float goal_x, float goal_y, float goal_z, float rotation, float speed,    float acceleration, int interpolation, int move_mode);

    int set_angle_move(float angle1, float angle2, float z, float rotation, float speed);

    int xyz_move(int direction, float distance, float speed);

    int single_joint_move(int axis, float distance, float speed);

    int trail_move(int point_number, float* x, float* y, float* z, float* r, float speed);

    void stop_move();

    bool is_robot_goto_target();

    void set_allow_distance_at_target_position(float x_distance, float y_distance, float z_distance, float r_distance);

    void set_catch_or_release_accuracy(float accuracy);

    bool judge_in_range(float x, float y, float z, float ratation);

    bool judge_position_gesture(float x, float y);

    int joint_home(int joint_num);

    int movel_xyz(float goal_x, float goal_y, float goal_z, float rotation, float speed);

    int movej_xyz(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float rough);

    int movej_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float roughly, int lr);

    int movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);

    int change_attitude(float speed);

    bool wait_stop();

    int pause_move();

    int resume_move();

    bool set_digital_out(int io_number, bool value);

    int get_digital_out(int io_out_num);

    int get_digital_in(int io_in_number);
    \
    int set_efg_state(int type, float distance);

    int get_efg_state(int* type, float* distance);

    int movej_j5(float j5_pos, float speed);

    float get_j5_positon(int type);




    int get_robot_id();

};

#endif // CONTROLBEANEX_H
