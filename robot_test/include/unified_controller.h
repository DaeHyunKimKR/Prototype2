#pragma once
#ifndef __CONTROLLER_H //?
#define __CONTROLLER_H //?
#include <iostream>
using namespace std;

class CScara
{
    public:
	CScara();
	virtual ~CScara();

    double _q[3]; //joint angle from encoder (deg)
    double _q_goal[3]; //goal joint angle (deg)
    double _x_ee_local[3]; //end-effector x,y-positions (m) and orientation (rad) in local coordinate // 0=x, 1=y, 2=yaw
    double _x_ee_goal_local[3]; //goal end-effector x,y-positions (m) and orientation (rad) in local coordinate // 0=x, 1=y, 2=yaw    
    bool   _bool_ee_control; //true: end-effector position command (IK control), false: joint angle command
    double _time_motion; //desired time for p-to-p motion
    double _link_length[3];
    bool   _rev;
    float error_code_ik;
    
    void calc_iversekinematics(double IK_x,double IK_y,double IK_a); //ee_cmd x,y,yaw angle (m, m, rad), jointangle_cmd: q1, q2, q3 (rad)    
    void read_state(double jointangle[]); //read robot state
    void write_cmd_from_FSM(double motion_time, double jointangle_cmd[], double endeffector_cmd[], bool bool_endeffector_ctrl); //read command
    void update_forward_kinematics();

    
    

    private:
    // void update_forward_kinematics();
    double range(double angle);
    // double front_pos_[3];
      

    double x, y, z, cos_q2, sin_q2_1, sin_q2_2, q1_1, q1_2, q2_1, q2_2, q3_1, q3_2, k1_1, k1_2, k2_1, k2_2, gamma_1, gamma_2; //variables for inverse kinematics
};

class CLinear
{
    public:
	CLinear();
	virtual ~CLinear();

    double _height; // position (m)
    double _height_goal; // goal position (m)
    double _time_motion; //desired time for p-to-p motion
    
    void read_state(double height); //read robot state
    void write_cmd_from_FSM(double motion_time, double height_cmd); //read command
};

class CTray
{
    public:
	CTray();
	virtual ~CTray();

    double _velocity_cmd; // -100 ~ 100    
    bool _bool_tray_cmd;
    
    //void read_state(double height); //read robot state
    void write_cmd_from_FSM(double _velocity_cmd, bool bool_tray_motion_cmd ); //read command
};

class CUController
{

public:
    CUController();
    virtual ~CUController(); //  

    CScara Scara_h;

    void Finite_State_Machine(int button, float axes1, float axes2, double time);
    // void read_state_deg(double jointangle[]);

    bool _mode, _mode2;
    int _goal_position[5];
    int _x_goal[5];
    bool _check_table[16];
    int _table_status[16];

    double IK_x, IK_y, IK_z, IK_a;
    // double _ee_pos[3];
    double front_pos_[3];
    // double _x_ee_local[3];

    ////////////////////////////////////////
	int _task_tray_cmd; // 0:ready 1:load the first object, 2: second object 3: third object 4: fourth object
	int _pre_tray_cmd;
	int _task_state; //for monitoring task state, 0: moving 1: loading complete, 2: done (ready) //TODO: 2 will become 0 after few (maybe 2~5 sec) seconds.
	double _target_position_from_vision[3]; //x,y,z
	////////////////////////////////////////

    double _operation_time, _init_time;
  

    // struct endeffector //TODO: modify
    // {
    //     double pos; //pos from encoder (??)
    //     double vel_des; //desired velocity (??)
    //     double time_motion; //desired time
    //     bool app_sensor[]; //measrued sensor data (on/off)
    // };

    // struct zmotor //TODO: modify
    // {
    //     double pos; //pos from encoder (m)
    //     double pos_goal; //desired position (m)
    //     double time_motion; //desired time
    //     bool break_cmd[]; //break command (on/off)
    // };

public:
    CScara Scara_forward;
    CScara Scara_backward;
    int _control_arm_num;

    CTray tray_forward;
    CTray tray_backward;

    CLinear Linear_forward;
    CLinear Linear_backward;
    
    //void read_EE();
    //void read_zmotor();

    //void compute();
    void finite_state_machine(int button, float axes1, float axes2, double time);
    void boxlane_initial_desired(int tray_num);
    void boxlane_table_desired(int tray_num);

    double _q_goal_forward[3];
    double _x_goal_forward[3];
    double _q_goal_backward[3];
    double _x_goal_backward[3];
    double _height_goal_linear;

    typedef enum
    {
        Test_Task,
        IK_Task,
        No_Task
    } Task;
    typedef enum{
        Ready,
        Front_Table_Lane,
        Back_Table_Lane,
        Front_Table,
        Back_Table,
        Item_Move,
        Drop_Point,
        Drop_Lane,
        IK_State
    } State;

    State _CurrentState;
    State _PreviousState;
    Task _CurrentTask;


private:

    double RANGE(double angle);
    double _length_link1 ;
    double _length_link2 ;
    double _length_link3 ;
    bool _rev;

    double _now_time4;
    int _box, _table;
    int _cnt;

    // typedef enum
    // {
    //     Test_Task,
    //     IK_Task,
    //     No_Task
    // } Task;
    // typedef enum{
    //     Ready,
    //     Front_Table_Lane,
    //     Back_Table_Lane,
    //     Front_Table,
    //     Back_Table,
    //     Item_Move,
    //     Drop_Point,
    //     Drop_Lane,
    //     IK_State
    // } State;

    // State _CurrentState;
    // State _PreviousState;
    // Task _CurrentTask;
    // double _init_time;
    
    int _x_ready[4];

    // double _q_goal_forward[3];
    // double _x_goal_forward[3];
    // double _q_goal_backward[3];
    // double _x_goal_backward[3];
    // double _height_goal_linear;
    bool _bool_ee_control;
    
    void Initialize();

    public:
    bool flag_Ready, flag_frontTL, flag_front, flag_ItemM;
    bool flag_DropP, flag_DropL;
    bool flag[6];


};




#endif

