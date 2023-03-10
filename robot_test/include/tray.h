#pragma once // ?

typedef unsigned char 	BYTE;
class Ctray
{

public:
	
    Ctray(int portnum);
	virtual ~Ctray(); //	

	void Open_port();
    void Reset();
    void BPS_Set();
    void Control_on();
    void Control_off();
    void Nom_Vel_Set();
    void Vel_Dir_Set();
    void Vel_Ctrl_Set();
    void Resol_Set();

    void Ctrl_Set_Read();
    void Pos_Ctrl_Read();
    void Vel_Ctrl_Read();
    void Resol_Read();

    void Velocity_Command(int mode); //mode 0 : stop, 1 : move direcction 1, 2 : move deirection 2
    void Sensor_Based(int mode); //mode 0 : stop, 1 : move direcction 1, 2 : move deirection 2


    unsigned char _Setting[10];
    unsigned char _Check_Setting[10];
    unsigned char _Check_Setting2[11];
    unsigned char _Check_Setting3[12];

    int tray_status;
    float error_code_tray;

private:
    unsigned char _bps_Set[7];
    unsigned char _Nom_Vel_Set[8];
    unsigned char _Vel_Ctrl_Set[10];
    unsigned char _set_dir[7];
    unsigned char _vel_cmd_dir0[10];
    unsigned char _vel_cmd_dir1[10];
    unsigned char _vel_cmd_zero[10];
    unsigned char _Resol_Set[8];
    unsigned char _Reset[6];
    unsigned char _req_Feedback[6];
    unsigned char _pos_Feedback[6];
    unsigned char _resol_Feedback[6];
    unsigned char _vel_Feedback[6];
    unsigned char _con_on[7];
    unsigned char _con_off[7]; 

    unsigned char _buf[255];
    unsigned char _dummy_q;

    // unsigned char _Setting[10];
    // unsigned char _Check_Setting[10];
    // unsigned char _Check_Setting2[11];
    // unsigned char _Check_Setting3[12];

    int _port;
    int _port_address;

};