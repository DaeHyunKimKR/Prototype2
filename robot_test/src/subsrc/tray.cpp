#include <tray.h>

 // C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <cmath>
#include <stdlib.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function0
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/select.h>

typedef unsigned int 	WORD;
typedef unsigned char   BYTE;

Ctray::Ctray(int portnum)
{
  _bps_Set[0] = 0xFF;
  _bps_Set[1] = 0xFE;
  _bps_Set[2] = 0x00;
  _bps_Set[3] = 0x03;
  _bps_Set[5] = 0x07;
  _bps_Set[6] = 0x0D;
  _bps_Set[4] = 0xE8;

  _Nom_Vel_Set[0] = 0xFF;
  _Nom_Vel_Set[1] = 0xFE;
  _Nom_Vel_Set[2] = 0x00;
  _Nom_Vel_Set[3] = 0x04;
  _Nom_Vel_Set[4] = 0x43; // check
  _Nom_Vel_Set[5] = 0x09;
  _Nom_Vel_Set[6] = 0x19;
  _Nom_Vel_Set[7] = 0x96;

  _Vel_Ctrl_Set[0] = 0xFF;
  _Vel_Ctrl_Set[1] = 0xFE;
  _Vel_Ctrl_Set[2] = 0x00;
  _Vel_Ctrl_Set[3] = 0x06;
  _Vel_Ctrl_Set[4] = 0x96;//
  _Vel_Ctrl_Set[5] = 0x05;
  _Vel_Ctrl_Set[6] = 0xFE;
  _Vel_Ctrl_Set[7] = 0xFE;
  _Vel_Ctrl_Set[8] = 0x4E;
  _Vel_Ctrl_Set[9] = 0x04;

  _set_dir[0] = 0xFF;
  _set_dir[1] = 0xFE;
  _set_dir[2] = 0x00;
  _set_dir[3] = 0x03;
  _set_dir[4] = 0xED;//
  _set_dir[5] = 0x0E;
  _set_dir[6] = 0x01;
  
  // _vel_cmd_dir0[0] = 0xFF;
  // _vel_cmd_dir0[1] = 0xFE;
  // _vel_cmd_dir0[2] = 0x00;
  // _vel_cmd_dir0[3] = 0x06;
  // _vel_cmd_dir0[4] = 0xF1;//0x01; //check sum
  // _vel_cmd_dir0[5] = 0x03;
  // _vel_cmd_dir0[6] = 0x00;//0x01; // 방향
  // _vel_cmd_dir0[7] = 0xF6;//0x02; // 속도1
  // _vel_cmd_dir0[8] = 0x05; // 속도2
  // _vel_cmd_dir0[9] = 0x0A;//0x0A; // 도달 time(0.1s 단위)

  _vel_cmd_dir0[0] = 0xFF;
  _vel_cmd_dir0[1] = 0xFE;
  _vel_cmd_dir0[2] = 0x05;
  _vel_cmd_dir0[3] = 0x06;
  _vel_cmd_dir0[4] = 0xEC;
  _vel_cmd_dir0[5] = 0x03;
  _vel_cmd_dir0[6] = 0x00;
  _vel_cmd_dir0[7] = 0xF6;
  _vel_cmd_dir0[8] = 0x05;
  _vel_cmd_dir0[9] = 0x0A;

  _vel_cmd_dir1[0] = 0xFF;
  _vel_cmd_dir1[1] = 0xFE;
  _vel_cmd_dir1[2] = 0x00;
  _vel_cmd_dir1[3] = 0x06;
  _vel_cmd_dir1[4] = 0xF0;//0x01; //check sum
  _vel_cmd_dir1[5] = 0x03;
  _vel_cmd_dir1[6] = 0x01;//0x01; // 방향
  _vel_cmd_dir1[7] = 0xF6;//0x02; // 속도1
  _vel_cmd_dir1[8] = 0x05; // 속도2
  _vel_cmd_dir1[9] = 0x0A;//0x0A; // 도달 time(0.1s 단위)

  _vel_cmd_zero[0] = 0xFF;
  _vel_cmd_zero[1] = 0xFE;
  _vel_cmd_zero[2] = 0x00;
  _vel_cmd_zero[3] = 0x06;
  _vel_cmd_zero[4] = 0xE2;//0x01; //check sum
  _vel_cmd_zero[5] = 0x03;
  _vel_cmd_zero[6] = 0x00;//0x01; // 방향
  _vel_cmd_zero[7] = 0x00;//0x02; // 속도1
  _vel_cmd_zero[8] = 0x00; // 속도2
  _vel_cmd_zero[9] = 0x14;//0x0A; // 도달 time(0.1s 단위)

  _Resol_Set[0] = 0xFF;
  _Resol_Set[1] = 0xFE;
  _Resol_Set[2] = 0x00;
  _Resol_Set[3] = 0x04;
  _Resol_Set[4] = 0xEF;
  _Resol_Set[5] = 0x0A;
  _Resol_Set[6] = 0x02;
  _Resol_Set[7] = 0x00;

  _Reset[0] = 0xFF;
  _Reset[1] = 0xFE;
  _Reset[2] = 0xFF;
  _Reset[3] = 0x02;
  _Reset[4] = 0xF1;
  _Reset[5] = 0x0D;

  _req_Feedback[0] = 0xFF;
  _req_Feedback[1] = 0xFE;
  _req_Feedback[2] = 0x00;
  _req_Feedback[3] = 0x02;
  _req_Feedback[5] = 0xA4;
  _req_Feedback[4] = ~(_req_Feedback[2]+_req_Feedback[3]+_req_Feedback[5]);

  _pos_Feedback[0] = 0xFF;
  _pos_Feedback[1] = 0xFE;
  _pos_Feedback[2] = 0x00;
  _pos_Feedback[3] = 0x02;
  _pos_Feedback[4] = 0x5C;//
  _pos_Feedback[5] = 0xA1;

  _vel_Feedback[0] = 0xFF;
  _vel_Feedback[1] = 0xFE;
  _vel_Feedback[2] = 0x00;
  _vel_Feedback[3] = 0x02;
  _vel_Feedback[4] = 0x5B;
  _vel_Feedback[5] = 0xA2;

  _resol_Feedback[0] = 0xFF;
  _resol_Feedback[1] = 0xFE;
  _resol_Feedback[2] = 0x00;
  _resol_Feedback[3] = 0x02;
  _resol_Feedback[4] = 0x56;
  _resol_Feedback[5] = 0xA7;

  _con_off[0] = 0xFF;
  _con_off[1] = 0xFE;
  _con_off[2] = 0x00;
  _con_off[3] = 0x03;
  _con_off[4] = 0xEF;
  _con_off[5] = 0x0C;
  _con_off[6] = 0x01;

  _con_on[0] = 0xFF;
  _con_on[1] = 0xFE;
  _con_on[2] = 0x00;
  _con_on[3] = 0x03;
  _con_on[4] = 0xF0;
  _con_on[5] = 0x0C;
  _con_on[6] = 0x00;

  unsigned char sensor[8];

  _dummy_q = 'q';
  _port = 0;
  _port_address = portnum;
}

Ctray::~Ctray()
{

}

void Ctray::Reset()
{
  write(_port, _Reset,sizeof(_Reset)); 
  //std::cout << "Reset Complete." <<std::endl;
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
}
void Ctray::BPS_Set()
{
    write(_port, _bps_Set,sizeof(_bps_Set)); // bps 설정
  //std::cout << "Baudrate Setting: 115200bps complete." <<std::endl;
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);
}

void Ctray::Control_on()
{
  write(_port, _con_on,sizeof(_con_on));
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);
  //tcflush(_port,  TCIOFLUSH);
}

void Ctray::Control_off()
{
  write(_port, _con_off,sizeof(_con_off));
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  // std::cout << "방향 = 0 " << std::endl;
  //usleep(2000);
}

void Ctray::Nom_Vel_Set()
{
  write(_port, _Nom_Vel_Set,sizeof(_Nom_Vel_Set)); // 정격속도 설정
  //std::cout << "Nominal Velocity Setting complete." <<std::endl;
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);
}

void Ctray::Vel_Dir_Set()
{
  write(_port, _set_dir,sizeof(_set_dir)); // 제어방향 설정
  //std::cout << "Motor Direction setting complete." <<std::endl;
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);
}

void Ctray::Vel_Ctrl_Set()
{
  write(_port, _Vel_Ctrl_Set,sizeof(_Vel_Ctrl_Set)); // 속도 제어기 설정
  //std::cout << "Velocity Controller Setting complete." <<std::endl;
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);
}

void Ctray::Resol_Set()
{
  write(_port, _Resol_Set,sizeof(_Resol_Set)); // 분해능 설정
  //std::cout << "Resolution Setting Complete." <<std::endl;
  usleep(20000);
  //tcflush(_port,  TCIOFLUSH);
  //usleep(2000);
}

void Ctray::Open_port()
{
   // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  //_port = open("/dev/ttyUSB3", O_RDWR|O_NOCTTY);
  _port = open("/dev/TRAY2", O_RDWR|O_NOCTTY);
  
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  
  bzero(&tty, sizeof(tty)); 
  // Read in existing settings, and handle any error
  if(tcgetattr(_port, &tty) != 0) 
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    error_code_tray = 1401;
  }

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  //tty.c_iflag = INPCK;
  //tty.c_oflag = 0;
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_lflag = 0;
  bzero(tty.c_cc, NCCS);
  tty.c_cc[VTIME] = 0; 
  tty.c_cc[VMIN] = 1;  

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CS8;
  tty.c_lflag &= ~ECHO;  
  tty.c_oflag = 0;
  //tty.c_lflag  =  ICANON;
  
  tcsetattr(_port,  TCSANOW,  &tty);
  tcflush(_port,  TCIOFLUSH);//
  // Save tty settings, also checking for error
  //if (tcsetattr(_port, TCSANOW, &tty) != 0) 
  //{
  //  printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  //}
  
  //write(_port, &_dummy_q,sizeof(_dummy_q));
  //usleep(20000);
  //uint8_t temp[8];
  //read(_port, &temp, sizeof(temp));
  //tcflush(_port,  TCIOFLUSH);
  
  write(_port, &_dummy_q,sizeof(_dummy_q));
  usleep(20000);
  unsigned char temp[8];
  read(_port, &temp, sizeof(temp));
  //std::cout <<  temp[1] << temp[2]  <<std::endl;

  Control_on();
  Vel_Dir_Set();
  Vel_Ctrl_Set();
  Resol_Set();
  //Control_on();
}

void Ctray::Ctrl_Set_Read()
{
  write(_port, _req_Feedback,sizeof(_req_Feedback)); //  설정
  std::cout << "Controller setup feedback is requested" <<std::endl;
  usleep(20000);
  tcflush(_port,  TCOFLUSH);
  usleep(2000);

  read(_port, _Check_Setting,sizeof(_Check_Setting));
  usleep(20000);
  tcflush(_port,  TCIOFLUSH);
  usleep(20000);
  std::cout << (int)_Check_Setting[0] <<std::endl;
  std::cout << (int)_Check_Setting[1] <<std::endl;
  std::cout << (int)_Check_Setting[2] <<std::endl;
  std::cout << (int)_Check_Setting[3] <<std::endl;
  std::cout << (int)_Check_Setting[4] <<std::endl;
  std::cout << (int)_Check_Setting[5] <<std::endl;
  std::cout << (int)_Check_Setting[6] <<std::endl;
  std::cout << (int)_Check_Setting[7] <<std::endl;
  std::cout << (int)_Check_Setting[8] <<std::endl;
  std::cout << (int)_Check_Setting[9] <<std::endl;
  std::cout << "test" <<std::endl;

}
void Ctray::Pos_Ctrl_Read()
{
    
  write(_port, _pos_Feedback,sizeof(_pos_Feedback));
  std::cout << "Controller pos feedback is requested" <<std::endl;
  usleep(20000);
  tcflush(_port,  TCOFLUSH);
  usleep(2000);
    read(_port, _Check_Setting3,sizeof(_Check_Setting3));
    usleep(20000);
    tcflush(_port,  TCIOFLUSH);
    usleep(20000);
    std::cout << "방향"<<(int)_Check_Setting3[6] <<std::endl;
    std::cout << "위치1"<<(int)_Check_Setting3[7] <<std::endl;
    std::cout << "위치2"<<(int)_Check_Setting3[8] <<std::endl;
    std::cout << "속도1"<< (int)_Check_Setting3[9] <<std::endl;
    std::cout << "속도2"<< (int)_Check_Setting3[10] <<std::endl;
    std::cout << "전류"<< (int)_Check_Setting3[11] <<std::endl;
    std::cout << "testtttttttt" <<std::endl;
}
void Ctray::Vel_Ctrl_Read()
{
  write(_port, _vel_Feedback,sizeof(_vel_Feedback));
  std::cout << "Controller vel feedback is requested" <<std::endl;
  usleep(20000);
  read(_port, _Check_Setting2,sizeof(_Check_Setting2));
  usleep(20000);
  std::cout << "방향: "<<(int)_Check_Setting2[6] <<std::endl;
  std::cout << "속도1: "<<(int)_Check_Setting2[7] <<std::endl;
  std::cout << "속도2: "<<(int)_Check_Setting2[8] <<std::endl;
  std::cout << "위치1: "<<(int)_Check_Setting2[9] <<std::endl;
  std::cout << "위치2: "<<(int)_Check_Setting2[10] <<std::endl;
}
void Ctray::Resol_Read()
{
  write(_port, _resol_Feedback,sizeof(_resol_Feedback));
  std::cout << "Controller vel feedback is requested" <<std::endl;
  usleep(20000);
  tcflush(_port,  TCOFLUSH);
  usleep(2000);

  read(_port, _Check_Setting2,sizeof(_Check_Setting2));
  usleep(20000);
  tcflush(_port,  TCIOFLUSH);
  usleep(20000);
  std::cout << "분해능1: "<<(int)_Check_Setting2[6] <<std::endl;
  std::cout << "분해능2: "<<(int)_Check_Setting2[7] <<std::endl;
  std::cout << "속도2: "<<(int)_Check_Setting2[8] <<std::endl;
  std::cout << "위치1: "<<(int)_Check_Setting2[9] <<std::endl;
  std::cout << "위치2: "<<(int)_Check_Setting2[10] <<std::endl;
}

void Ctray::Velocity_Command(int mode)
{
    if(mode == 0)
    {
        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
        tray_status = 0;
    }
    else if(mode == 1)
    {
        write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_dir0));
        tray_status = 1;
    //    if(temp[0] == 0 && temp[1] == 0 && temp[2] == f && ) //%01fffff || temp == %10ffff)
    //    {
    //      write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_zero));
    //      if(temp == %00fffff)
    //      {
    //        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
    //      }
    //    }
    }
    else if(mode == 2)
    {
        write(_port, _vel_cmd_dir1,sizeof(_vel_cmd_dir1));
        tray_status = -1;
    //    if(temp == %01fffff || temp == %10ffff)
    //    {
    //      write(_port, _vel_cmd_dir1,sizeof(_vel_cmd_zero));
    //      if(temp == %00fffff)
    //      {
    //        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
    //      }
    //    }
    }
    else
    {
        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
        std::cout << "Wrong command - Tray!!!" << std::endl;
    }
    usleep(20000);
}


void Ctray::Sensor_Based(int mode)
{
    if(mode == 0)
    {
        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
        tray_status = 0;
    }
    else if(mode == 1)
    {
        write(_port, &_dummy_q,sizeof(_dummy_q));
        usleep(20000);
        unsigned char temp[8];
        read(_port, &temp, sizeof(temp));
        
        write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_dir0));
        tray_status = 1;

        static bool sensor_check = false;
        static bool is_on_tray = false;
        static bool is_traying = false;
        static int movement =0;

        if(temp[2]=='1'&&!is_on_tray) 
        {
          is_traying = true;        
        }
        if(temp[2]=='0'&&is_traying)
        {
          is_on_tray = true;
          is_traying = false;
        }

        if(is_on_tray){
          write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));

        }else{
          write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_dir0)); 
        }

    }
    else if(mode == 2)
    {
        write(_port, _vel_cmd_dir1,sizeof(_vel_cmd_dir1));
        tray_status = -1;
    }
    else
    {
        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
        std::cout << "Wrong command - Tray!!!" << std::endl;
    }
    usleep(20000);
}



// while(1)
// {
//   write(_port, &_dummy_q,sizeof(_dummy_q));
//   usleep(20000);
//   unsigned char temp[8];
//   read(_port, &temp, sizeof(temp));
//   std::cout <<  temp[1] << temp[2]  <<std::endl;
//   //Velocity_Command(1);
// }








/*

void Ctray::Sensor_Control()
{
  write(_port, &_dummy_q,sizeof(_dummy_q));
  // usleep(20000);
  unsigned char sensor[8];
  read(_port, &sensor, sizeof(sensor));
  
  if( sensor[1] == '0' && sensor[2] == '0' )
  {
    write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_dir0));
    std::cout <<  sensor[1] << sensor[2]  <<std::endl;
    // if(sensor[1] == 1)
    // {
    //   write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
    //   read(_port, &sensor, sizeof(sensor));
    //   std::cout <<  sensor[1]  <<std::endl;
    // }
  }
  else if( sensor[1] == '1' || sensor[2] == '1' )
  {
    write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
    std::cout <<  sensor[1] << sensor[2]  <<std::endl;
  }
  tcflush(_port,  TCIOFLUSH);
}

void Ctray::Velocity_Command(int mode)
{
    if(mode == 0)
    {
        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
        tray_status = 0;
    }
    else if(mode == 1)
    {
        write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_dir0));
        tray_status = 1;

        if( sensor[1] == 1 || sensor[2] == 1 ) //%01fffff || sensor == %10ffff)
        {
          write(_port, _vel_cmd_dir0,sizeof(_vel_cmd_zero));

          if( sensor[1] == 0 && sensor[2] == 0 )
          {
            write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
          }
        }

    }
    else if(mode == 2)
    {
        write(_port, _vel_cmd_dir1,sizeof(_vel_cmd_dir1));
        tray_status = -1;

  
        if( sensor[1] == 1 || sensor[2] == 1 ) //%01fffff || sensor == %10ffff)
        {
          write(_port, _vel_cmd_dir1,sizeof(_vel_cmd_dir1));

          if( sensor[1] == 0 && sensor[2] == 0 )
          {
            write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
          }
        }
    }
    else
    {
        write(_port, _vel_cmd_zero,sizeof(_vel_cmd_zero));
        std::cout << "Wrong command - Tray!!!" << std::endl;
    }
    usleep(20000);
}

*/