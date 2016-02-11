#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <valarray> // for faster and simple vector math
#include <cmath> // atan calculation (seems valarray already includes it.)
#include <thread> // Multi threading
#include <unistd.h> // linux read and write
#include <fcntl.h> // linux file input, output
#include <sys/types.h>
#include <string.h>
#include <signal.h>
#include "ArmourDetect.h"
#include <termios.h>

using namespace std;
using namespace cv;

struct termios config;

volatile sig_atomic_t flag = 0;
void myFunc(int sig)
{
	flag = 1;
}


bool UsingCam = true;
const int filterSize=1;
float convYaw[filterSize] = {}; //initialize filter array
float convPitch[filterSize] = {}; //initialize filter array
int n=0;
float yaw=0, pitch=0;
float last_yaw, last_pitch;
const uint8_t HEADER = 0xCE; //write
uint8_t CMD; //send move=1; send gimble=2; set by the condition; write;
uint8_t DATA_LENGTH; //the sent data's length; write

uint8_t ReachEnd;//0, 1; read;
uint8_t Hit = 0; //front(face to huangdi) 1; back(face to jidi) 2; not hit 0; read;
uint8_t Segment; //1,2,3 from left to right in the view from jidiqu; read;
bool YawReach;//reach 1; read
bool PitchReach;// read

bool AutoMove = false;//set by ArmourDet.cpp. If not detected armour, set to 1; otherwise, set to 0;
bool AutoRotate = false;//set by ArmourDet.cpp
bool IsFound;
uint8_t Move = 0; //1:left_slow; 2:left_fast; 3:right_slow; 4:right_fast; 0:stop; write;
float Yaw = 0;//-150 to 120 (0 to 60 for temprary)
float Pitch = -30;//-30 to 0
float deltaY = 0.07; // to be changed with the frequency
float deltaP = 0.02; // to be changed with the frequency



void Send_HEADER(uint8_t CMD, int fd);
void float2bytes(float fl,uint8_t* byteArray);
void receive(int fd);
void SerialSetup(int fd);

typedef enum
{
	RX_SOF,
	REACH_END,
	HP_LOW,
	HP_HIGH,
	ARMOR,
}Rx_State_e;

typedef struct
{
	uint8_t ReachEnd;
	uint16_t RemainHP;
	uint8_t Armor;
}Rx_Struct_t;

Rx_State_e rx_state;
Rx_Struct_t rx_struct;

bool update(float yaw, float pitch) {
	return (fabs(yaw-last_yaw)>0.8 || fabs(pitch-last_pitch)>0.4);
}

int main()
{
	//OPEN THE PORT
	char strSerialPort[255];
	int fd;
	signal(SIGINT, myFunc);

	sprintf(strSerialPort, "/dev/ttySAC0");
	fd = open(strSerialPort, O_RDWR);

	if (fd == -1)
	{
		cout << "Error in opening the port.\n";
		return -1;
	}
	else
	{
		cout << "Port " << strSerialPort << " successfully opened!\n";
	}

	SerialSetup(fd);

	int CamSetup=CamInitial();
	if(CamSetup==-1){
		return 0;
	}
	time_t last_time_Mv;
	time_t last_time_Rt;
	time(&last_time_Mv);
	time(&last_time_Rt); //delay

	while (true)
	{
		last_yaw = yaw;
		last_pitch = pitch;
		time_t this_time;
		time(&this_time); //delay
		
		Detect();
		float fyaw= 0,fpitch = 0;
		if (update) {
			//filter yaw & pitch to smooth the input, fyaw stands for filtered yaw angle
			convPitch[n]=pitch;
			convYaw[n]=yaw;
			n=(n+1)%filterSize;
			for (int i=0; i<filterSize; i++) {
				fyaw += convYaw[i];
				fpitch += convPitch[i];
			}
			fyaw /= filterSize;
			fpitch /= filterSize;
			uint8_t yawB[4];
			uint8_t pitchB[4];
			float2bytes(fyaw, yawB);
			float2bytes(fpitch, pitchB);
			cout << "fyaw:"<<fyaw<<"; fpitch:" << fpitch << endl;
			uint8_t data[8];
			for (int i=0; i<4; i++)
			{
				data[i]=yawB[i];
				data[i+4]=pitchB[i];
			}
			CMD=2;
			Send_HEADER(CMD, fd);
			int Write = write(fd, data, 8);
			cout << "	write rt byte" << Write << endl;
		}

		if(IsFound==1 && (fabs(yaw)<=1 && fabs(pitch) <=0.5))
		{
			uint8_t data[] = {1};
			CMD = 3;
			Send_HEADER(CMD, fd);
			int Write = write(fd, data, 1);
			cout << "	write shoot byte" << Write << endl;			
		}
		else {
			uint8_t data[] = {0};
			CMD = 3;
			Send_HEADER(CMD, fd);
			int Write = write(fd, data, 1);
			cout << "	write shoot byte" << Write << endl;
		}

		if (AutoMove)
		{
			Move = (Move == 0 ? 1 : Move);//start the robot

			if (this_time - last_time_Mv >= 1.5)
			{ //del
				if (Move == 1 || Move == 2)
				{
					Move += 2;
				}
				else if (Move == 3 || Move == 4)
				{
					Move -= 2;
				}

				time(&last_time_Mv); //delay
			}

			
			if (Hit != 0 && Move < 2)
			{
				uint8_t seg = Segment;
				Move = (Move + 2) % 4;
				//move for one seg
//				while(Segment==seg) {;}
			}

			uint8_t data[]={Move};
			CMD=1;
			Send_HEADER(CMD, fd);
			int Write = write(fd, data, 1);
			cout << "	write mv byte" << Write << endl;
		}
		else
		{
			Move = 0;
		}


		if (AutoRotate)
		{
			Yaw += deltaY;
			Pitch += deltaP;

			if ( (Yaw >= 60 && deltaY > 0) || (Yaw <= 0 && deltaY < 0) )
			{
				deltaY = -deltaY;
			}
			
			if ( (Pitch >= 0 && deltaP > 0) || (Pitch <= -30 && deltaP < 0) )
			{
				deltaP = -deltaP;
			}

			uint8_t yawB[4];
			uint8_t pitchB[4];
			float2bytes(Yaw, yawB);
			float2bytes(Pitch, pitchB);
			uint8_t data[8];

			for (int i=0; i<4; i++)
			{
				data[i]=yawB[i];
				data[i+4]=pitchB[i];
			}
			CMD=2;
			Send_HEADER(CMD, fd);
			int Write = write(fd, data, 8);
			cout << "	write rt byte" << Write << endl;
		}

//		receive(fd);

//		cout << Move << " " << Yaw << " " << Pitch << endl;

		if(flag)
		{
			cout << "stop." << endl;
			flag = 0;
			break;
		}
	}

//	DetectThread.join();
	close(fd);
	cout << "Terminated.\n";
	return 0;
}

void Send_HEADER(uint8_t cmd, int fd)
{
	if (cmd==1)
	{ //move sig
		DATA_LENGTH = 1;
	}
	else if (cmd == 2)
	{ //gimble sig
		DATA_LENGTH = 8;
	}
	else if (cmd == 3)
	{ //shoot sig
		DATA_LENGTH = 1;
	}

	uint8_t SendHeader[] = {HEADER, DATA_LENGTH, cmd};
	int Send = write(fd, SendHeader, 3);
}

void float2bytes(float fl,uint8_t* byteArray)
{
	union converter
	{
		float f;
		uint8_t bytes[4];
	}cv;
	cv.f = fl;

	for(int i = 0; i < 4; i++)
	{
		byteArray[i] = cv.bytes[i];
	}
}

void receive(int fd)
{
	uint8_t rx_buffer[100];
	int bytesRead = read(fd, rx_buffer, 100);
	for(int i = 0; i < bytesRead; i++)
	{
		uint8_t inByte = rx_buffer[i];
		switch(rx_state)
		{
			case RX_SOF:
			{
				if(inByte == 0xCE) rx_state = REACH_END;
			}break;
			case REACH_END:
			{
				if(inByte > 1 || inByte < -1)
				{
					rx_state = RX_SOF;
				}
				else
				{	
					rx_struct.ReachEnd = inByte;
					rx_state = HP_LOW;
				}
			}break;
			case HP_LOW:
			{
				rx_struct.RemainHP = inByte;
				rx_state = HP_HIGH;
			}break;
			case HP_HIGH:
			{
				rx_struct.RemainHP |= (inByte << 8);
				rx_state = ARMOR;
			}break;
			case ARMOR:
			{
				cout << "HP " << rx_struct.RemainHP << endl;
				rx_struct.Armor = inByte;
				rx_state = RX_SOF;
			}break;
			default:
			{
				rx_state = RX_SOF;
			}
		}
		cout << "rx_state "<< rx_state << endl;
		cout << "inByte "<< inByte << endl;
	}
}

void SerialSetup(int fd) {
	if(!isatty(fd)) { cout <<"... error handling ..."<<endl; }

 //
 // Get the current configuration of the serial interface
 //
 if(tcgetattr(fd, &config) < 0) { cout <<"... error handling ..."<<endl; }

 //
 // Input flags - Turn off input processing
 //
 // convert break to null byte, no CR to NL translation,
 // no NL to CR translation, don't mark parity errors or breaks
 // no input parity check, don't strip high bit off,
 // no XON/XOFF software flow control
 //
 config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

 //
 // Output flags - Turn off output processing
 //
 // no CR to NL translation, no NL to CR-NL translation,
 // no NL to CR translation, no column 0 CR suppression,
 // no Ctrl-D suppression, no fill characters, no case mapping,
 // no local output processing
 //
 // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
 //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
 config.c_oflag = 0;

 //
 // No line processing
 //
 // echo off, echo newline off, canonical mode off, 
 // extended input processing off, signal chars off
 //
 config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

 //
 // Turn off character processing
 //
 // clear current char size mask, no parity checking,
 // no output processing, force 8 bit input
 //
 config.c_cflag &= ~(CSIZE | PARENB);
 config.c_cflag |= CS8;

 //
 // One input byte is enough to return from read()
 // Inter-character timer off
 //
 config.c_cc[VMIN]  = 1;
 config.c_cc[VTIME] = 0;

 //
 // Communication speed (simple version, using the predefined
 // constants)
 //
 if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
     cout <<"... error handling ..."<<endl;
 }

 //
 // Finally, apply the configuration
 //
 if(tcsetattr(fd, TCSAFLUSH, &config) < 0) { cout <<"... error handling ..."<<endl; }
	cout << "serial set up completed" << endl;

}
/* //can be del
/*		Send_HEADER(CMD, fd);
		if (CMD == 1) {
			int Write = write(fd, MOVE, 1);
		}
		else if (CMD == 2){
			uint8_t yawB[4];
			float2bytes(float Yaw, uint8* yawB);
			uint8_t pitchB[4];
			float2bytes(float Pitch, uint8* pitchB);
			uint8_t data[] = {yawB, pitchB, Shoot};
			int Write = write(fd, data, 9);
		}
*/
	
	
