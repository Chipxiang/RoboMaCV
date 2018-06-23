#include <time.h>
#include <stdio.h>
#include <iostream>
#include <valarray> // for faster and simple vector math
#include <cmath> // atan calculation (seems valarray already includes it.)
#include <thread> // Multi threading
#include <unistd.h> // linux read and write
#include <fcntl.h> // linux file input, output
#include <sys/types.h>
#include <string.h>

using namespace std;

const uint8_t HEADER = 0xCE; //write
uint8_t MOVE = 4; //0:right_slow; 1:left_slow; 2:right_fast; 3:left_fast; 4:stop; write;
uint8_t Shoot = 0; //set by ArmourDet.cpp; Shoot=1: shoot; Shoot=0: not shoot; write;
uint8_t ReachEnd;//0, 1; read;
uint8_t Hit = 0; //front(face to huangdi) 1; back(face to jidi) 2; not hit 0; read;
bool AutoMove = true;//set by ArmourDet.cpp. If not detected armour, set to 1; otherwise, set to 0;
uint8_t Segment; //1,2,3 from left to right in the view from jidiqu; read;
float Yaw = 0;//absolute angle; write
float Pitch = 0;// write
float deltaY = 0.06; // to be changed with the frequency
float deltaP = 0.01; // to be changed with the frequency
uint8_t CMD; //send move=1; send gimble=2; set by the condition; write;
uint8_t DATA_LENGTH; //the sent data's length; write
bool YawReach;//reach 1; read
bool PitchReach;// read
bool AutoRotate = true;//set by ArmourDet.cpp

void Send_HEADER(uint8_t CMD);
void float2bytes(float fl,uint8* byteArray);

int main() {
	char strSerialPort[255];
	int fd;
	sprintf(strSerialPort, "/dev/ttySAC0");
	fd = open(strSerialPort, O_RDWR);
	if (fd == -1) {
		cout << "Error in opening the port." << endl;
		return -1;
	}
	else {
		cout << "Port " << strSerialPort << " successfully opened. " << endl;
	}


	time_t last_time_Mv; //del
	time(&last_time_Mv); //del
	time_t last_time_Rt; //del
	time(&last_time_Rt); //del
	while (true) {
		time_t this_time; //del
		time(&this_time); //del
		if (AutoMove) {
//			if (ReachEnd) {
			if (MOVE==4) {
				MOVE=0;
			}
			if (this_time - last_time_Mv>=2) { //del
				if (MOVE < 2) {
					MOVE = (MOVE+1)%2;
				}
				else if (MOVE>=2&&MOVE<4) {
					MOVE = (MOVE+1)%2+2;
				}
//				while (ReachEnd) {;}
				time(&last_time_Mv); //del
			}
/*			
			if (Hit!=0 && MOVE<2) {
				uint8_t seg=Segment;
				MOVE = (MOVE+2)%4;
				//move for one seg
//				while(Segment==seg) {;}
			}
*/
			Send_HEADER(CMD);
			int Write = write(fd, MOVE, 1);
		}
		else { //this is almost impossible in our design...
			MOVE = 4;
		}

		if (AutoRotate) {
			Yaw = Yaw + deltaY;
			Pitch = Pitch + deltaP;
			if (this_time - last_time_Rt>=1.5) { //del
				deltaY = -deltaY;
				deltaP = -deltaP;
//				while (ReachEnd) {;}
				time(&last_time_Rt); //del
			}
			Send_HEADER(CMD);
			uint8_t yawB[4];
			float2bytes(float Yaw, uint8* yawB);
			uint8_t pitchB[4];
			float2bytes(float Pitch, uint8* pitchB);
			uint8_t data[] = {yawB, pitchB};
			int Write = write(fd, data, 8);
/*
			if (Yaw == 0) {
				Yaw = 10;//To be adjusted
				Pitch = 10;//To be adjusted
			}
			if (YawReach) {
				Yaw = -Yaw;
			}
			if (PitchReach) {
				Pitch = -Pitch;
			}
*/
		}

		if(Shoot==1) {
			Send_HEADER(CMD);
			int Write = write(fd, Shoot, 1);
		}
/* //can be del
		else {
			deltaY = 0;
			deltaP = 0;
		}
*/
//		cout << "MOVE"<< unsigned(MOVE) <<endl;
//		uint8_t SendSig[] = {HEADER};
//		int NumWrite = write(fd, SendSig, 1);
//		cout << NumWrite <<endl;
//		uint8_t read_buffer[1];
//		int bytes_read = read(fd,&read_buffer,1);
//		cout << bytes_read << " " << read_buffer << endl;
//		for (int i=0; i<bytes_read;i++){
//			cout << "Converted content; " << unsigned(read_buffer[i]) << endl;
//		}
//		cout << fd << endl;
/*		Send_HEADER(CMD);
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
	}
	close(fd);
}

void Send_HEADER(uint8_t cmd) {
	if (cmd==1) { //move sig
		DATA_LENGTH = 1;
	}
	else if (cmd == 2) { //gimble sig
		DATA_LENGTH = 9;
	}
	else if (cmd == 3) { //shoot sig
		DATA_LENGTH = 1;
	}
	uint8_t SendHeader[] = {HEADER, DATA_LENGTH, cmd};
	int Send = write(fd, SendHeader, 3);
}

void float2bytes(float fl,uint8* byteArray) {
	union converter {
		float f;
		uint8_t bytes[4];
	}cv;
	cv.f = fl;
	for(int i=0;i<4;i++) {
		byteArray[i] = cv.bytes[i];
	}
}
