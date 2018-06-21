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

uint8_t MOVE = 4; //0:right_slow; 1:left_slow; 2:right_fast; 3:left_fast; 4:stop; read;
uint8_t ReachEnd;//0, 1; read;
uint8_t Hit = 0; //front(face to huangdi) 1; back(face to jidi) 2; not hit 0; read;
uint8_t AutoMove = 1;//set by ArmourDet.cpp. If not detected armour, set to 1; otherwise, set to 0;
uint8_t Segment; //1,2,3 from left to right in the view from jidiqu
double Yaw = 10;//relative angle
double Pitch = 10;
bool YawReach;//reach 1
bool PitchReach;
bool AutoRotate = true;//set by ArmourDet.cpp

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


	time_t last_time; //del
	time(&last_time); //del
	while (true) {
		time_t this_time; //del
		time(&this_time); //del
		if (AutoMove) {
//			if (ReachEnd) {
			if (MOVE==4) {
				MOVE=0;
			}
			if (this_time - last_time>=2) { //del
				if (MOVE < 2) {
					MOVE = (MOVE+1)%2;
				}
				else if (MOVE>=2&&MOVE<4) {
					MOVE = (MOVE+1)%2+2;
				}
//				while (ReachEnd) {;}
				time(&last_time); //del
			}
			
			if (Hit!=0 && MOVE<2) {
				uint8_t seg=Segment;
				MOVE = (MOVE+2)%4;
				//move for one seg
//				while(Segment==seg) {;}
			}
		}
		else {
			MOVE = 4;
		}

		if (AutoRotate) {
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
		}
		else {
			Yaw = 0;
			Pitch = 0;
		}
//		cout << "MOVE"<< unsigned(MOVE) <<endl;
		uint8_t SendSig[] = {MOVE};
//		cout << "write"<<endl;
		int NumWrite = write(fd, SendSig, 1);
//		cout << NumWrite <<endl;
		uint8_t read_buffer[1];
		int bytes_read = read(fd,&read_buffer,1);
//		cout << bytes_read << " " << read_buffer << endl;
		for (int i=0; i<bytes_read;i++){
			cout << "Converted content; " << unsigned(read_buffer[i]) << endl;
		}
//		cout << fd << endl;
	}
	close(fd);
}
