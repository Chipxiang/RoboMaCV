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

struct point {
	float pit;
	float yaw;
};

bool UsingCam = true;

point convP[3] = {}; //initialize filter array
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
	return (fabs(yaw-last_yaw)>0.6 || fabs(pitch-last_pitch)>0.3);
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
		float fyaw=0,fpitch = 0;
		if (update) {
			//filter yaw & pitch to smooth the input, fyaw stands for filtered yaw angle
			convP.pit[n]=pitch;
			convP.yaw[n]=yaw;
			n=(n+1)%3;
			for (int i=0; i<3; i++) {
				fyaw += convP.yaw[i];
				fpitch += convP.pit[i];
			}
			fyaw /= 3;
			fpitch /= 3;
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
	
	//OpenCV stores in channel order of BGR.
	Mat InterestingSurf;
	//not necessary to extract the specific channel into the mat
	extractChannel(image, InterestingSurf, ch);//recover(next line too)
	ColSeg(InterestingSurf, 100, 0, InterestingSurf.rows - 1);//can be put into a vector instead of using a matrix
	LightBar = LocateValidPatch(InterestingSurf, 5, 0.7, 75);
	Point2i TargetXY;
	if (LightBar.size() > 1)
		TargetXY = LocateArmorCentre(LightBar, 10, 30);


	// Generate the angles and send through serial
//	PitAng = 180.0/pi*atan((refS.height/2 - TargetXY.y)/FocusPixel);
	YawAng = 180.0/pi*atan((refS.width/2 - TargetXY.x)/FocusPixel);
	PitAng = 180.0/pi*atan((refS.height/2 - TargetXY.y-distanceVer)/FocusPixel);
//	double distanceTarReal=distanceTar*pixel2real; //recover
//	double distanceTarReal=distanceTar*2400; //recover
//	double offsetAngPit = 0.5*(PitAng - asin((v*v*distanceTarReal*sin(PitAng)-9.8*distanceTarReal*cos(PitAng)*distanceTarReal*cos(PitAng))/(v*v*distanceTarReal))); //calculate the offset angle of the gravity, recover
//	PitAng = 0.5*(asin((v*v*distanceTarReal*sin(PitAng)-9.8*distanceTarReal*cos(PitAng)*distanceTarReal*cos(PitAng))/(v*v*distanceTarReal))+PitAng); //recover	
//	PitAng += 180/pi*offsetAngPit; //accurate PitAng after gravaty and view coordinate adjustment, recover


	contFrame[2] = contFrame[1];
	contFrame[1] = contFrame[0];
	contFrame[0].x = TargetXY.x;
	contFrame[0].y = TargetXY.y;
	contFrame[0].d = distanceTar;

	if (TargetXY.x != 0) {
		IsFound = 1;
		temp[1] = temp[0];
		temp[0] = predict(contFrame, 1);
	}
	else {
		PitAng = 0;
		YawAng = 0;
	}

	pitch = PitAng;
	yaw = YawAng;
		
	if (IsShow)
		showImage(image, IsFound, contFrame[0], temp[1]);

	if (waitKey(1) != -1)
	{
		cout << "Paused by user. Wanna continue? Yes 1 or No 0: ";
		cin >> IsBlue; // Make use of this variable.
		if (IsBlue)
		{
			cout << "Blue 1 or red 0, make your choice: ";
			cin >> IsBlue;
		}
		else
			return;
	}
	// Print out processing time
	long LastTime = clock() - TPrev;
	cout << "Image count: " << count << ", CLOCKS_PER_SEC: " << CLOCKS_PER_SEC
		<< ", Lasting time: " << LastTime << endl;
}

vector<RotatedRect> LocateValidPatch(Mat InputImg, int MinArea, double MinEccen, double MinOri)
{
	// MinArea sets minimun area requirement for a patch; MinEccen: smallest eccentricity; MinOri: minimum orientation
	vector<vector<Point>> contours;
	// findContours(InputImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
	findContours(InputImg, contours, 0, 1, Point());
	vector<RotatedRect> minEllipse(contours.size());
	vector<RotatedRect> ValidPatch;
	
	double MaxRatio = sqrt(1 - MinEccen * MinEccen);
	
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > MinArea)
		{
			minEllipse[i] = fitEllipse(contours[i]);
			// calibrate its orientation
			// (90 - minEllipse[i].angle) is the orientation wrt x-axis and is thus put into the functoin 
			minEllipse[i].angle = 90 - OrieUndist(minEllipse[i].center.x, minEllipse[i].center.y, (90-minEllipse[i].angle));
			double WHRatio = minEllipse[i].size.width / minEllipse[i].size.height;
			// The angle is that rates a "vertical tall rectangle" in counterclockwise direction.
			// "-3.0/contours[i].size()" is used to compensate for small patch where eccentricity is hard to guarantee.
			if ((abs(90 - minEllipse[i].angle) > MinOri) && ((WHRatio-3.0/contours[i].size()) < MaxRatio))
			{
				ValidPatch.push_back(minEllipse[i]); // A valid patch
			}
		}
	}
	return ValidPatch;
}

Point2i LocateArmorCentre(vector<RotatedRect> LBar, double OrieDiffMax, double CentIncliDiffMax)
{
	Point2i CentreXY(0, 0);
	int ArrLen = LBar.size();
	int MaxInd = 0;
	int SecMaxInd = 0; // second largest

	// For simplicity, use valarray for element-wise computations. All are initialised as 0s.
	valarray<double> Area(ArrLen); //area
	valarray<double> Centx(ArrLen); //centroidx
	valarray<double> Centy(ArrLen); //centroidy
	valarray<double> Ecc(ArrLen); //eccentricity
	valarray<double> Orie(ArrLen); //orientation to the right hand side direction
	valarray<double> MaAxLen(ArrLen); //majorAxisLength

	for (int i = 0; i < ArrLen; i++)
	{
		Area[i] = LBar[i].size.area();
		if (Area[i] > Area[MaxInd])
		{
			SecMaxInd = MaxInd;
			MaxInd = i; // find out the one with maximum area				
		}
		else if (Area[i] > Area[SecMaxInd])
		{
			SecMaxInd = i;
		}
		Centx[i] = LBar[i].center.x;
		Centy[i] = LBar[i].center.y;
		double WHRatio = LBar[i].size.width / LBar[i].size.height;
		Ecc[i] = sqrt(1 - WHRatio * WHRatio);
		Orie[i] = 90 - LBar[i].angle;
		MaAxLen[i] = LBar[i].size.height;
	}
	// When seeing a large candidate, DQ it if it's larger than 1.5 of the second largest
	// Because it is probably just the main judge system's light. 
	if (Area[MaxInd] > 750 && Area[MaxInd] > 1.5*Area[SecMaxInd])
	{
		Area[MaxInd] = 0;
		MaxInd = SecMaxInd;
	}

	/* Validity criteria:
	In a pair, one's major axis should not be 3 times longer than the other, otherwise it is invalid;
	Inclinations should not be too different;
	Distance should not be too far or too near;
	Height should not be too different;
	Centre line should not form an angle less than 30 to original orientation.
	*/
	valarray<double> OrieDiff(ArrLen);
	valarray<double> DistDiff(ArrLen);
	valarray<double> CentIncliDiff(ArrLen);
	valarray<double> MaAxLenAve(ArrLen);
	valarray<double> HeightDiff(ArrLen);
	valarray<bool> ValidPair(ArrLen);
	int PairInd = -1;
	for (int i = 0; i < ArrLen - 1; i++) // Run from the largest patch
	{
		// Find the indice as the one with the largest area. 
		MaxInd = -1;
		for (int j = 0; j < ArrLen; j++)
		{
			if (MaxInd == -1) { MaxInd = j; }
			else { if (Area[j] > Area[MaxInd]) { MaxInd = j; } }
		}
		OrieDiff = abs(Orie - Orie[MaxInd]) - 90.0;
		OrieDiff = 90.0 - abs(OrieDiff); // turn into an absolute value with 90
		DistDiff = sqrt((Centx - Centx[MaxInd])*(Centx - Centx[MaxInd]) + (Centy - Centy[MaxInd])*(Centy - Centy[MaxInd]));
		// Note coordinate direction of y is inverted in image coordiates!
		CentIncliDiff = abs(Orie[MaxInd] - 180.0 / pi * atan((Centy[MaxInd] - Centy) / (Centx - Centx[MaxInd])));
		CentIncliDiff = 90.0 - abs(CentIncliDiff - 90.0);
		MaAxLenAve = (MaAxLen + MaAxLen[MaxInd]) / 2.0;
		HeightDiff = abs(Centy - Centy[MaxInd]);
		ValidPair = (MaAxLenAve > 2.0 / 3 * MaAxLen[MaxInd]) && (MaAxLenAve < 2 * MaAxLen[MaxInd])
			&& (OrieDiff < OrieDiffMax) && (HeightDiff < 1.5*MaAxLenAve) && (DistDiff < 5.0*MaAxLenAve)
			&& (DistDiff > MaAxLenAve) && (CentIncliDiff > CentIncliDiffMax);
		if (ValidPair.sum() == 0) // no valid pair found
		{
			Area[MaxInd] = 0; // DQ this patch and move to next candidate
			/*
			if (i == ArrLen - 2)
				cout << "No valid pair found. " << endl;*/
		}
		else
		{
			// Find the pair's index as that with the largest area. 
			for (int j = 0; j < ValidPair.size(); j++)
			{
				if (ValidPair[j])
					if (PairInd == -1)
					{ PairInd = j; }
					else
					{ if (Area[j] > Area[PairInd]) { PairInd = j; } }
			}
			CentreXY.x = (Centx[MaxInd] + Centx[PairInd]) / 2;
			CentreXY.y = (Centy[MaxInd] + Centy[PairInd]) / 2;
			//cout << "Valid pair found with [" << MaxInd << "] and [" << PairInd << "], at " << TargetXY << endl;
			break;
		}
		distanceTar = FocusPixel * armHeight / MaAxLenAve[MaxInd]; //calculate the distance to the target, recover
	}

	return CentreXY;
}

void ColSeg(Mat& img, int Thre, int StaRow, int EndRow)
{
	// from start row StaRow to end row EndRow (both included).
	uchar* p;
	for (int i = StaRow; i <= EndRow; ++i) // Careful it's <= to include EndRow
	{
		p = img.ptr<uchar>(i);
		for (int j = 0; j < img.cols; ++j)
		{
			p[j] = 255 * (p[j] > Thre);
		}
	}
}

double OrieUndist(double CenX, double CenY, double Orie)
{
	// This function is to calibrate the true orientation of an line on the graph
	// Note units should all be radian except Orie and OrieCali;
	double xT, yT; // True coordinates with (0,0) being at the centre and +y points upward. 
	xT = CenX - VideoWidth / 2.0;
	if (xT == 0)
		xT += 0.1; // avoid dividing by zero in atan();
	yT = VideoHeight / 2.0 - CenY;
	double Gamma = atan(sqrt(xT*xT + yT * yT) / FocusPixel); // Angle to be rotated around the axis
	double OrieCali = Orie;
	if (Gamma > 2 * pi / 180) // only calibrate when angle is big enough
	{
		double RotateAxisOrie = atan(yT / xT); // Rotation axis.
		double OrieNew = RotateAxisOrie - pi / 180 * Orie; // Orientation relative to the rotation axis;
		// What if atan(inf) is put into this since pi is just an approximation?
		// Cast into range (-pi/2, pi/2]
		OrieNew = OrieNew - pi * (OrieNew > pi / 2);
		OrieNew = OrieNew + pi * (OrieNew <= -pi / 2);
		if (abs(OrieNew) < (pi - 0.01)) // Do so only when angle is not so big to avoid boudary crossing and infinity detections
			OrieNew = atan(tan(OrieNew)*cos(Gamma));// The calibrated result
		OrieCali = RotateAxisOrie - OrieNew;
		// Cast into range [-pi/2, pi/2)
		OrieCali = OrieCali - pi * (OrieCali > pi / 2);
		OrieCali = OrieCali + pi * (OrieCali <= -pi / 2);
		OrieCali = 180 / pi * OrieCali;
	}
	return OrieCali;
}

Position predict(Position f[], double time)
{
	double newX;
	double newY;
	double newD;

	if (f[1].x == 0)
	{
		return f[0];
	}
	else if (f[2].x == 0)
	{
		newX = f[0].x * (time + 1) - f[1].x * time;
		newY = f[0].y * (time + 1) - f[1].y * time;
		newD = f[0].d * (time + 1) - f[1].d * time;

		return (struct Position){newX, newY, newD, time};
	}
	else
	{
		double XA = (f[0].x + f[2].x)/2 - f[1].x;
		double XB = (f[0].x - f[2].x)/2;
		double YA = (f[0].y + f[2].y)/2 - f[1].y;
		double YB = (f[0].y - f[2].y)/2;
		double DA = (f[0].d + f[2].d)/2 - f[1].d;
		double DB = (f[0].d - f[2].d)/2;

		double constB = time + 1;
		double constA = constB * constB;
		newX = constA * XA + constB * XB + f[1].x;
		newY = constA * YA + constB * YB + f[1].y;
		newD = constA * DA + constB * DB + f[1].d;

		return (struct Position){newX, newY, newD, time};
	}
}

void showImage(Mat image, bool found, Position Now, Position Predict)
{
	CvScalar colour;
	if (IsBlue)
		colour = CV_RGB(0, 0, 255);
	else
		colour = CV_RGB(255, 0, 0);

	for (int i = 0; i< LightBar.size(); i++)
	{
		// ellipse
		ellipse(image, LightBar[i], colour, 2, 8);
	}
	if (found)
	{
		colour = CV_RGB(255, 255, 255);
		line(image, { (int)Now.x - 10, (int)Now.y }, { (int)Now.x + 10, (int)Now.y }, colour, 2);
		line(image, { (int)Now.x, (int)Now.y - 10 }, { (int)Now.x, (int)Now.y + 10 }, colour, 2);

		colour = CV_RGB(255, 0, 0);//draw the predict point
		line(image, { (int)Predict.x - 10, (int)Predict.y }, { (int)Predict.x + 10, (int)Predict.y }, colour, 2);
		line(image, { (int)Predict.x, (int)Predict.y - 10 }, { (int)Predict.x, (int)Predict.y + 10 }, colour, 2);
	}

	namedWindow("Contours window");
	imshow("Contours window", image);
}
