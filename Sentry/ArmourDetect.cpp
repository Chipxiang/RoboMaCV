#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp> // region detection
#include <time.h>
#include <opencv2/core/types_c.h> // To use CvScalar
#include <stdio.h>
#include <iostream>
#include <valarray> // for faster and simple vector math
#include <cmath> // atan calculation (seems valarray already includes it.)
#include <thread> // Multi threading
#include <unistd.h> // linux read and write
#include <fcntl.h> // linux file input, output
#include <signal.h>
#include "ArmourDetect.h"

using namespace cv;
using namespace std;

struct Position
{
	double x;
	double y;
	double d;
	double time;
};

const double pi = 3.14159265359;
const int VideoWidth = 1280; //1920
const int VideoHeight = 720; //1080
const int ThreNum = 3; //After test, 3 is the optimal for Odroid UX4
extern bool IsFound;
const bool IsShow = false;
const bool UsingCam = true; //false for video
bool IsBlue; 
const double VAngle = 76 * pi/180; // radian
const double armHeight = 0.1; //the height of the armour light, recover
const double FocusPixel = 1.0/tan(VAngle/2)*VideoWidth/2;
const double v=20; //velocity, recover
//const double pixel2real=20; //recover
double detectInt = 0.1;//second
double distanceTar; //the distance from target to the camera, recover
const double distanceVer=35; //the vertical distance between the camera center and the barrel(in pixel), recover

Position contFrame[3] = {(struct Position){0, 0, 0, 0}, (struct Position){0, 0, 0, 0}, (struct Position){0, 0, 0, 0}};//for prediction

vector<RotatedRect> LocateValidPatch(Mat, int, double, double);
vector<RotatedRect> LightBar;
Point2i LocateArmorCentre(vector<RotatedRect>, double, double);
double OrieUndist(double CenX, double CenY, double Orie);
Position temp[2];//for test

void ColSeg(Mat&, int, int, int);
Position predict(Position f[], double time);
void showImage(Mat image, bool found, Position Now, Position Predict);

extern float yaw,pitch;
static VideoCapture MyVideo;
Size refS;

/* Camera settings:
   Brightness -64
   Contrast 0
   Saturation 128
   Hue 0
   Gamma 160
   Gain 0
   White Balance Temperature 4000
   Sharpness 0
   Backlight Compensation
   Exposure 16
*/

void CamInitial() {
        //MyVideo.set(CV_CAP_PROP_BUFFERSIZE, 3); 
	if (UsingCam)
	{
		//string pipeline = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)2592, height=(int)1944, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
		//string pipeline = "v4l2src device='/dev/video1' ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)I420 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
		//MyVideo.open(pipeline, CAP_GSTREAMER);

		MyVideo.open(1); 
		if (!MyVideo.isOpened())
		{
			cout << "Could not open the camera. " << endl;
			return;
		}
		else
			cout << "Camera successfully opened. " << endl;
	}
	else
	{
		MyVideo.open("LabRecordB.avi"); 
		if (!MyVideo.isOpened())
		{
			cout << "Could not open the video. " << endl;
			return;
		}
		else
			cout << "Video successfully opened. " << endl;
	}

	if (UsingCam)
	{
		cout << "Setting up the camera" << endl;
		// Setup Camera 
		MyVideo.set(CV_CAP_PROP_FRAME_WIDTH, VideoWidth);
		MyVideo.set(CV_CAP_PROP_FRAME_HEIGHT, VideoHeight);
		MyVideo.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
		cout << "FPS:" << MyVideo.get(CV_CAP_PROP_FPS) << endl;
		
		// Seems ubuntu cannot get correct data from MyVideo.get() command.
	}
	refS = Size(VideoWidth, VideoHeight);
	double FramteRate = MyVideo.get(CAP_PROP_FPS);

	cout << "Blue 1 or red 0, make your choice: ";
	cin >> IsBlue; 
	cout << "Processing starts!" << endl;
}

void Detect()
{	
	int count = 0;
	time_t TPrev = clock();	// In Odroid XU4 a CPU's working time is in nanoseconds. 

	IsFound = 0;
	int ch = IsBlue ? 0 : 2; // blue channel or red channel
	float PitAng, YawAng; 
	Mat image;
	MyVideo >> image; // Read the next frame
	if (image.empty()) // Check if read out
	{
		cout << "Video has been read out. " << endl;
		return;
	}
	
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
