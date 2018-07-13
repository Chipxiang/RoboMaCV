#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp> // region detection
#include <opencv2/objdetect/objdetect.hpp>
#include <time.h>
#include <opencv2/core/types_c.h> // To use CvScalar
#include <stdio.h>
#include <iostream>
#include <valarray> // for faster and simple vector math
#include <cmath> // atan calculation (seems valarray already includes it.)
#include <thread> // Multi threading
#include <unistd.h> // linux read and write
#include <fcntl.h> // linux file input, output

using namespace cv;
using namespace std;

const double pi = 3.14159265359;
const bool IsShow = true;
const bool UsingCam = false;
const double VAngle = 76*pi/180; // radian
const int VideoWidth = 640; //1280
const int VideoHeight = 480; //720
const int ThreNum = 3; //After test, 3 is the optimal for Odroid UX4
const double FocusPixel = 1.0/tan(VAngle/2)*VideoWidth/2;;
bool foundSudoku = false;
bool foundLED = false;
void ColSeg(Mat&, int, int, int);
vector<RotatedRect> LocateValidPatch(Mat, int, double, double);
Point2i LocateArmorCentre(vector<RotatedRect>, double, double);
double OrieUndist(double CenX, double CenY, double Orie);
bool checkSudoku(const vector<vector<Point2i>> & contours, vector<RotatedRect> & sudoku_rects);
void chooseTargetPerspective(const Mat & image, const vector<RotatedRect> & sudoku_rects);
double self_max(double a, double b, double c);
double self_min(double a, double b, double c);
Mat findRed(Mat find_frame);
bool findRange(Mat input_frame);
Point match_number(Mat find_roi, int number);
void initializeTemplates();
bool recognizeLED(Mat redRaw);
Mat templates[9];
//Mat copy_frame;
int target_digit = 1;
int ledDigits[5];
Mat redNums[5], redBoard,fire_roi;
bool foundRange = false;
Rect roi_rect;
Point target_point = Point(1,1);


struct Point2fWithIdx {
		cv::Point2f p;
		size_t idx;
		Point2fWithIdx(const cv::Point2f _p, size_t _idx) :p(_p), idx(_idx){}
};
struct contour_sorter // 'less' for contours
{
    bool operator ()( const vector<Point>& a, const vector<Point> & b )
    {
        Rect ra(boundingRect(a));
        Rect rb(boundingRect(b));
        // scale factor for y should be larger than img.width
        return (ra.x < rb.x);
    }
};
vector<cv::RotatedRect> sudoku_rects;
Mat sudoku_mat[9];
int sudoku_width = 127;
int sudoku_height = 71;
RotatedRect adjustRRect(const cv::RotatedRect & rect);
/* Armour Camera settings:
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

/* Rune Camera settings:
   Brightness 0
   Contrast 32
   Saturation 60
   Hue 0
   Gamma 100
   Gain 0
   White Balance Temperature 4600
   Sharpness 2
   Backlight Compensation 1
   Exposure Aperture Priority Mode(157)
*/


int Start()
{
	initializeTemplates();
	thread th[ThreNum]; // Create threads
	cout << "Initilising..." << endl;
	// Create a VideoCapture object and use camera to capture the video
	VideoCapture MyVideo;
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
			return -1;
		}
		else
			cout << "Camera successfully opened. " << endl;
	}
	else
	{
		cout << "Enter Video Name:" << endl;
		string inputVideo = "nine.mp4";
		//cin >> inputVideo;
		MyVideo.open(inputVideo); 
		if (!MyVideo.isOpened())
		{
			cout << "Could not open the video. " << endl;
			return -1;
		}
		else
			cout << "Video successfully opened. " << endl;
	}
	
	// Setup Serial
	int idxSerialPortIndex = 0;
	char strSerialPort[255];
	/*int fd;
	while(true) // Try all ports till a valid one;
	{
		sprintf(strSerialPort, "/dev/ttyACM%d", idxSerialPortIndex);
		fd = open(strSerialPort, O_RDWR);
		if (fd == -1)
			idxSerialPortIndex++;
		else
		{
			cout << "Port " << strSerialPort << " successfully opened. " << endl;
			break;
		}
		if (idxSerialPortIndex > 100) 
		{
			cout << "Error in opening the port." << endl;
			return -1;
		}
	}
	*/
	if (UsingCam)
	{
		cout << "Setting up the camera" << endl;
		// Setup Camera 
		MyVideo.set(CV_CAP_PROP_FRAME_WIDTH, VideoWidth);
		MyVideo.set(CV_CAP_PROP_FRAME_HEIGHT, VideoHeight);
		MyVideo.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
		cout << "FPS:" << MyVideo.get(CV_CAP_PROP_FPS) << endl;
		

		
		//MyVideo.set(CAP_PROP_EXPOSURE, 100); 

		cout << "Bri, Sat, Cont, Expo: " << endl
			<< MyVideo.get(CAP_PROP_BRIGHTNESS) << ", "
			<< MyVideo.get(CAP_PROP_SATURATION) << ", "
			<< MyVideo.get(CAP_PROP_CONTRAST) << ", "
			<< MyVideo.get(CAP_PROP_EXPOSURE) << endl;
		
		// Seems ubuntu cannot get correct data from MyVideo.get() command.
		// Size refS = Size((int)MyVideo.get(CAP_PROP_FRAME_WIDTH), (int)MyVideo.get(CAP_PROP_FRAME_HEIGHT));
	}
	Size refS = Size(VideoWidth, VideoHeight);
	double FramteRate = MyVideo.get(CAP_PROP_FPS);
	bool isArm = false;
	cout << "Rune Mode(0) or Armour Mode (1)?";
	//cin >> isArm;
	if (isArm){
					
		cout << "Blue 1 or red 0, make your choice: ";
		bool IsBlue; 
		cin >> IsBlue; 
		cout << "Processing starts!" << endl;
		int count = 0;
		time_t TPrev = clock();	// In Odroid XU4 a CPU's working time is in nanoseconds. 
		while(true)
		{
			int8_t IsFound = 0;
			int ch = IsBlue ? 0 : 2; // blue channel or red channel
			int8_t PitAng, YawAng; 
			Mat image;
			MyVideo >> image; // Read the next frame
			if (image.empty()) // Check if read out
			{
				cout << "Video has been read out. " << endl;
				break;
			}
			
			//OpenCV stores in channel order of BGR.
			Mat InterestingSurf;
			extractChannel(image, InterestingSurf, ch);
			ColSeg(InterestingSurf, 100, 0, InterestingSurf.rows - 1);

			/*for (int i = 0; i < ThreNum; i++) // MultiThreading
			{
				int SRow = 1.0*i/ThreNum*InterestingSurf.rows;
				int ERow = 1.0*(i+1)/ThreNum*InterestingSurf.rows - 1;
				th[i] = thread(ColSeg, ref(InterestingSurf), 100, SRow, ERow);			
			}
			for (int i = 0; i < ThreNum; i++) // Wait till all finishes
			{
				if (th[i].joinable()) // just for safety, probably not necessary 
					th[i].join();
			}*/
			vector<RotatedRect> LightBar = LocateValidPatch(InterestingSurf, 5, 0.7, 75);
			Point2i TargetXY;
			if (LightBar.size() > 1)
			{
				TargetXY = LocateArmorCentre(LightBar, 10, 30);
			}
			// Generate the angles and send through serial
			PitAng = 180/pi*atan((refS.height/2 - TargetXY.y)/FocusPixel);
			YawAng = 180/pi*atan((refS.width/2 - TargetXY.x)/FocusPixel);
			if (TargetXY.x != 0)
				IsFound = 1;
			else
			{
				PitAng = 0;
				YawAng = 0;
			}
			int8_t SendSig[4] = {IsFound, PitAng, YawAng, 90}; // 90 is just for check. 
			//int8_t SendSig[4] = {49,50,51,52};
			//int NumWrite = write(fd, SendSig, 4);
			
			if (IsShow)
			{
				cout << int(SendSig[0]) << " " << int(SendSig[1]) << " " << int(SendSig[2]) << " " << int(SendSig[3]) <<endl;
				// Draw identified cross
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
				if (TargetXY.x != 0)
				{
					colour = CV_RGB(255, 255, 255);
					line(image, { TargetXY.x - 10, TargetXY.y }, { TargetXY.x + 10, TargetXY.y }, colour, 2);
					line(image, { TargetXY.x, TargetXY.y - 10 }, { TargetXY.x, TargetXY.y + 10 }, colour, 2);
				}
			
				//namedWindow("Contours window");
				imshow("Contours window", image);
			}
			//waitKey(0);
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
					break;
			}
			/*	
			// Keep the frame rate
			TCurr = clock();

			int WaitTime = 1000.0 / FramteRate - (TCurr - TPrev);
			if (WaitTime > 0 )
			{

				Sleep(WaitTime);
			}
			
			//TPrev = TCurr;
			
			if (count == 100)
				break;*/
			count++;
		} // while(true)
		/**/
		// Print out processing time
		long LastTime = clock() - TPrev;
		cout << "Image count: " << count << ", CLOCKS_PER_SEC: " << CLOCKS_PER_SEC
			<< ", Lasting time: " << LastTime << endl;
		
		// cout << "Lasting time: " << LastTime << endl;
		//close(fd); // Close serial port. 

		return 0;
	}
	else{
		//cout << "***testing on qiyue section" << endl;
		//cout << "Flame(0) or Written (1) ***" << "###" << endl;
		//bool isWritten = true;
		//cin >> isWritten;
		cout << "Process Starts" << endl;
		int count = 0;
		time_t TPrev = clock();	// In Odroid XU4 a CPU's working time is in nanoseconds. 
		
		while(true)
		{
			//foundSudoku = false;
			int8_t IsFound = 0;
			int8_t PitAng, YawAng; 
			Mat image;
			MyVideo >> image; // Read the next frame
			
			if (image.empty()) // Check if read out
			{
				cout << "Video has been read out. " << endl;
				break;
			}
			resize(image, image, Size(640, 360), 0, 0, INTER_CUBIC);
			Mat src;
			cvtColor(image, src, CV_BGR2GRAY);
			Mat binary;
			threshold(src, binary, 160, 255, THRESH_BINARY);
			//threshold(src, binary, 200, 255, THRESH_BINARY);
			vector<vector<Point2i>> contours;
			vector<Vec4i> hierarchy;
			
			findContours(binary, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			sudoku_rects.clear();
			foundSudoku = checkSudoku(contours, sudoku_rects);
			if (foundSudoku){
				chooseTargetPerspective(binary, sudoku_rects);
				foundSudoku = true;
			}
			else{
				foundRange = findRange(image);
			}
			CvScalar colour;
			colour = CV_RGB(0, 0, 255);
			//qiyue adding
			//cout << "testing on qiyue section" << endl;
			//cout << "testing on foundSudoku: " << foundSudoku << endl;
			//output center
			
			if (foundRange){
				Mat redRaw = findRed(image);
				if(!redRaw.empty()){
					//foundLED = true;
					foundLED = recognizeLED(redRaw);					
				}
				else{ foundLED = false; }
				/*Point targetCenter = match_number(fire_roi, 9);
				targetCenter.x += roi_rect.x;
				targetCenter.y += roi_rect.y;
				//cout << roi_rect.size() << endl;		
				imshow("roi", fire_roi);
				if(targetCenter!= Point(-1,-1)){
					line(image, {targetCenter.x - 10, targetCenter.y }, { targetCenter.x + 10, targetCenter.y }, CV_RGB(0, 0, 255), 2);
					line(image, {targetCenter.x, targetCenter.y - 10 }, { targetCenter.x, targetCenter.y + 10 }, CV_RGB(0, 0, 255), 2);
				}*/
				//imshow("Target", copy_frame);
			}
			
			else if (foundSudoku){
				/*for(int i=0; i<9; i++){
					cout << i << "th sudoku" << endl;
					cout << sudoku_rects[i].center.x << " " << sudoku_rects[i].center.y << endl;
					cout << endl;
				}*/
				int ledBuffer[5] = {0};
				double xa,xb,xc,xd,xred;
				xc = sudoku_rects[8].center.x + (112.0/370.0)*(sudoku_rects[7].center.x - sudoku_rects[8].center.x) * ((sudoku_rects[7].center.x - sudoku_rects[8].center.x)/(sudoku_rects[6].center.x - sudoku_rects[7].center.x));
				xd = sudoku_rects[8].center.x + ((112.0+520.0)/370.0)*(sudoku_rects[7].center.x - sudoku_rects[8].center.x) * ((sudoku_rects[7].center.x - sudoku_rects[8].center.x)/(sudoku_rects[6].center.x - sudoku_rects[7].center.x));
				//xred = (104.0/370.0)*(sudoku_rects[7].center.x - sudoku_rects[8].center.x) * ((sudoku_rects[7].center.x - sudoku_rects[8].center.x)/(sudoku_rects[6].center.x - sudoku_rects[7].center.x));
				double ya,yb,yc,yd;
				ya = sudoku_rects[8].center.y - (276.0/220.0)*(sudoku_rects[5].center.y - sudoku_rects[8].center.y) * ((sudoku_rects[5].center.y - sudoku_rects[8].center.y)/(sudoku_rects[2].center.y - sudoku_rects[5].center.y));
				yb = sudoku_rects[6].center.y - (276.0/220.0)*(sudoku_rects[3].center.y - sudoku_rects[6].center.y) * ((sudoku_rects[3].center.y - sudoku_rects[6].center.y)/(sudoku_rects[0].center.y - sudoku_rects[3].center.y));
				yc = sudoku_rects[8].center.y - (152.2/220.0)*(sudoku_rects[5].center.y - sudoku_rects[8].center.y) * ((sudoku_rects[5].center.y - sudoku_rects[8].center.y)/(sudoku_rects[2].center.y - sudoku_rects[5].center.y));
				yd = sudoku_rects[6].center.y - (152.2/220.0)*(sudoku_rects[3].center.y - sudoku_rects[6].center.y) * ((sudoku_rects[3].center.y - sudoku_rects[6].center.y)/(sudoku_rects[0].center.y - sudoku_rects[3].center.y));
				//cout << "xc = " << xc << endl;
				//cout << "xd = " << xd << endl;
				//cout << "ya = " << ya << endl;
				//cout << "yb = " << yb << endl;
				//cout << "yc = " << yc << endl;
				//cout << "yd = " << yd << endl;
				//cout << "xred = " << xred << endl;
				//cout << endl;

				xc = xc * 0.98;
				ya = ya * 0.98;
				yb = yb * 0.98;
				xd = xd * 1.02;
				yc = yc * 1.02;
				yd = yd * 1.02;
				
				if (ya<0 || ya > self_max(sudoku_rects[8].center.y, sudoku_rects[7].center.y, sudoku_rects[6].center.y) ||
					yb<0 || yb > self_max(sudoku_rects[8].center.y, sudoku_rects[7].center.y, sudoku_rects[6].center.y) ||
					yc<0 || yc > self_max(sudoku_rects[8].center.y, sudoku_rects[7].center.y, sudoku_rects[6].center.y) ||
					yd<0 || yd > self_max(sudoku_rects[8].center.y, sudoku_rects[7].center.y, sudoku_rects[6].center.y) ||
					xc<0 || xc > self_min(sudoku_rects[1].center.x, sudoku_rects[4].center.x, sudoku_rects[7].center.x) ||
					xd<0 || xd > self_min(sudoku_rects[0].center.x, sudoku_rects[3].center.x, sudoku_rects[6].center.x) ){
						foundLED = false;
						//cout << "sudoku position wiered" << endl;
					}
				else{
					//foundLED = true;
					Mat redNum;
					Mat redRaw;
					Rect rect1(xc,min(ya,yb),xd-xc,max(yc,yd)-min(ya,yb));
					Mat binaryLED;
					threshold(src, binaryLED, 200, 255, THRESH_BINARY);
					binaryLED(rect1).copyTo(redRaw);
					//threshold(redBoard, redRev, 150, 255, THRESH_BINARY_INV);
					foundLED = recognizeLED(redRaw);					
					/*for(int i=0; i<5; i++){
						Rect rect2(xc+i*(1.0/5.0)*(xd-xc),min(ya,yb),(1.0/5.0)*(xd-xc),max(yc,yd)-min(ya,yb));
						binary(rect2).copyTo(redNum);
						
						redNums[i] = redNum;
						Mat resized, reversed;
						resize(redNums[i], resized, Size(28,28),INTER_CUBIC);
						threshold(resized, reversed, 150, 255, THRESH_BINARY_INV);
						redNums[i] = reversed;
						//cout << reversed << endl;
						imwrite("LED" + to_string(i) + ".jpg", reversed);

						waitKey(1);
					}*/
					
				}
				//end of adding
			}
			
			
			if (IsShow)
			{
				if (foundLED){
					line(image, { target_point.x - 10, target_point.y }, { target_point.x + 10, target_point.y }, colour, 2);
					line(image, { target_point.x, target_point.y - 10 }, { target_point.x, target_point.y + 10 }, colour, 2);
				}
				
				string ledString = "";
				for(int i=0; i< 5; i++){
					ledString += to_string(ledDigits[i]);
				}
				putText(image,ledString, Point(30,30), FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,0,255), 1, CV_AA);
				//namedWindow("Contours window");
				imshow("Contours window", image);
				waitKey(1);
			}/*
			*/
		}	
		return 0;
	}
/**/
	
} // main

bool recognizeLED(Mat redRaw){
	resize(redRaw, redRaw, Size(544,162),INTER_CUBIC);
	Mat element = getStructuringElement(CV_SHAPE_ELLIPSE, Size(5, 5));
	//erode(redRaw, redBoard,element, Point(-1, -1), 1, 1, 1);
	dilate(redRaw, redBoard,element, Point(-1, -1), 4, 1, 1);
	//imshow("Board",redBoard);
	vector<vector<Point>> contours;
	findContours(redBoard, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<vector<Point>> digitCnt;
	Rect ledRect;
	sort(contours.begin(), contours.end(), contour_sorter());
	int j=0;
	for (int i = 0; i < contours.size(); i++){
		if(contours[i].size() > 10){
			ledRect = boundingRect(contours[i]);
			//if(ledRect.width/ledRect.height>91.0/123.0-0.1 && ledRect.width/ledRect.height>91.0/123.0+0.1){
				//rectangle(redBoard, ledRect, colour);
				digitCnt.push_back(contours[i]);
				//cout << contours[i] << endl;
				j++;
			//}
			if (j>5){
				return false;
			}
		}
	}
	if (j!=5){
		return false;
	}
	//if(foundLED){
		for (int i=0; i<5; i++){
			ledRect = boundingRect(digitCnt[i]);
			//if 1
			if(ledRect.height *1.0 / (ledRect.width*1.0) > 2){
				ledDigits[i] = 1;
			}
			//other digits
			else{
				Point2f pts[4];
				int on[7] = {0};
				Mat roi0 = redBoard(ledRect);
				//dilate(roi0, roi0,element, Point(-1, -1), 2, 1, 1);
				vector<Point> shape;
				uchar* p;
				//cout << roi0 << endl << endl;
				bool f = false;
				for (int i = 0.09*roi0.rows; i < roi0.rows; ++i) // Careful it's <= to include EndRow
				{
					p = roi0.ptr<uchar>(i);
					for (int j = 0; j < roi0.cols; ++j)
					{
						if(p[j] != 0){
							pts[0] = Point(j,i);
							f = true;
							break;
						}
					}
					if(f) break;
				}
				//shape.push_back(Point(0, roi0.rows-1));
				
				for (int i = roi0.rows-0.09*roi0.rows-1; i >=0; --i)
				{
					p = roi0.ptr<uchar>(i);
					for (int j = roi0.cols-1; j >=0 ; --j)
					{
						if(p[j] != 0){
							pts[2] = Point(j,i);
							f = true;
							break;
						}
					}
					if(f) break;
				}
				
				for (int i = 0.09*roi0.rows; i < roi0.rows; ++i)
				{
					p = roi0.ptr<uchar>(i);
					for (int j = roi0.cols-1; j >=0 ; --j)
					{
						if(p[j] != 0){
							pts[3] = Point(j,i);
							f = true;
							break;
						}
					}
					if(f) break;
				}
				
				 // Assemble a rotated rectangle out of that info
				//RotatedRect box = minAreaRect(Mat(shape));
				//std::cout << "Rotated box set to (" << box.boundingRect().x << "," << box.boundingRect().y << ") " << box.size.width << "x" << box.size.height << std::endl;
				//rectangle(roi0, box.boundingRect(), colour);
				

				//pts[0] = Point(5, 0);
				//pts[1] = Point(0, roi0.rows-1);
				//pts[2] = Point(roi0.cols-1, roi0.rows-7);
				//pts[3] = Point(roi0.cols-1, 0);

				// Does the order of the points matter? I assume they do NOT.
				// But if it does, is there an easy way to identify and order 
				// them as topLeft, topRight, bottomRight, bottomLeft?

				cv::Point2f src_vertices[3];
				src_vertices[0] = pts[0];
				src_vertices[1] = pts[2];
				src_vertices[2] = pts[3];
				//src_vertices[3] = not_a_rect_shape[3];

				Point2f dst_vertices[3];
				dst_vertices[0] = Point(0, 0.09*roi0.rows);
				dst_vertices[1] = Point(roi0.cols-1, roi0.rows-0.09*roi0.rows-1);
				dst_vertices[2] = Point(roi0.cols-1, 0.09*roi0.rows);
			  
				Mat warpAffineMatrix = getAffineTransform(src_vertices, dst_vertices);

				cv::Mat rotated;
				cv::Size size(ledRect.width, ledRect.height);
				warpAffine(roi0, rotated, warpAffineMatrix, size, INTER_LINEAR, BORDER_CONSTANT);

				//imwrite("rotated.jpg", rotated);
				
				Mat roi = rotated;
				Size s = roi.size();
				
				int roiH = s.height;
				int roiW = s.width;
				int dW = roiW*0.31;
				int dH = roiH*0.18;
				int dHC = dH * 0.5;
				int segments[7][4] = 
				{
					{dW,0,ledRect.width-dW,dH}, //top
					{0,dH, dW, ledRect.height/2-dHC}, //top-left
					{ledRect.width - dW, dH, ledRect.width, ledRect.height/2-dHC}, //top-right
					{dW, ledRect.height/2 -dHC, ledRect.width-dW, ledRect.height/2 + dHC}, //center
					{0, ledRect.height/2+dHC, dW, ledRect.height-dH}, //bottom-left
					{ledRect.width - dW, ledRect.height/2+dHC, ledRect.width, ledRect.height-dH}, //bottom-right
					{dW, ledRect.height - dH, ledRect.width-dW, ledRect.height}
				};
				
				Mat test = Mat::zeros(rotated.size(), CV_64FC1);

				for(int j = 0; j<7; j++){
					Rect segRect = Rect(segments[j][0], segments[j][1], (segments[j][2] - segments[j][0]), (segments[j][3] - segments[j][1]));
					
					Mat segRoi = roi(segRect);
					float total = countNonZero(segRoi);
					float area = (segments[j][2] - segments[j][0])* (segments[j][3] - segments[j][1]);
					if(total*1.0/(area*1.0)>0.7){
						on[j] = 1;
						rectangle(test, segRect, Scalar(0, 0, 0));
						rectangle(rotated, segRect, Scalar(0, 0, 0));
					}else{
						on[j]=0;
					}
					if(total*1.0/(area*1.0)>0.6 &&  total*1.0/(area*1.0)< 0.8){
						imwrite("Test.jpg", rotated);
					}
					//imshow("1", test);
					//imshow("2", rotated);
				}
				
				int DIGITS_LOOKUP[9][7] = {
					{0, 0, 1, 0, 0, 1, 0},
					{1, 0, 1, 1, 1, 0, 1},
					{1, 0, 1, 1, 0, 1, 1},
					{0, 1, 1, 1, 0, 1, 0},
					{1, 1, 0, 1, 0, 1, 1},
					{1, 1, 0, 1, 1, 1, 1},
					{1, 0, 1, 0, 0, 1, 0},
					{1, 1, 1, 1, 1, 1, 1},
					{1, 1, 1, 1, 0, 1, 1}
				};
				int y = 0;
				bool flag = false;
				for (y = 0; y < 9; y++){
					for(int x = 0; x<7; x++){
						if(DIGITS_LOOKUP[y][x] != on[x])
							break;
						if(DIGITS_LOOKUP[y][x] == on[x] && x ==6)
							flag = true;
					}
					if (flag){
						break;
					}
				}
				
				//ledBuffer[i] = y+1;
				//if(y!=10){
					ledDigits[i] = y+1;
					//cout << ledRect.x << " " << ledRect.y << " " << ledRect.width << " " << ledRect.height << " " << endl << endl;
				//}
			}
			
			//cout << ledDigits[i] << " ";
			
		}
		/*int ledChangeCounter = 0;
		for(int i =0; i< 5;i++){
			if(ledBuffer[i] != ledDigits[i]){
				ledChangeCounter ++;
			}
		}
		if (ledChangeCounter >1){
			for(int i =0; i< 5;i++){
				if(ledBuffer[i] != 10){
					ledDigits[i] = ledBuffer[i];
				}
			}
		}*/
		//cout << endl;
	//}
	
	//imshow("redBoard",redBoard);

	//imwrite("Board.jpg", redRev);
	waitKey(1);
	
	
	//imshow("binary",binary);
	//waitKey(1);
}

void chooseTargetPerspective(const Mat & image, const vector<RotatedRect> & sudoku_rects){
    for (int i=0;i<9;i++){
		RotatedRect rect = sudoku_rects[i];
        // matrices we'll use
        Mat M, rotated, cropped;
        // get angle and size from the bounding box
        float angle = rect.angle;
        Size rect_size = rect.size;
        // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
        if (rect.angle < -45.) {
            angle += 90.0;
            swap(rect_size.width, rect_size.height);
        }
        // get the rotation matrix
        M = getRotationMatrix2D(rect.center, angle, 1.0);
        // perform the affine transformation
        warpAffine(image, rotated, M, image.size(), INTER_CUBIC);
        // crop the resulting image
        getRectSubPix(rotated, rect_size, rect.center, cropped);
        
        Mat resized, reversed, processed;
        resize(cropped, resized, Size(28,28),INTER_CUBIC);
        threshold(resized, reversed, 150, 255, THRESH_BINARY_INV);
        
        //Mat kernel = getStructuringElement(0, Size(3,3));
		//morphologyEx(reversed, processed, MORPH_OPEN, kernel);
		//imwrite("Sudoku" + to_string(i) + ".pgm", reversed);
		//imshow("Sudoku" + to_string(i), reversed);
		sudoku_mat[i] = reversed;
	}
	
}

bool checkSudoku(const vector<vector<Point2i>> & contours, vector<RotatedRect> & sudoku_rects){
	if (contours.size() < 9)
        return false;

    float width = sudoku_width;
	float height = sudoku_height;
	float ratio = 28.0 / 16.0;
	int sudoku = 0;

    float low_threshold = 0.6;
    float high_threshold = 1.4;
    vector<Point2f> centers;
    for (size_t i = 0; i < contours.size(); i++) {
		RotatedRect rect = minAreaRect(contours[i]);
		rect = adjustRRect(rect);
		const Size2f & s = rect.size;
		float ratio_cur = s.width / s.height;

		if (ratio_cur > 0.8 * ratio && ratio_cur < 1.2 * ratio &&
			s.width > low_threshold * width && s.width < high_threshold * width &&
			s.height > low_threshold * height && s.height < high_threshold * height &&
			((rect.angle > -10 && rect.angle < 10) || rect.angle < -170 || rect.angle > 170)){

			sudoku_rects.push_back(rect);
            centers.push_back(rect.center);
            //vector<Point2i> poly;
            //approxPolyDP(contours[i], poly, 20, true);
            ++sudoku;
		}
	}

    //cout << "sudoku num: " << sudoku << endl;

    if (sudoku > 15)
        return false;
	if (sudoku <9)
		return false;
    if(sudoku > 9){
        float dist_map[15][15] = {0};
        // calculate distance of each cell center
        for(int i = 0; i < sudoku; ++i){
            for (int j = i+1; j < sudoku; ++j){
                float d = sqrt((centers[i].x - centers[j].x)*(centers[i].x - centers[j].x) + (centers[i].y - centers[j].y)*(centers[i].y - centers[j].y));
                dist_map[i][j] = d;
                dist_map[j][i] = d;
            }
        }

        // choose the minimun distance cell as center cell
        int center_idx = 0;
        float min_dist = 100000000;
        for(int i = 0; i < sudoku; ++i){
            float cur_d = 0;
            for (int j = 0; j < sudoku; ++j){
                cur_d += dist_map[i][j];
            }
            if(cur_d < min_dist){
                min_dist = cur_d;
                center_idx = i;
            }
        }

        // sort distance between each cell and the center cell
        vector<pair<float, int> > dist_center;
        for (int i = 0; i < sudoku; ++i){
            dist_center.push_back(make_pair(dist_map[center_idx][i], i));
        }
        std::sort(dist_center.begin(), dist_center.end(), [](const pair<float, int> & p1, const pair<float, int> & p2) { return p1.first < p2.first; });

        // choose the nearest 9 cell as suduku
        vector<RotatedRect> sudoku_rects_temp;
        for(int i = 0; i < 9; ++i){
            sudoku_rects_temp.push_back(sudoku_rects[dist_center[i].second]);
        }
        sudoku_rects_temp.swap(sudoku_rects);
    }
	return sudoku_rects.size() == 9;
}

RotatedRect adjustRRect(const RotatedRect & rect){
	const Size2f & s = rect.size;
	if (s.width > s.height)
		return rect;
	return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
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
	}
	/*
	// For debug
	for (int i = 0; i < ArrLen; i++)
	{
	cout << "[" << i << "]" << LBar[i].center << ", " << LBar[i].size << ", " << Orie[i] << endl;
	}
	for (int i = 0; i < ArrLen; i++)
	{
	cout << "OrieDiff, DistDiff, PCentIncliDiff, PMajorAxisVAve [" << i << "]= ";
	cout << OrieDiff[i] << ", " << DistDiff[i] << ", " << CentIncliDiff[i] << ", " << MaAxLenAve[i] << endl;
	}*/
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

double self_max(double a, double b, double c){
	if (a >= b){
		if (a >= c){
			return a;
		}
		else{
			return c;
		}
	}
	else{
		if (b >= c){
			return b;
		}
		else {
			return c;
		}
	}
}

double self_min(double a, double b, double c){
	if (a <= b){
		if (a <= c){
			return a;
		}
		else{
			return c;
		}
	}
	else{
		if (b <= c){
			return b;
		}
		else {
			return c;
		}
	}
}

Point match_number(Mat find_roi, int number)
{
	string findpicture;
	Point maxLoc, minLoc;
	double minVal, maxVal;
	Mat dst = templates[number-1];
	if (!dst.data)
	{
		return Point(-1,-1);
	}
	vector<vector<Point>> contours;
	Mat binaryRoi;
	cvtColor(find_roi, binaryRoi, CV_BGR2GRAY);
	threshold(binaryRoi, binaryRoi, 180, 255, THRESH_BINARY);
	findContours(binaryRoi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
	//cout << boundingRect(contours[0]).height << " " << find_roi.rows << endl;
	
	
	resize(dst, dst, Size(find_roi.rows/4.6*(7.0/8.0),find_roi.rows/4.6), 0, 0, INTER_CUBIC);
	imshow("dstllalal", dst);
	
	/******************确保模板大小小于roi********************/
	//cout << "TEST" << endl;
	if (find_roi.rows < dst.rows || find_roi.cols < dst.cols)
		return Point(-1,-1);
	Mat right_mat;
	matchTemplate(find_roi, dst, right_mat, 0);
	//cout << "TEST" << endl;
	minMaxLoc(right_mat, &minVal, &maxVal, &minLoc, &maxLoc);
	//cout << minVal<< endl;
	Point point = Point(minLoc.x + dst.cols / 2, minLoc.y + dst.rows / 2);
	//rectangle(find_roi, Rect(minLoc.x, minLoc.y, dst.cols, dst.rows), Scalar(0, 0, 255), 1);
	//circle(find_roi, point, 2, Scalar(255, 0, 0), 2);
	//cout << "中心点："<<point << endl;
	return point;
}

Mat findRed(Mat find_frame){
	Mat redRaw;
	//cout << "***************" << endl;
	//cout << roi_rect.x << "," << roi_rect.y << endl;
	//cout << roi_rect.width << " " << roi_rect.height << endl;
	int width = roi_rect.width*580.0/1140.0;
	int height = roi_rect.height*180.0/710.0;
	//cout << width << " " << height << endl;
	int xa, ya, xd, yd;
	xa = roi_rect.x + roi_rect.width*280.0/1140.0;
	xd = xa + roi_rect.width*(300.0+560.0)/1140.0;
	ya = roi_rect.y - roi_rect.height*190.0/710.0;
	yd = ya + roi_rect.height*180.0/710.0;
	//cout << xa << " " << ya << endl;
	//cout << " " << xd << " " << yd << endl;
	//cout << "######################" << endl;
	if (xa< roi_rect.x || xa > roi_rect.width + roi_rect.x ||
		ya< 0 || ya > roi_rect.y ||
		xd< roi_rect.x || xd > roi_rect.width + roi_rect.x ||
		yd<0 || yd > roi_rect.y ){
			return redRaw;
			//cout << "sudoku position wiered" << endl;
		}
	else{
		//cout << "######################" << endl;
		//foundLED = true;
		Rect rect1(xa, ya, width, height);
		Mat binaryLED;
		cvtColor(find_frame, binaryLED, CV_BGR2GRAY); 
		threshold(binaryLED, binaryLED, 240, 255, THRESH_BINARY);
		binaryLED(rect1).copyTo(redRaw);
		return redRaw;
	}
	return redRaw;
}

bool findRange(Mat input_frame)
{
	//cout << "enter find range" << endl;
	//dst = Mat::zeros(int_frame.size(), CV_8UC3);
	/*drawing 画布 将一些画矩形的操作在这个窗口输出*/
	
	
	//copy = int_frame.clone();
	/* copy_frame 备份*/
	Mat copy_frame, int_frame;
	input_frame.copyTo(copy_frame);
	input_frame.copyTo(int_frame);
	//imshow("in", copy_frame); //copy_frame is the original frame
	/****************灰度 二值化******************/
	cvtColor(int_frame, int_frame, CV_BGR2GRAY); //int_frame is the converted frame
	
	
	threshold(int_frame, int_frame, 130, 255, THRESH_BINARY); //two flags determine the best threashold for binarization
	//imshow("thresh", int_frame);
	blur( int_frame, int_frame, Size(3,3) );
	Canny(int_frame, int_frame, 120, 240); //这里用Canny的效果还不错
	
		

	 //int_frame is the binarized image
	/**************开始找两侧的白色灯光*******************/
	vector<vector<Point> > contours;
	vector<Rect> rightRect; //all the contours on the right half
	vector<Rect> leftRect;
	
	findContours(int_frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	if (contours.size()>5) {
		//cout << "come on:"<<contours.size()<<endl;
		vector<Rect> boundRect(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) < 300 || contourArea(contours[i]) > 1500){  //这里的值是根据实际情况做出处理 剔除一些明显不是的轮廓
				continue;
			}
			boundRect[i] = boundingRect(Mat(contours[i]));
			if (boundRect[i].width*1.0 / (boundRect[i].height*1.0) < 1.2 || boundRect[i].width *1.0 / (boundRect[i].height*1.0)> 2.8)   //限制宽高比来进一步确定点 if语句内的值也可以根据实际改变
				continue;
			//drawContours(input_frame, contours, i, Scalar(0.0, 255), 2);
			//rectangle(int_frame, boundRect[i], Scalar(255, 0, 0));
				/*******这里我将一整张图片从中间切开成两张处理，但不显示出来******/

			if ((boundRect[i].tl().x+boundRect[i].br().x)/2< int_frame.cols / 2) //c changed to the average
				leftRect.push_back(boundRect[i]);
			else
				rightRect.push_back(boundRect[i]);
		}
		groupRectangles(leftRect, 1, 0.2);
		groupRectangles(rightRect, 1, 0.2);
		
		/*Mat drawing = Mat::zeros(int_frame.size(), CV_8UC3);
		for (int i = 0; i < rightRect.size(); i++)
		{
			rectangle(drawing, rightRect[i], Scalar(0, 255, 0), 2);
			
		}
		for(int i=0; i<leftRect.size(); i++){
			rectangle(drawing, leftRect[i], Scalar(0, 255, 0), 2);
		}
		imshow("drawing", drawing);*/
		
		if(leftRect.size() >= 5 && rightRect.size()>= 5) {
			//cout << leftRect.size() << " " << rightRect.size() << endl;
			vector<Rect> left_rect;
			vector<Rect> right_rect;
			bool find_left = true;
		//c, should be recovered
		//	if (leftRect.size() < 5 && rightRect.size() < 5)
		//		return false;

			/*判断左侧的每一个矩形之间的左上坐标的x的误差值*/
			//cout << "More" << endl;
			int rectCounter = 0;
			for (int i = 0; i < leftRect.size(); i++)
			{
				for (int j = 0; j < leftRect.size(); j++){
					if(i==j)
						continue;
					if (abs(leftRect[i].tl().x - leftRect[j].tl().x) < 0.4*leftRect[i].width && leftRect[i].height - leftRect[j].height <0.2*leftRect[i].height
					&& leftRect[i].width - leftRect[i].width < 0.2 * leftRect[i].width )
						rectCounter ++;
				}
					
				if (rectCounter > 3)
					left_rect.push_back(leftRect[i]);
				rectCounter = 0;
			}

			/*for (int i = 0; i < left_rect.size(); i++)
			{
				rectangle(drawing, left_rect[i], Scalar(255, 0, 0), 2);
			}*/
		//	imshow("drawing", drawing);
			for (int i = 0; i < rightRect.size(); i++)
			{
				for (int j = 0; j < rightRect.size(); j++){
					if(i==j)
						continue;
					if (abs(rightRect[i].tl().x - rightRect[j].tl().x) < 0.4*rightRect[i].width && rightRect[i].height - rightRect[j].height <0.2*rightRect[i].height
					&& rightRect[i].width - rightRect[i].width < 0.2 * rightRect[i].width )
						rectCounter ++;
				}
					
				if (rectCounter > 3){
					right_rect.push_back(rightRect[i]);
					//cout << rightRect[i].x << " " << rightRect[i].y << endl;
				}
				rectCounter = 0;
			}
				
				/*判断右侧的每一个矩形之间的左上坐标的x的误差值*/
				/*bool find_right = true;
				for (int i = 0; i < rightRect.size() - 1; i++)
				{

					for (int j = 1; j < rightRect.size(); j--)
					{

						if (abs(rightRect[i].tl().x - rightRect[j].tl().x) > 10)
							find_right = false;
					}
					if (find_right)
						right_rect.push_back(rightRect[i]);
					find_right = true;
				}*/	

			
			

			

			/******这个函数是给师姐做的数码管识别做的 确定现在达到第几个数字******/
//			bool change = judge_which_number(copy_frame(right_rect[0]));
			//cout << change << endl;
			//cout << right_rect.size() << " " << left_rect.size() << endl;
			if(right_rect.size() ==5 && left_rect.size()==5){
				//cout << "find left and right" << endl;
				//高度设定为最低矩形的右下点到最高矩形的左上点的y的差 （以右侧的为准）
				//c, adjusted into the max and min of two
				int height = (abs(max(right_rect[0].br().y, left_rect[0].br().y) - min(right_rect[right_rect.size()-1].tl().y, left_rect[left_rect.size() - 1].tl().y)));
				//int height = (abs(max(right_rect[0].br().y, left_rect[0].br().y) - min(right_rect[right_rect.size()-1].tl().y, right_rect[right_rect.size() - 1].tl().y))\
					+ 2*right_rect[0].height); 
				int y =  min(right_rect[right_rect.size()-1].tl().y, left_rect[left_rect.size() - 1].tl().y);

				/*判断设定的高度加上起始y的值是否超过图片的宽度*/
				if( y - right_rect[0].height < 0 || (height + y) > copy_frame.rows){
					return false;
				} //c
				
				int start_x=1000, start_y=1000; //the starting point of roi_rect
				/*for (int i=0; i<left_rect.size(); i++) {
					if (start_x>left_rect[i].br().x) {
						start_x=left_rect[i].br().x;
					}
				}*/
				start_x = left_rect[left_rect.size()-1].br().x;
				
				start_y = min(left_rect[left_rect.size()-1].tl().y, right_rect[right_rect.size() - 1].tl().y);
				start_x += left_rect[0].width*0.5;
				start_y -= left_rect[0].height*0.5;

				/****这个矩形内的图片就是我要进行模板匹配的图片 这个矩形的范围可以在rectangle窗口自己观察****************/
				int roi_height = height + left_rect[0].height;
				int roi_width = right_rect[0].tl().x - left_rect[0].br().x - left_rect[0].width;
				roi_rect = Rect(start_x, start_y , roi_width ,roi_height);

				//cout <<left_rect[0].br().y << endl;
				//cout << right_rect[0].height << endl;
				//cout << abs(right_rect[0].br().y - right_rect[right_rect.size() - 1].tl().y) << endl;
				//cout << left_rect[left_rect.size() - 1].tl().y << endl;
				//rectangle(drawing, roi_rect, Scalar(0, 0, 255), 5);
				//cout << "yes" <<endl;
				if(start_x >= 0 && start_y >= 0 && start_x+ roi_width < copy_frame.size().width && start_y + roi_height < copy_frame.size().height)
				{
					fire_roi = copy_frame(roi_rect);
					//cvtColor(roi, roi, CV_BGR2GRAY); 
					threshold(fire_roi, fire_roi, 245, 255, THRESH_BINARY);
					imshow("fireRoi",fire_roi);
					if (!fire_roi.data)
						return false;

					/*********第二个参数就是我要找的大神符数字************/
					/*Point targetCenter = match_number(roi, 9);
					targetCenter.x += roi_rect.x;
					targetCenter.y += roi_rect.y;
					//cout << roi_rect.size() << endl;		
					imshow("roi", roi);
					if(targetCenter!= Point(-1,-1)){
						line(copy_frame, {targetCenter.x - 10, targetCenter.y }, { targetCenter.x + 10, targetCenter.y }, CV_RGB(0, 0, 255), 2);
						line(copy_frame, {targetCenter.x, targetCenter.y - 10 }, { targetCenter.x, targetCenter.y + 10 }, CV_RGB(0, 0, 255), 2);
					}
					imshow("Target", copy_frame);*/
					//imshow("rectangle",drawing);
					/*for (int i=0; i<3; i++) {
						for (int j=0; j<3; j++) {
							cut(roi, i, j);
						}
					}*/
				}
				else {
					return false;
				}
				//imshow("dst", dst);
			}
			else{
				//cout << "not find left and right" << endl;
				return false;
			}
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
	return true;
}
void initializeTemplates()
{
	for(int i=0; i<9; i++){
		templates[i] = imread(to_string(i) + "test.jpg");
	}
}

