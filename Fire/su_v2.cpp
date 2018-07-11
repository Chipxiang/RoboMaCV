#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <cmath>

using namespace std;
using namespace cv;

/*
 * thoughts about the program
 * find the 10 blocks (5 in each side): 
 */
 

bool usingCam = true;
int detectedNum;

bool findRange (Mat input_image);


int main(){
	//thread th[ThreNum];
	cout << "Initializing..." << endl;
	VideoCapture capture;
	
	// check whether video / camera opened
	if (usingCam){
		capture.open(1);
		if (!capture.isOpened()){
			cout << "Could not open the camera. " << endl;
			return -1;
		}
		else{
			cout << "Camera successfully opened. " << endl;
		}
	}
	else{
		//cout << "Enter Video Name: " << endl;
		string inputVideo;
		//cin >> inputVideo;
		inputVideo = "nine.mp4";
		//inputVideo = "Rune.avi";
		capture.open(inputVideo);
		if (!capture.isOpened()){
			cout << "Could not open the video. " << endl;
			return -1;
		}
		else{
			cout << "Video successfully opened. " << endl;
		}
	}
	
	//cout << "enter the number you want to detect: ";
	//cin >> detectedNum;
	
	Mat frame;
	while (true){
		capture >> frame;
		if (!frame.data){
			break;
		}
		cout << frame.size() << endl;
		// do the cutting here
		findRange(frame);
		
		waitKey(1);
	}
	
	return 0;
}

bool findRange (Mat input_image){
	Mat copied_image, drawing;
	input_image.copyTo(copied_image); 
	imshow("raw image", input_image);
	
	/**********drawing: to show the rectangles and the range************/
	drawing = Mat::zeros(input_image.size(), CV_8UC3);
	
	/**********pre-process of image************/
	cvtColor(input_image, input_image, CV_BGR2GRAY);
	imshow("image after cvtColor", input_image);
	threshold(input_image, input_image, 50, 255, THRESH_BINARY | THRESH_OTSU); // try to figure out a proper number for threshold
	imshow("THRESH", input_image);
	blur(input_image, input_image, Size(3,3));
	Canny(input_image, input_image, 120, 240);
	imshow("CANNY", input_image);
	
	/*************findContours to recognize all rectangles***********************/
	vector<vector<Point> > contours;
	//vector<Vec4i> hierarchy;
	findContours(input_image, contours,/* hierarchy,*/ RETR_TREE, CHAIN_APPROX_SIMPLE);
	vector<Rect> boundRect(contours.size());
	// show all the rectangles
	if (contours.size() > 0){
		//stringstream ss;
		for (int i=0; i<contours.size(); i++){
			//ss << i;
			drawContours(drawing, contours, i, Scalar(0.0, 255), 2);
			boundRect[i] = boundingRect(Mat(contours[i])); // important step!
			//rectangle(drawing, boundRect[i], Scalar(0.0, 255), 2);
			//cout << "contour area: " << contourArea(contours[i]) << endl;
		}
		imshow("rectangle", drawing);
	}
	else{
		return false;
	}
	
	/****************select the needed rectangles********************/
	vector<Rect> selectedRect;
	vector<Rect> leftRect;
	vector<Rect> rightRect;
	Mat drawing2 = Mat::zeros(input_image.size(), CV_8UC3);
	
	//first step: size
	if (boundRect.size() > 0){
		for (int i=0; i<boundRect.size(); i++){
			if (boundRect[i].area() > 300 
				&& boundRect[i].area() < 1500 
				&& boundRect[i].width / boundRect[i].height > 0.8 // why not 1.0???
				&& boundRect[i].width / boundRect[i].height < 2.5){
				selectedRect.push_back(boundRect[i]);
				rectangle(drawing2, boundRect[i], Scalar(0, 0, 255));
				cout << "selected contour area: " << contourArea(contours[i]) << endl;
			}
		}
		imshow ("selected rectangle", drawing2);
	}
	else{
		return false;
	}
	
	// second step: find 5 rectangles --> cut rectangles into left and right
	if (selectedRect.size() > 0){
		for (int i=0; i<selectedRect.size(); i++){
			if (selectedRect[i].tl().x < input_image.cols / 2){
				leftRect.push_back(selectedRect[i]);
			}
			else {
				rightRect.push_back(selectedRect[i]);
			}
		}
	}
	
	// third step: find the left 5 rectangles
	vector<Rect> selectedLeft;
	Mat drawing3 = Mat::zeros(input_image.size(), CV_8UC3);
	if (leftRect.size() > 0){
		cout << "###" << "leftRect.size: " << leftRect.size() << endl;
		for (int i=0; i < leftRect.size(); i++){
			selectedLeft.push_back(leftRect[i]);
			cout << "i: " << i << endl;
			for (int j=i+1; j < leftRect.size(); j++){
				if (abs ( leftRect[i].height - leftRect[j].height ) < 5 
					&& abs ( leftRect[i].width / leftRect[j].width ) < 5
					&& abs ( leftRect[i].tl().x - leftRect[j].tl().x ) < 5 
					&& abs ( leftRect[i].br().x - leftRect[j].br().x ) < 5
					&& abs ( leftRect[i].br().y - leftRect[j].br().y ) > leftRect[i].height){
					selectedLeft.push_back(leftRect[j]);
					cout << "j: " << j << endl;
				}
			}
			for (int j=0; j<selectedLeft.size(); j++){
				for (int k=j+1; k<selectedLeft.size(); k++){
					if (abs ( leftRect[i].br().y - leftRect[j].br().y ) < 5){
						
					}
				}
			}
			
			if (selectedLeft.size() >= 3 && selectedLeft.size() <= 5){
				for (int j=0; j<selectedLeft.size(); j++){
					rectangle(drawing3, selectedLeft[j], Scalar(0, 0, 225));
					cout << "**" << "selectedLeft[" << j << "].height: " << selectedLeft[j].height << endl;
					cout << "**" << "selectedLeft[" << j << "].width: " << selectedLeft[j].width << endl;
					cout << "**" << "selectedLeft[" << j << "].tl().x: " << selectedLeft[j].tl().x << endl;
					cout << "**" << "selectedLeft[" << j << "].tl().y: " << selectedLeft[j].tl().y << endl;
					cout << "**" << "selectedLeft[" << j << "].br().x: " << selectedLeft[j].br().x << endl;
					cout << "**" << "selectedLeft[" << j << "].br().y: " << selectedLeft[j].br().y << endl;
				}
				cout << "testing, selectedLeft not cleared, selectedLeft.size: " << selectedLeft.size() << endl;
				break;
			}
			else {
				selectedLeft.clear();
				cout << "testing, selectedLeft cleared, selectedLeft.size: " << selectedLeft.size() << endl;
			}
			//imshow("left selected rec", drawing3);
			//waitKey(0);
		}
		imshow("left selected rec", drawing3);
		waitKey(0);
	}
	
	return true;
}
