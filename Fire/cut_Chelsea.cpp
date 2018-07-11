#include <opencv2/opencv.hpp>
#include <sstream>
#include <algorithm>

/************************************************************************


	主要自己改的地方有三点 1.视频素材的路径 2.模板储存的位置及名字 3. 剔除轮廓的大小
	在模板匹配时 如果你的图片以及刚刚好了 可以把金字塔降采样去掉或者减少次数
	该代码缺少一个数码管识别，已经写好的那个判断打到第几个是没写完的（原因是没有成功打过大神符的视频 当然也可以写下去
	只是可能有点坑，最近学Python没空续写）
	模板匹配学过opencv都会 如果要拿模板匹配去打比赛，减少运算量是首要的，我感觉模板匹配有点Low。。。

**************************************************************************/
using namespace std;
using namespace cv;

bool usingCam = true;
Point match_number(Mat, int);
void pD(Mat *src);
bool findRange(Mat);
bool judge_which_number(Mat judge);
void cut(Mat roi, int i, int j);
void findRed();
double self_min(double a, double b, double c);
double self_max(double a, double b, double c);
//bool findRed();
Mat dst;
Mat roi;
Mat drawing;
Mat frame;
Mat copy_frame;
Mat templates[9];
Rect roi_rect;
bool foundRange = false;
bool foundLED;
int main()
{
	VideoCapture capture;
	//Mat frame;
	const int VideoWidth = 640;
	const int VideoHeight = 480;
	
	if(!usingCam) {
		capture.open("./nine.mp4");   /*视频素材*/
		if (!capture.isOpened()){
			cout << "Could not open the video. " << endl;
			return -1;
		}
		else{
			cout << "Video successfully opened. " << endl;
		}
	}
	else {
		capture.open(1);
//		capture.set(CV_CAP_PROP_FRAME_WIDTH, VideoWidth);
//		capture.set(CV_CAP_PROP_FRAME_HEIGHT, VideoHeight);
//		capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
		if (!capture.isOpened()){
			cout << "Could not open the camera. " << endl;
			return -1;
		}
		else{
			cout << "Camera successfully opened. " << endl;
		}
	}
	
	for(int i=0; i<9; i++){
		templates[i] = imread(to_string(i) + "test.jpg");
		//pD(&templates[i]);
	}
	/*vector<Point> cnt;
	Mat temp[9];
	for (int i=0; i<9; i++){
		templates[i] = imread(to_string(i+1) + ".jpg"); 
		pD(&templates[i]);
		templates[i].copyTo(temp[i]);
		threshold(temp[i], temp[i], 185, 255, THRESH_BINARY);
		cvtColor( templates[i], templates[i], CV_BGR2GRAY );
		
		threshold(templates[i], templates[i], 185, 255, THRESH_BINARY);
		vector<vector<Point>> contours;
		findContours(templates[i], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		//CvScalar colour;
		//colour = CV_RGB(0, 0, 255);
		//rectangle(test, boundingRect(contours[0]), colour);
		//imwrite(to_string(i) + "testCont.jpg",test);
		//Mat resized, reversed, processed;
		int cntNum = 0;
		for (int j=0; j<contours.size(); j++){
			if(contours[j].size()>contours[cntNum].size()){
				cntNum = j;
			}
		}
		if(i==0){
			cnt = contours[cntNum];
		}
		else{
			if(contours[cntNum].size()>cnt.size()){
				cnt = contours[cntNum];
				cout << i << endl;
			}
		}
		
	}
	for(int i=0;i<9;i++){
		Rect rect = boundingRect(cnt);
		rect.height += 8;
		rect.width += 8;
		rect.x -=4;
		rect.y -=4;
		Mat roi = temp[i](rect);
		
		//resize(roi, resized, Size(28,28),INTER_CUBIC);
		
		//resize(test, resized, Size(28,28),INTER_CUBIC);
		//threshold(resized, reversed, 150, 255, THRESH_BINARY);
		imwrite(to_string(i) + "test.jpg",roi);
	}*/
	
	while (true)
	{
		capture >> frame;
		if (frame.empty()){
			cout << "Video has been read out.";
			break;
		}
		if(!usingCam)
			resize(frame, frame, Size(640, 360), 0, 0, INTER_CUBIC);

		if (!frame.data)
		{
			break;
		}
		//cout <<frame.size() << endl;
		//match_number(frame, 4);
		foundRange = findRange(frame);
		cout << foundRange << endl;
		findRed();
		imshow("frame", frame);
		waitKey(1);
	}
	return 0;
}



/*缩小模板匹配范围并将左右两侧的5个判断位置找出来*/
/*这个处理方法感觉比较low 其实这个算法可以自己写 看自己是否能找到更加好的算法 我的代码写得很乱还没处理干净
	因此可以跳过这段代码 知道思路就好*/
bool findRange(Mat int_frame)
{
	cout << "enter find range" << endl;
	//dst = Mat::zeros(int_frame.size(), CV_8UC3);
	/*drawing 画布 将一些画矩形的操作在这个窗口输出*/
	//drawing = Mat::zeros(int_frame.size(), CV_8UC3);
	//copy = int_frame.clone();
	/* copy_frame 备份*/
	int_frame.copyTo(copy_frame);
	//imshow("in", copy_frame); //copy_frame is the original frame
	/****************灰度 二值化******************/
	cvtColor(int_frame, int_frame, CV_BGR2GRAY); //int_frame is the converted frame
	threshold(int_frame, int_frame, 50, 255, THRESH_BINARY | THRESH_OTSU); //two flags determine the best threashold for binarization
	
	Canny(int_frame, int_frame, 120, 240); //这里用Canny的效果还不错
	//imshow("THRESH", int_frame); //int_frame is the binarized image
	/**************开始找两侧的白色灯光*******************/
	vector<vector<Point> > contours;
	vector<Rect> rightRect; //all the contours on the right half
	vector<Rect> leftRect;
	
	findContours(int_frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	if (contours.size()>1) {
		//cout << "come on:"<<contours.size()<<endl;
		vector<Rect> boundRect(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) < 222 || contourArea(contours[i]) > 666){  //这里的值是根据实际情况做出处理 剔除一些明显不是的轮廓
				continue;
			}
			boundRect[i] = boundingRect(Mat(contours[i]));
			if (boundRect[i].width / boundRect[i].height<1.0 || boundRect[i].width / boundRect[i].height>2.5)   //限制宽高比来进一步确定点 if语句内的值也可以根据实际改变
				continue;
	//			drawContours(drawing, contours, i, Scalar(0.0, 255), 2);
			//rectangle(int_frame, boundRect[i], Scalar(255, 0, 0));
				/*******这里我将一整张图片从中间切开成两张处理，但不显示出来******/

			if ((boundRect[i].tl().x+boundRect[i].br().x)/2< int_frame.cols / 2) //c changed to the average
				leftRect.push_back(boundRect[i]);
			else
				rightRect.push_back(boundRect[i]);
		}

		if(leftRect.size()> 3 && rightRect.size()>3) {
			vector<Rect> left_rect;
			vector<Rect> right_rect;
			bool find_left = true;
		//c, should be recovered
		//	if (leftRect.size() < 5 && rightRect.size() < 5)
		//		return false;

			/*判断左侧的每一个矩形之间的左上坐标的x的误差值*/
			for (int i = 0; i < leftRect.size() - 1; i++)
			{
				for (int j = 1; j < leftRect.size(); j--)
					if (abs(leftRect[i].tl().x - leftRect[j].tl().x) > 10)
						find_left = false;
				if (find_left)
					left_rect.push_back(leftRect[i]);
				find_left = true;
			}

			/*for (int i = 0; i < left_rect.size(); i++)
			{
				rectangle(drawing, left_rect[i], Scalar(255, 0, 0), 2);
			}*/
		//	imshow("drawing", drawing);

			
			/*判断右侧的每一个矩形之间的左上坐标的x的误差值*/
			bool find_right = true;
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
			}
			/*for (int i = 0; i < right_rect.size(); i++)
			{
				rectangle(drawing, right_rect[i], Scalar(0, 255, 0), 2);
			}
			imshow("drawing", drawing);*/

			/******这个函数是给师姐做的数码管识别做的 确定现在达到第几个数字******/
//			bool change = judge_which_number(copy_frame(right_rect[0]));
			//cout << change << endl;
			
			//changed by mona
			if(find_left && find_right){
				cout << "find left and right" << endl;
				//高度设定为最低矩形的右下点到最高矩形的左上点的y的差 （以右侧的为准）
				//c, adjusted into the max and min of two
				int height = (abs(max(right_rect[0].br().y, left_rect[0].br().y) - min(right_rect[right_rect.size()-1].tl().y, right_rect[right_rect.size() - 1].tl().y)));
				//int height = (abs(max(right_rect[0].br().y, left_rect[0].br().y) - min(right_rect[right_rect.size()-1].tl().y, right_rect[right_rect.size() - 1].tl().y))\
					+ 2*right_rect[0].height); 
				int y = left_rect[left_rect.size() - 1].tl().y;

				/*判断设定的高度加上起始y的值是否超过图片的宽度*/
				y = (y - right_rect[0].height) < 0 ? 0 : (y - right_rect[0].height); //c
				(height + y) > copy_frame.rows ? (height = copy_frame.rows - y) : height; //c
				
				int start_x=1000, start_y=1000; //the starting point of roi_rect
				for (int i=0; i<left_rect.size(); i++) {
					if (start_x>left_rect[i].br().x) {
						start_x=left_rect[i].br().x;
					}
				}

				start_y=min(left_rect[left_rect.size()-1].tl().y, right_rect[right_rect.size() - 1].tl().y);
				start_x+= (right_rect[right_rect.size()-1].br().x-right_rect[right_rect.size()-1].tl().x)*0.645;
				start_y-=(right_rect[right_rect.size()-1].br().x-right_rect[right_rect.size()-1].tl().x)*0.03125;

				/****这个矩形内的图片就是我要进行模板匹配的图片 这个矩形的范围可以在rectangle窗口自己观察****************/
				int roi_height = height+(right_rect[right_rect.size()-1].br().x-right_rect[right_rect.size()-1].tl().x)*0.03125;
				roi_rect = Rect(start_x*0.95, start_y*0.95,roi_height*1.1*1.7,roi_height*1.1);

				//cout <<left_rect[0].br().y << endl;
				//cout << right_rect[0].height << endl;
				//cout << abs(right_rect[0].br().y - right_rect[right_rect.size() - 1].tl().y) << endl;
				//cout << left_rect[left_rect.size() - 1].tl().y << endl;
				//rectangle(drawing, roi_rect, Scalar(0, 0, 255), 5);
				//cout << "yes" <<endl;
				if(start_x >= 0 && start_y >= 0 && start_x+ roi_height*1.7*1.1< copy_frame.size().width && start_y + roi_height*1.1 < copy_frame.size().height)
				{
					
					roi = copy_frame(roi_rect);
					//cvtColor(roi, roi, CV_BGR2GRAY); 
					threshold(roi, roi, 200, 255, THRESH_BINARY);
					if (!roi.data)
						return false;

					/*********第二个参数就是我要找的大神符数字************/
					Point targetCenter = match_number(roi, 9);
					targetCenter.x += roi_rect.x;
					targetCenter.y += roi_rect.y;
					//cout << roi_rect.size() << endl;		
					imshow("roi", roi);
					if(targetCenter!= Point(-1,-1)){
						line(copy_frame, {targetCenter.x - 10, targetCenter.y }, { targetCenter.x + 10, targetCenter.y }, CV_RGB(0, 0, 255), 2);
						line(copy_frame, {targetCenter.x, targetCenter.y - 10 }, { targetCenter.x, targetCenter.y + 10 }, CV_RGB(0, 0, 255), 2);
					}
					imshow("Target", copy_frame);
					//imshow("rectangle",drawing);
					/*for (int i=0; i<3; i++) {
						for (int j=0; j<3; j++) {
							cut(roi, i, j);
						}
					}*/
				}
				else {
					cout << "size exeeded" <<endl;
					return false;
				}
				//imshow("dst", dst);
			}
			else{
				cout << "not find left and right" << endl;
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


/*void cut(Mat roi, int i, int j) {
	int n=3*i+j;
	int height = roi.size().height;
	int width = roi.size().width;
	string writePic;
	vector<int> compression;
	compression.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression.push_back(95);
	cout << "yes"<<endl;
	if(j*width*0.3627 >= 0 && i*height*0.3666 >= 0 && width*0.2745 + j*width*0.3627 < width && height*0.2667 + i*height*0.3666 < height)
	{
		Rect cut_roi = Rect(j*width*0.3627, i*height*0.3666,width*0.2745,height*0.2667);
		cout << "no"<<endl;
		writePic="write" + to_string(n) + ".jpg";
		cout << writePic<<endl;
		Mat roi_num = roi(cut_roi);
		cout << "daobuliao" <<endl;
		try {
			bool write=imwrite(writePic, roi_num, compression);
//				bool write=imwrite(writePic, roi_num);
			if (!write)
				cout << "write failed."<<endl;
		} catch (runtime_error& ex) {
			cout << stderr << ex.what() << endl;
			return;
		}
	}
	else {
		cout << "the size exceeded"<<endl;
		return;
	}
}*/


/*在库里面找到图片并进行模板匹配，第二个参数是输入的数字，及第N个数码管的数字*/
Point match_number(Mat find_roi, int number)
{
	string findpicture;
	Point maxLoc, minLoc;
	double minVal, maxVal;
	/*Mat roiBin;
	find_roi.copyTo(roiBin);
	cvtColor(roiBin, roiBin, CV_BGR2GRAY);
	threshold(roiBin, roiBin, 180, 255, THRESH_BINARY);
	//Canny(roiBin, roiBin, 120, 240); //这里用Canny的效果还不错
	vector<vector<Point>> contours;
	findContours(roiBin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//Mat resized, reversed, processed;
	int cntNum = 0;
	for (int j=0; j<contours.size(); j++){
		if(contours[j].size()>contours[cntNum].size()){
			cntNum = j;
		}
	}
	//30.0/boundingRect(contours[cntNum]).width * find_roi.cols
	//40.0/boundingRect(contours[cntNum]).height * find_roi.rows
	*/
	Mat dst = templates[number-1];
	if (!dst.data)
	{
		return Point(-1,-1);
	}
	//cout << boundingRect(contours[cntNum]).width << " " << boundingRect(contours[cntNum]).height << endl;
	//cout << find_roi.rows << " " << find_roi.cols << endl;
	//resize(dst, dst, Size(boundingRect(contours[cntNum]).width/30.0 * dst.cols,boundingRect(contours[cntNum]).width/30.0 * dst.cols*(4.0/3.0)), 0, 0, INTER_CUBIC);
	//rectangle(find_roi,boundingRect(contours[cntNum]), Scalar(0, 0, 255), 1);
	//imshow("binRoi", roiBin);
	//imshow("Roi", find_roi);
	resize(dst, dst, Size(find_roi.cols/9.8,find_roi.cols/9.8*(4.0/3.0)), 0, 0, INTER_CUBIC);
	
	imshow("Tst", dst);
	//waitKey(0);
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
	cout << "中心点："<<point << endl;
	return point;
}

	/*输入的图片过大，用金字塔处理3次*/
void pD(Mat *src) 
{
	pyrDown(*src, *src);
	pyrDown(*src, *src);
	pyrDown(*src, *src);
	//pyrDown(*src, *src);
}

void findRed(){
	
	if (foundRange){
		cout << "***************" << endl;
		/*for(int i=0; i<9; i++){
			cout << i << "th sudoku" << endl;
			cout << sudoku_rects[i].center.x << " " << sudoku_rects[i].center.y << endl;
			cout << endl;
		}*/
		cout << roi_rect.x << "," << roi_rect.y << endl;
		cout << roi_rect.width << " " << roi_rect.height << endl;
		int width = roi_rect.width*580/1140;
		int height = roi_rect.height*180/710;
		cout << width << " " << height << endl;
		/*Rect sudoku_rects[9];
		for (int i=0; i<3; i++){
			for (int j=0; j<3; j++){
				sudoku_rects[8-i*3-j] = Rect(i*row + roi_rect.x, j*col + roi_rect.y, row, col);
			}
		}
		cout << "######################" << endl;
		int ledBuffer[5] = {0};*/
		//int xa,xb,xc,xd,xred;
		//xc = sudoku_rects[8].width/2+sudoku_rects[8].x + (112.0/370.0)*(sudoku_rects[7].width/2+sudoku_rects[7].x - (sudoku_rects[8].width/2+sudoku_rects[8].x)) * ((sudoku_rects[7].width/2+sudoku_rects[7].x - (sudoku_rects[8].width/2+sudoku_rects[8].x))/(sudoku_rects[6].width/2+sudoku_rects[6].x - (sudoku_rects[7].width/2+sudoku_rects[7].x)));
		//xd = sudoku_rects[8].width/2+sudoku_rects[8].x + ((112.0+520.0)/370.0)*(sudoku_rects[7].width/2+sudoku_rects[7].x - (sudoku_rects[8].width/2+sudoku_rects[8].x)) * ((sudoku_rects[7].width/2+sudoku_rects[7].x - (sudoku_rects[8].width/2+sudoku_rects[8].x))/(sudoku_rects[6].width/2+sudoku_rects[6].x - (sudoku_rects[7].width/2+sudoku_rects[7].x)));
		//xred = (104.0/370.0)*(sudoku_rects[7].center.x - sudoku_rects[8].center.x) * ((sudoku_rects[7].center.x - sudoku_rects[8].center.x)/(sudoku_rects[6].center.x - sudoku_rects[7].center.x));
		//int ya,yb,yc,yd;
		//ya = sudoku_rects[8].height/2+sudoku_rects[8].y - (276.0/220.0)*(sudoku_rects[5].height/2+sudoku_rects[5].y - (sudoku_rects[8].height/2+sudoku_rects[8].y)) * ((sudoku_rects[5].height/2+sudoku_rects[5].y - (sudoku_rects[8].height/2+sudoku_rects[8].y))/(sudoku_rects[2].height/2+sudoku_rects[2].y - (sudoku_rects[5].height/2+sudoku_rects[5].y)));
		//yb = sudoku_rects[6].height/2+sudoku_rects[6].y - (276.0/220.0)*(sudoku_rects[3].height/2+sudoku_rects[3].y - (sudoku_rects[6].height/2+sudoku_rects[6].y)) * ((sudoku_rects[3].height/2+sudoku_rects[3].y - (sudoku_rects[6].height/2+sudoku_rects[6].y))/(sudoku_rects[0].height/2+sudoku_rects[0].y - (sudoku_rects[3].height/2+sudoku_rects[3].y)));
		//yc = sudoku_rects[8].height/2+sudoku_rects[8].y - (152.2/220.0)*(sudoku_rects[5].height/2+sudoku_rects[5].y - (sudoku_rects[8].height/2+sudoku_rects[8].y)) * ((sudoku_rects[5].height/2+sudoku_rects[5].y - (sudoku_rects[8].height/2+sudoku_rects[8].y))/(sudoku_rects[2].height/2+sudoku_rects[2].y - (sudoku_rects[5].height/2+sudoku_rects[5].y)));
		//yd = sudoku_rects[6].height/2+sudoku_rects[6].y - (152.2/220.0)*(sudoku_rects[3].height/2+sudoku_rects[3].y - (sudoku_rects[6].height/2+sudoku_rects[6].y)) * ((sudoku_rects[3].height/2+sudoku_rects[3].y - (sudoku_rects[6].height/2+sudoku_rects[6].y))/(sudoku_rects[0].height/2+sudoku_rects[0].y - (sudoku_rects[3].height/2+sudoku_rects[3].y)));
		//cout << "xc = " << xc << endl;
		//cout << "xd = " << xd << endl;
		//cout << "ya = " << ya << endl;
		//cout << "yb = " << yb << endl;
		//cout << "yc = " << yc << endl;
		//cout << "yd = " << yd << endl;
		//cout << "xred = " << xred << endl;
		//cout << endl;

		/*xc = xc * 0.98;
		ya = ya * 0.98;
		yb = yb * 0.98;
		xd = xd * 1.02;
		yc = yc * 1.02;
		yd = yd * 1.02;*/
		int xa, ya, xd, yd;
		xa = roi_rect.x + roi_rect.width*280/1140;
		xd = xa + roi_rect.width*(300+560)/1140;
		ya = roi_rect.y - roi_rect.height*190/710;
		yd = ya + roi_rect.height*180/710;
		cout << xa << " " << ya << endl;
		cout << " " << xd << " " << yd << endl;
		//cout << "######################" << endl;
		if (xa<0 || xa > roi_rect.width + roi_rect.x ||
			ya<0 || ya > roi_rect.y ||
			xd<0 || xd > roi_rect.width + roi_rect.x ||
			yd<0 || yd > roi_rect.y ){
				foundLED = false;
				cout << "sudoku position wiered" << endl;
			}
		else{
			cout << "######################" << endl;
			foundLED = true;
			Mat redNum;
			Mat redRaw;
			Rect rect1(xa, ya, width, height);
			Mat binaryLED;
			threshold(copy_frame, binaryLED, 200, 255, THRESH_BINARY);
			binaryLED(rect1).copyTo(redRaw);
			imshow("redRaw", redRaw);
			//threshold(redBoard, redRev, 150, 255, THRESH_BINARY_INV);

			
			
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
			/*resize(redRaw, redRaw, Size(312,86),INTER_CUBIC);
			Mat element = getStructuringElement(CV_SHAPE_ELLIPSE, Size(3, 3));
			//erode(redRaw, redBoard,element, Point(-1, -1), 1, 1, 1);
			dilate(redRaw, redBoard,element, Point(-1, -1), 1, 1, 1);
			//imshow("Board",redBoard);
			vector<vector<Point>> contours;
			findContours(redBoard, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			vector<vector<Point>> digitCnt;
			Rect ledRect;
			sort(contours.begin(), contours.end(), contour_sorter());
			int j=0;
			for (int i = 0; i < contours.size(); i++){
				if(contours[i].size() > 5){
					ledRect = boundingRect(contours[i]);
					//rectangle(redBoard, ledRect, colour);
					digitCnt.push_back(contours[i]);
					//cout << contours[i] << endl;
					j++;
					if (j>5){
						foundLED = false;
						break;
					}
				}
			}
			if (j!=5){
				foundLED = false;
			}
			if(foundLED){
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
						dilate(roi0, roi0,element, Point(-1, -1), 2, 1, 1);
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
						
						for (int i = roi0.rows-6; i >=0; --i)
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
						dst_vertices[0] = Point(0, 5);
						dst_vertices[1] = Point(roi0.cols-1, roi0.rows-6);
						dst_vertices[2] = Point(roi0.cols-1, 5);
					  
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
							if(total*1.0/(area*1.0)>0.6){
								on[j] = 1;
								rectangle(test, segRect, colour);
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
						ledDigits[i] = y+1;
						//cout << ledRect.x << " " << ledRect.y << " " << ledRect.width << " " << ledRect.height << " " << endl << endl;
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
			}
			
			//imshow("redBoard",redBoard);

			//imwrite("Board.jpg", redRev);
			waitKey(1);
			
			
			//imshow("binary",binary);
			//waitKey(1);
		//}
		//end of adding
	}
			
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



	/*判断到第几个数字*/
/*bool judge_which_number(Mat judge)
{
	Mat judgebgr[3];
	Mat judgeb;
	split(judge, judgebgr);
	threshold(judgebgr[0], judgeb, 220, 255, THRESH_BINARY);
	imshow("judgeb",judgeb);
	vector<vector<Point> > contours;
	findContours(judgeb,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0)
		return false;
	vector<Rect> boundRect(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		boundRect[i] = boundingRect(contours[i]);
		if (boundRect[i].area() > judgeb.size().area() /2)
			return true;
	}
	return false;
}
*/
