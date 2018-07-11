#include <opencv2/opencv.hpp>
#include <sstream>





/************************************************************************


	主要自己改的地方有三点 1.视频素材的路径 2.模板储存的位置及名字 3. 剔除轮廓的大小
	在模板匹配时 如果你的图片以及刚刚好了 可以把金字塔降采样去掉或者减少次数
	该代码缺少一个数码管识别，已经写好的那个判断打到第几个是没写完的（原因是没有成功打过大神符的视频 当然也可以写下去
	只是可能有点坑，最近学Python没空续写）
	模板匹配学过opencv都会 如果要拿模板匹配去打比赛，减少运算量是首要的，我感觉模板匹配有点Low。。。

**************************************************************************/
using namespace std;
using namespace cv;

bool match_number(Mat, int);
void pD(Mat *src);
bool findRange(Mat);
bool judge_which_number(Mat judge);
//Mat dst;
Mat drawing;
Mat copy_frame;
int main()
{
	VideoCapture capture;
	capture.open("./nine.mp4");   /*视频素材*/
	Mat frame;
	
	while (true)
	{
		capture >> frame;
		if (!frame.data)
		{
			break;
		}
		cout <<frame.size() << endl;
		match_number(frame, 5);
		findRange(frame);
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

	
	//dst = Mat::zeros(int_frame.size(), CV_8UC3);
	/*drawing 画布 将一些画矩形的操作在这个窗口输出*/
	drawing = Mat::zeros(int_frame.size(), CV_8UC3);
	//copy = int_frame.clone();
	/* copy_frame 备份*/
	int_frame.copyTo(copy_frame);
	imshow("in", int_frame);
	/****************灰度 二值化******************/
	cvtColor(int_frame, int_frame, CV_BGR2GRAY);
	threshold(int_frame, int_frame, 20, 255, THRESH_BINARY | THRESH_OTSU);
	imshow("THRESH", int_frame);
	Canny(int_frame, int_frame, 120, 240);                 //这里用Canny的效果还不错

	/**************开始找两侧的白色灯光*******************/
	vector<vector<Point> > contours;
	findContours(int_frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

	vector<Rect> boundRect(contours.size());
	vector<Rect> rightRect;
	vector<Rect> leftRect;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) < 500 || contourArea(contours[i]) > 1500)           //这里的值是根据实际情况做出处理 剔除一些明显不是的轮廓
			continue;
		boundRect[i] = boundingRect(Mat(contours[i]));
		if (boundRect[i].width / boundRect[i].height<1.0 || boundRect[i].width / boundRect[i].height>2.5)   //限制宽高比来进一步确定点 if语句内的值也可以根据实际改变
			continue;
		drawContours(drawing, contours, i, Scalar(0.0, 255), 2);
		//rectangle(src, boundRect[i], Scalar(255, 0, 0));

		/*******这里我将一整张图片从中间切开成两张处理，但不显示出来******/
		if (boundRect[i].tl().x < copy_frame.cols / 2)
			leftRect.push_back(boundRect[i]);
		else
			rightRect.push_back(boundRect[i]);
	}

	vector<Rect> left_rect;
	vector<Rect> right_rect;
	bool find_left = true;


	if (leftRect.size() < 5 && rightRect.size() < 5)
		return false;

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

	for (int i = 0; i < left_rect.size(); i++)
	{
		rectangle(drawing, left_rect[i], Scalar(255, 0, 0), 2);
	}
	imshow("drawing", drawing);

	
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


	/******这个函数是给师姐做的数码管识别做的 确定现在达到第几个数字******/
	bool change = judge_which_number(copy_frame(right_rect[0]));
	//cout << change << endl;
	

	//高度设定为最低矩形的右下点到最高矩形的左上点的y的差 （以右侧的为准）
	int height = (abs(right_rect[0].br().y - right_rect[right_rect.size() - 1].tl().y + 2*right_rect[0].height)); 
	int y = left_rect[left_rect.size() - 1].tl().y;

	/*判断设定的高度加上起始y的值是否超过图片的宽度*/
	y = (y - right_rect[0].height) < 0 ? 0 : (y - right_rect[0].height);
	(height + y) > copy_frame.rows ? (height = copy_frame.rows - y) : height;


	/****这个矩形内的图片就是我要进行模板匹配的图片 这个矩形的范围可以在rectangle窗口自己观察****************/
	Rect roi_rect = Rect(left_rect[left_rect.size() - 1].br().x, y, \
		right_rect[right_rect.size() - 1].tl().x - left_rect[left_rect.size() - 1].br().x, \
		height);


	//cout <<left_rect[0].br().y << endl;
	//cout << right_rect[0].height << endl;
	//cout << abs(right_rect[0].br().y - right_rect[right_rect.size() - 1].tl().y) << endl;
	//cout << left_rect[left_rect.size() - 1].tl().y << endl;
	rectangle(drawing, roi_rect, Scalar(0, 0, 255), 5);

	Mat roi = copy_frame(roi_rect);
	if (!roi.data)
		return false;

	/*********第二个参数就是我要找的大神符数字************/
	match_number(roi, 8);
	//cout << roi_rect.size() << endl;
	
	
	imshow("rectangle",drawing);
	imshow("roi", roi);
	//imshow("dst", dst);

	
}



/*在库里面找到图片并进行模板匹配，第二个参数是输入的数字，及第N个数码管的数字*/
bool match_number(Mat find_roi, int number)
{
	string findpicture;
	Point maxLoc, minLoc;
	double minVal, maxVal;
	
	//adding
	stringstream ss;
	string intNum;
	ss << number;
	ss >> intNum;
	findpicture = "./" + intNum + ".jpg"; 
	//adding end
	  //预先放好模板 绝对路径自己改
	Mat dst = imread(findpicture);
	if (!dst.data)
	{
		return false;
	}
	pD(&dst);
	
	/******************确保模板大小小于roi********************/
	
	if (find_roi.rows < dst.rows || find_roi.cols < dst.cols)
		return false;
	Mat right_mat;
	matchTemplate(find_roi, dst, right_mat, 0);
	minMaxLoc(right_mat, &minVal, &maxVal, &minLoc, &maxLoc);
	//cout << minVal<< endl;
	Point point = Point(minLoc.x + dst.cols / 2, minLoc.y + dst.rows / 2);
	rectangle(find_roi, Rect(minLoc.x, minLoc.y, dst.cols, dst.rows), Scalar(0, 0, 255), 1);
	circle(find_roi, point, 2, Scalar(255, 0, 0), 2);
	cout << "中心点："<<point << endl;
	return true;
}

/*输入的图片过大，用金字塔处理3次*/
void pD(Mat *src) 
{
	pyrDown(*src, *src);
	pyrDown(*src, *src);
	pyrDown(*src, *src);
}


/*判断到第几个数字*/
bool judge_which_number(Mat judge)
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
