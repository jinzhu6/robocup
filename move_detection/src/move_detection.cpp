#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "image_converter.h" 
using namespace cv;
using namespace std;
const int tol = 10, w = 640, h = 480;
int num = 0;
Mat image_a(h, w, CV_8UC3), image_sum = Mat::zeros(h, w, CV_16UC3);
Vector<Rect> findregion(Mat image);
void move_detect(Mat image);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_detection");
	ros::NodeHandle nh;
	ImageConverter kinect_image(nh);  

	Mat image;
 //       namedWindow("原始图片");
	namedWindow("处理后的图像");
	while(ros::ok())
	{
		image = kinect_image.get_image(); 
/*		if(!image.empty())
		{
			cv::imshow("原始图片", image);	

		}  */	
       

		if(!image.empty())
		{

			move_detect(image);	
			cv::imshow("处理后的图像", image);	
		}

		ros::spinOnce();
	}
	        return 0;

}

Vector<Rect> findregion(Mat image)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat con;
	Vector<Rect> rects;
	findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	if (contours.size() > 0)
	{
		for (int idx = 0; idx >= 0; idx = hierarchy[idx][0])
		{
			Rect rect = boundingRect(contours[idx]);
			float rateofhw = float(rect.height) / float(rect.width);
			con = Mat::zeros(image.size(), CV_8UC1);
			if (rateofhw<4.&&rateofhw>0.25)
			{
				drawContours(con, contours, idx, Scalar(1), CV_FILLED, 8, hierarchy);
				if (countNonZero(con) > 50)
					rects.push_back(rect);

			}
		}
	}
	return rects;
}

void move_detect(Mat image)
{
				if (num < tol)
				{
					Mat temp;
					image.convertTo(temp, CV_16U);
					image_sum += temp;

				}

				if (num == tol)
				{
					image_sum /= tol;
					image_sum.convertTo(image_a, CV_8U);
//					namedWindow("avg");
//					imshow("avg", image_a);
				}
				
				if (num >= tol + 5)
				{
					
						Mat diff;
						Mat colorholeimage = image;
						Mat colorholeimage_a = image_a;
						absdiff(colorholeimage, colorholeimage_a, diff);

						Mat gray(h, w, CV_8UC1), b(h, w, CV_8UC1);
						cvtColor(diff, gray, CV_BGR2GRAY);
						//imwrite("gray.bmp", gray);
						double t = threshold(gray, b, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
						if (t > 10)
						{
							//cout <<"threshold:"<< t << endl;
							Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
							dilate(b, b, element);
							element = getStructuringElement(MORPH_RECT, Size(12, 12));
							erode(b, b, element);
							Vector<Rect> rects = findregion(b.clone());
							if (rects.size())
							{
								Rect rect;
								for (uchar i = 0; i < rects.size(); i++)
								{
									rect.x = rects[i].x;
									rect.y = rects[i].y;
									rect.height = rects[i].height;
									rect.width = rects[i].width;
									rectangle(image, rect, Scalar(0, 0, 255), 2);
								}
							}

//							namedWindow("close");
//							imshow("close", b);
							//imwrite("b.bmp", b);
						}
						num = 0; //修改为动态的环境检测动作识别
					

				}
				num++;
}
