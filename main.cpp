#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/shape/shape.hpp>
#include "harris.h"

using namespace std;
using namespace cv;

Harris harris;

void cut(cv::Mat image)
{
    cv::Mat result = image.clone();
    cv::Mat min;
    cv::Mat max;
    cv::Mat R;

    harris.update(image);

    vector<cv::Point> corners = harris.getCorners();
    int size = corners.size();
    for(int i=0;i<size;i++){
        int x = corners[i].x;
        int y = corners[i].y;
        rectangle(result,cv::Point(x-4,y-4),cv::Point(x+4,y+4),cv::Scalar(0,255,0),2);
    }

    min = harris.getMin();
    max = harris.getMax();
    R   = harris.getR();

    //保存图片
    imwrite("result.jpg",result);
    imwrite("min.jpg",min);
    imwrite("max.jpg",max);
    imwrite("R.jpg",R);

    //展示中间结果
    imshow("result",result);
    imshow("min",min);
    imshow("max",max);
    imshow("R",R);
}

int main()
{
    VideoCapture cap(0);
    if(!cap.isOpened()){
        cout<<"error: can't open the camera"<<endl;
        return -1;
    }

    Mat frame;
    namedWindow("frame",1);

    while(true){
        cap>>frame;
        imshow("frame",frame);
        char key = waitKey(3);
        if(key == ' '){     //
             cut(frame);
             char key2; //阻塞暂停
             do{
                 key2 = waitKey();
                 if(key2 == 27) return 1;
             }while(key2 != ' ');
        }else if(key == 27){ //esc键退出
            return 1;
        }
    }
}