/**
 * @author cwy
 * @date 2020/12/14
 * @description: 这是cwy计算机视觉课程第三次作业封装的库
 * @function: 实现了基于Harris算法的角点检测
*/

#include "harris.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;

Harris::Harris(cv::Mat image)
{
    update(image);
}

Harris::Harris()
{
}

void Harris::update(cv::Mat image)
{
    image_w = image.cols;
    image_h = image.rows;

    cv::Mat image_grey;
    cv::cvtColor(image,image_grey,cv::COLOR_BGR2GRAY);


    //计算x和y方向的一阶导, 并计算Ix*Iy
    cv::Mat Ix = calIx(image_grey)/5;
    cv::Mat Iy = calIy(image_grey)/5;
    Ixy = calIxy(Ix,Iy);


    Ix2 = powMat<int32_t>(Ix);
    Iy2 = powMat<int32_t>(Iy);

    //线性窗口求和
    meanFilter(Ix2);
    meanFilter(Iy2);
    meanFilter(Ixy);


    getEigenvalue(); //计算特征值
    calR();                 //计算R
}

Harris::~Harris()
{
}

//使用线性矩阵进行window(x,y)进行卷积
void Harris::meanFilter(cv::Mat& image)
{
    const int WSIZE = 5;
    float window[WSIZE][WSIZE] ={
        { 2, 4, 5,  4,  2 },
        { 4, 9, 12, 9,  4 },
        { 5, 12,15, 12, 5 },
        { 4, 9, 12, 9,  4 },
        { 2, 4, 5,  4,  2 }
    };

    cv::Mat filter(WSIZE,WSIZE,CV_32F,window);
    filter = filter/159;

    image = convolve<int32_t,float>(image,filter);
}

//计算Iy
cv::Mat Harris::calIy(cv::Mat image)
{
    cv::Mat Iy(image_h,image_w,CV_32S,cv::Scalar::all(0));
    /**
     * 卷积算子
     *  2  2  4  2  2
     *  1  1  2  1  1
     *  0  0  0  0  0
     * -1 -1 -2 -1 -1
     * -2 -2 -4 -2 -2
    */ 
   int y_filter[] = {2,1,0,-1,-2};
   int x_filter[] = {1,1,2,1,1};

   cv::Mat tmp = convolve<uchar,int32_t>(image,cv::Mat(5,1,CV_32S,y_filter)); 
   Iy = convolve<int32_t,int32_t>(tmp,cv::Mat(1,5,CV_32S,x_filter));
   absMat<int32_t>(Iy);

   return Iy;
}

//计算Ix
cv::Mat Harris::calIx(cv::Mat image)
{
    cv::Mat Ix(image_h,image_w,CV_32S,cv::Scalar::all(0));
    /**
     * 卷积算子
     *  2  1  0  -1  -2
     *  2  1  0  -1  -2
     *  4  2  0  -2  -4
     *  2  1  0  -1  -2
     *  2  1  0  -1  -2
    */ 
   int y_filter[] = { 1,1,2,1,1 };
   int x_filter[] = { 2,1,0,-1,-2};

    /**
     * 为了提高计算速度
     * 先在y方向做卷积,之后在x方向做卷积
     */
   cv::Mat tmp = convolve<uchar,int32_t>(image,cv::Mat(5,1,CV_32S,y_filter)); 
   Ix = convolve<int32_t,int32_t>(tmp,cv::Mat(1,5,CV_32S,x_filter));
   absMat<int32_t>(Ix);

   return Ix;
}

//计算Ixy
cv::Mat Harris::calIxy(cv::Mat Ix,cv::Mat Iy)
{
    cv::Mat Ixy(image_h,image_w,CV_32S,cv::Scalar::all(0));

    for(int y = 0; y < image_h; y++){
        for(int x = 0; x < image_w; x++){
            Ixy.at<int32_t>(y,x) = Ix.at<int32_t>(y,x)*Iy.at<int32_t>(y,x);
        }
    }
    return Ixy;
}

//取绝对值
template<typename TYPE>
void Harris::absMat(cv::Mat& mat)
{
    int w = mat.cols;
    int h = mat.rows;

    for(int y = 0; y < h; y++){
        for(int x = 0; x < w; x++){
            if(mat.at<TYPE>(y,x)<0) {
                mat.at<TYPE>(y,x) *= -1;
            }
        }
    }

}

//取绝对值
template<typename TYPE>
cv::Mat Harris::powMat(cv::Mat& mat,int n)
{
    
    int w = mat.cols;
    int h = mat.rows;
    cv::Mat result(h,w,CV_32S,cv::Scalar::all(0));

    for(int y = 0; y < h; y++){
        for(int x = 0; x < w; x++){
            result.at<int32_t>(y,x)  = pow(mat.at<TYPE>(y,x),n);
        }
    }
    return result;
}

//计算R的值
void Harris::calR()
{
    int max = 0;
    R = cv::Mat(image_h,image_w,CV_32S);
    for(int y=0;y<image_h;y++){
        for(int x=0;x<image_w;x++){

            int det = mmin.at<int32_t>(y,x) * mmax.at<int32_t>(y,x);
            int trace = mmin.at<int32_t>(y,x) + mmax.at<int32_t>(y,x);
            R.at<int32_t>(y,x) = det - k*trace*trace;
            if( R.at<int32_t>(y,x) > max) max =  R.at<int32_t>(y,x);
        }
    }
    this->max_r = max;
}





/**
 * @param image 被卷积的矩阵,通道数需要为1
 * @param filter 卷积的算子, 长和宽需要是奇数
 * @return 卷积的结果, 大小和image保持一致
 * @description: 计算卷积
*/
template<typename IMGTYPE,typename FILTERTYPE>
cv::Mat Harris::convolve(cv::Mat image,cv::Mat filter)
{
    const int filter_w = filter.cols;
    const int filter_h = filter.rows;
    // cout<<"convolve "<<filter_h<<":"<<filter_w<<endl;
    const int half_filter_w = filter_w/2;
    const int half_filter_h = filter_h/2;
    // cout<<"half convolve "<<half_filter_h<<":"<<half_filter_w<<endl;
    cv::Mat result(image_h,image_w,CV_32S,cv::Scalar::all(0));
    // cout<<"f:"<<filter<<endl;
    for(int y = 0; y < image_h; y++){
        for(int x = 0; x < image_w; x++){
            for(int j = -half_filter_h; j <= half_filter_h; j++){
                for(int i = -half_filter_w; i <= half_filter_w; i++){

                    //卷积的值越界(超出原图的范围,则该点用中心点代替)
                    if(x+i<0 || x+i >= image_w || y+j < 0 || y+j >= image_h){
                        result.at<int32_t>(y,x) +=  (filter.at<FILTERTYPE>(j+half_filter_h,i+half_filter_w))*\
                                                image.at<IMGTYPE>(y,x);
                    }else{
                        result.at<int32_t>(y,x) += (filter.at<FILTERTYPE>(j+half_filter_h,i+half_filter_w))*\
                                                image.at<IMGTYPE>(y+j,x+i);
                    }
                }
            }
        }
    }
    return result;
}

/**
 * @description: 计算特征值中的最大值和最小值
 */ 
void Harris::getEigenvalue()
{
    mmin = cv::Mat(image_h,image_w,CV_32S,cv::Scalar::all(0));
    mmax = cv::Mat(image_h,image_w,CV_32S,cv::Scalar::all(0));
    for(int y = 0; y < image_h; y++){
        for(int x = 0; x < image_w; x++){
            double mat[] = {
                Ix2.at<int32_t>(y,x), Ixy.at<int32_t>(y,x),
                Ixy.at<int32_t>(y,x), Iy2.at<int32_t>(y,x)
            };
            cv::Mat tmp(2,2,CV_64FC1,mat);
            cv::Mat mvalue;
            cv::eigen(tmp, mvalue);
            mmax.at<int32_t>(y,x) =(int) (mvalue.at<double>(0,0) );
            mmin.at<int32_t>(y,x) =(int) (mvalue.at<double>(1,0) );
        }
    }

}

//获取R的值
cv::Mat Harris::getR()
{
    cv::Mat result(image_h,image_w,CV_8UC3,cv::Scalar::all(0));
    for(int y=0;y<image_h;y++){
        for(int x=0;x<image_w;x++){
            if(this->R.at<int32_t>(y,x) > 0){
                int v = 1.0*this->R.at<int32_t>(y,x)/max_r*(255*255*255);
                result.at<cv::Vec3b>(y,x)[2] = v%255;
                v /= 255;
                result.at<cv::Vec3b>(y,x)[1] = v%255;
                v /= 255;
                result.at<cv::Vec3b>(y,x)[0] = v;
            }else{
                int v = -1.0*this->R.at<int32_t>(y,x)/max_r*(255*255*255);
                result.at<cv::Vec3b>(y,x)[0] = v%255;
                v /= 255;
                result.at<cv::Vec3b>(y,x)[1] = v%255;
                v /= 255;
                result.at<cv::Vec3b>(y,x)[2] = v;
            }
            
        }
    }
    return result;
}

//获取Max
cv::Mat Harris::getMax()
{
    cv::Mat result(image_h,image_w,CV_8U,cv::Scalar::all(0));

    // 获取最大值
    int maxvalue = 0;
    for(int y=0;y<image_h;y++){
        for(int x=0;x<image_w;x++){
            if(this->mmax.at<int32_t>(y,x) > maxvalue){
                maxvalue = this->mmax.at<int32_t>(y,x);
            }
        }
    }

    for(int y=0;y<image_h;y++){
        for(int x=0;x<image_w;x++){
            result.at<uchar>(y,x) = 1.0*this->mmax.at<int32_t>(y,x)/maxvalue*255;
        }
    }

    return result;
}

//获取Min
cv::Mat Harris::getMin()
{
    cv::Mat result(image_h,image_w,CV_8U,cv::Scalar::all(0));

    // 获取最大值
    int maxvalue = 0;
    for(int y=0;y<image_h;y++){
        for(int x=0;x<image_w;x++){
            if(this->mmin.at<int32_t>(y,x) > maxvalue){
                maxvalue = this->mmin.at<int32_t>(y,x);
            }
        }
    }

    for(int y=0;y<image_h;y++){
        for(int x=0;x<image_w;x++){
            result.at<uchar>(y,x) = 1.0*this->mmin.at<int32_t>(y,x)/maxvalue*255;
        }
    }

    return result;
}

//获取角点坐标
vector<cv::Point> Harris::getCorners(int threshold)
{
    const double MIN_THRES = 100; //最小阈值
    //默认取最大值的十分之一
    //默认为相对值, 对于纯色图则不适用, 所以设置100的最小阈值
    if(threshold == 0) {
        threshold = max(0.05*max_r,MIN_THRES);
    }
    const int MASK = 8;
    vector<cv::Point> points;
    for(int y = 0; y < image_h; y++){
        for(int x = 0; x < image_w; x++){
            if(R.at<int32_t>(y,x) > threshold){
                bool flag = true;
                for(int j = -MASK;j<=MASK;j++){
                    for(int i=-MASK;i<=MASK;i++){
                        if( y+j < 0 ||y+j>=image_h||x+i<0||x+i>=image_w) continue;
                        else if( (R.at<int32_t>(y+j,x+i) > R.at<int32_t>(y,x)) ||\
                             (R.at<int32_t>(y+j,x+i) == R.at<int32_t>(y,x) && (j<0 || i<0))){
                            flag = false;
                            i = j = MASK+1;
                        }
                    }
                }
                if(flag) points.push_back(cv::Point(x,y));
            }
        }
    }

    //检测的点过少, 则降低标准再检测一次
    if(threshold>MIN_THRES && points.size()<20){
        points = getCorners(threshold*0.8);
    }
    return points;
}

