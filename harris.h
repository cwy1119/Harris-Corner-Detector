#ifndef _HARRIS_H_
#define _HARRIS_H_
#include <opencv2/core/core.hpp>
#include <vector>
using std::vector;

class Harris
{
private:
    const int k = 0.05; //计算R时用到的参数

    cv::Mat Ix2;     //x方向的梯度(一阶导)
    cv::Mat Iy2;     //y方向的梯度(一阶导)
    cv::Mat Ixy;    //Ix * Iy

    cv::Mat R;      //R矩阵
    cv::Mat mmax;
    cv::Mat mmin;

    int max_r;

    int image_w;
    int image_h;

    template<typename IMGTYPE,typename FILTERTYPE>
    cv::Mat convolve(cv::Mat image,cv::Mat filter);

    cv::Mat calIx(cv::Mat image);
    cv::Mat calIy(cv::Mat image);
    cv::Mat calIxy(cv::Mat Ix,cv::Mat Iy);

    void calR();

    void meanFilter(cv::Mat& image);

    template<typename TYPE>
    void absMat(cv::Mat& mat);

    template<typename TYPE>
    cv::Mat powMat(cv::Mat& mat,int n=2);
public:
    Harris();
    Harris(cv::Mat image);
    ~Harris();

    void update(cv::Mat image);
    void getEigenvalue();
    cv::Mat getR();
    cv::Mat getMax();
    cv::Mat getMin();
    vector<cv::Point> getCorners(int threshold = 0); //获取角点坐标
};

#endif