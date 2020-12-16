# Implement of Harris Corner Detector


## 一、代码功能

1. 写代码实现Harris Corner检测算法
2. 读入摄像头, 按空格暂停回放并做一次Harris Corner检测
3. 将中间结果保存下来



## 二、软件说明

### 1) 环境依赖

+ opencv3.4
+ mingw64 (作者采用的编译器, 也可以采用其他的)

### 2) 编译项目	

+ 修改makefile文件中的`INCLUDE_PATH`和`LIB_PATH`,改为本机opencv的路径
+ 输入`make`或者`mingw32-make`即可编译

### 3) 运行程序

​		终端输入`./harris.exe`(window环境)即可运行,  按下空格键即可做一次Harris Corner检测,程序会自动将中间结果的图保存下来. 再按一次空格, 可以继续选择.



## 三、算法实现

### 1) 提取过程

 1. 按下空格后从摄像头读取图片, 并转为灰度图

 2. 使用sobel算子计算Ix,Iy的值, 代码中采用的为5*5的sobel算子

    - 计算Ix采用的算子

    $$
    {\left[ {\begin{array}{*{20}{c}}
    2&1&0&-1&-2\\
    2&1&0&-1&-2\\
    4&2&0&-2&-4\\
    2&1&0&-1&-2\\
    2&1&0&-1&-2\
    \end{array}} \right]}
    $$

    - 计算Iy采用的算子

    $$
    {\left[ {\begin{array}{*{20}{c}}
    2&2&4&2&2\\
    1&1&2&1&1\\
    0&0&0&0&0\\
    -1&-1&-2&-1&-1\\
    2&-2&-4&-2&-2\
    \end{array}} \right]}
    $$

3. 使用window(x,y)窗口对Ix,Iy进行加权平均,窗口算子如下
   $$
   {\left[ {\begin{array}{*{20}{c}}
       2&4&5&4&2\\
       4&9&12&9&4\\
       5&12&15&12&5\\
       4&9&12&9&4\\
       2&4&5&4&2
       \end{array}} \right]}
   $$

4. 计算特征矩阵的特征值max, min

$$
{\left[ {\begin{array}{*{20}{c}}
   Ix^2&IxIy\\
   IxIy&Iy^2
   \end{array}} \right]}
$$

5. 根据以下公式计算R的值
   $$
   R = detM - k(traceM)^2 
   $$

$$
detM =  λ1λ2
$$

$$
traceM =  λ1 +  λ2
$$

6. 最后根据R的值来选取特征点(其中R值为正且数值较大的点归为角点, R值为负且值较大的点归为边缘点, 剩下的归为块状区域点.

###  2) 主要算法

1. 自己封装了一个卷积函数, 用于sobel算子求梯度和window窗口求加权平均值, 函数核心代码如下:

   ```c++
   /**
    * @param image 被卷积的矩阵,通道数需要为1
    * @param filter 卷积的算子, 长和宽需要是奇数
    * @return 卷积的结果, 大小和image保持一致
    * @description: 计算卷积
   */
   cv::Mat Harris::convolve(cv::Mat image,cv::Mat filter){
   for(int y = 0; y < image_h; y++){
       for(int x = 0; x < image_w; x++){
           for(int j = -half_filter_h; j <= half_filter_h; j++){
               for(int i = -half_filter_w; i <= half_filter_w; i++){
   
                   //卷积的值越界(超出原图的范围,则该点用中心点代替)
                   if(x+i<0 || x+i >= image_w || y+j < 0 || y+j >= image_h){
                     ...//省略, 算子扫描时越界, 则用当前点的值代替
                   }else{
                     ...//细节省略
                   }
               }
           }
       }
   }
   }
   
   ```

2. 计算特征矩阵的特征值, 计算特征值时采用opencv提供的`cv::eigen`函数进行实现, 将得到的两个参数分别存入max和min图中.

3. 计算得到R图后, 设置一定的阈值进行筛选, 和实验二Hough检测结果类似, 检测结果往往是扎堆的, 为了防止过于密集, 采用了一个MASK*MASK大小掩码对于区域的点进行筛选

### 3) 代码优化

1. 使用sobel算子进行梯度求职的时候, 将一个二维的算子查分成两个一维的算子进行卷积, 以达到降低时间复杂度的目的.

   ```c++
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
   ```

2. 设置了一个阈值自动调整机制: 对于不同的结果, 往往会需要不同的阈值. 在阈值过大的情况下, 检测到的角点就会偏少, 所以函数在实现时, 发现检测的角点较少时( < 20), 会自动降低阈值重新检测.核心代码如下:

   ```c++
   vector<cv::Point> Harris::getCorners(int threshold)
   {
       const double MIN_THRES = 100; //最小阈值
       //默认取最大值的十分之一
       //默认为相对值, 对于纯色图则不适用, 所以设置100的最小阈值
       if(threshold == 0) {
           threshold = max(0.05*max_r,MIN_THRES);
       }
      			....//检测角点
   
       //检测的点过少, 则降低标准再检测一次
       if(threshold>MIN_THRES && points.size()<20){
           points = getCorners(threshold*0.8);
       }
       return points;
   }
   ```

## 四、实验结果

1. 摄像头读入的原图展示如下:

   ![quicker_d148c3eb-ad53-4f20-aa60-eb151a1bfbaf.png](https://i.loli.net/2020/12/16/ZlGmdxtX2fJUASR.png)

2. 按下空格键, 经过一次Harris Corner检测的结果如下: 可以观察到,复杂场景情况下,桌子,椅子和一些小的物品的角点基本上都能够被正确地识别. (但是也存在少量的点没有被检测出来)

   ![quicker_37e483b6-1405-492f-9b99-e3c2f97db790.png](https://i.loli.net/2020/12/16/sZknEaIMtCgBDir.png)

3. 该场景的一些过程图展示如下, 第一张图为特征值中较大者, 第二张为特征值较小者, 第三章图为R图. 在R图中,可以较为直观地看到不同像素点的R值, 其中红色和黑色区域 表示R值比较小, 即块状区域, 黄色表示边缘区域, 白色部分就是我们需要找的角点部分.

   ![quicker_db9901d4-a002-47de-8d1a-e0a49d812e57.png](https://i.loli.net/2020/12/16/MimtohHCXb6A8rU.png)

   ![quicker_24abc0e3-741d-459a-bb61-30976ea6206d.png](https://i.loli.net/2020/12/16/WdOE8cyZzf9miSN.png)

   ![quicker_6d787caa-0ced-4bb6-beea-772c05ea33ab.png](https://i.loli.net/2020/12/16/M2COfFhw8qerTmI.png)

4. 对于一些比较简单的场景, 由于有最小识别数目的保证, 检测效果也比较令人满意, 但是相比于复杂的场景, 将边缘点误认为角点的概率会增大. 展示图如下

   ![quicker_046f3142-6380-4dd0-b9d0-c7481e401166.png](https://i.loli.net/2020/12/16/hzEmbP3igYoNlCf.png)


