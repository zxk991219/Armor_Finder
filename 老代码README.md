# TJ-ROBOMASTER-AUTOMATIC-AIMING

| 作者   | 负责部分       |
|:------:|:--------------:|
| 韩煦源 | 自瞄装甲板识别、分类器 |
| 曾宪坤 | PNP距离与角度解算 |

## 一、代码运行环境

|操作系统|运行库|
|-------|--------|
|Ubuntu16.04<br />Windows WSL|OpenCV 3.4.7<br />cmake 3.15.4|

- 本代码统一使用**640×480**大小的图像进行处理

## 二、程序编译及运行

### Ubuntu16.04（在项目文件夹下）

```shell
mkdir build
cd build
cmake ..
make
sudo ./main
```

### 程序所用宏介绍

```C++
// #define USE_VIDEO                   //使用录像调试
#define USE_CAMERA                     //使用摄像头调试

// #define DEBUG                       //打印DEBUG语句
// #define FRAME_BY_FRAME              //逐帧调试

// #define USE_RED                     //目标装甲板灯条为红色
#define USE_BLUE                       //目标装甲板灯条为蓝色

#define USE_HSV_FILTER                 //使用HSV二值化
// #define SHOW_DEBUG_HSV              //显示HSV二值化后图像
// #define SHOW_MEDIANBLUR             //显示中值滤波图像
// #define SHOW_CONTOURS               //显示获取的边缘
// #define SHOW_IMAGEPART_LIGHT        //显示灯条矩形截图
// #define SHOW_ARMOR_IMAGE            //显示装甲板矩形截图

// #define SHOW_LIGHT                  //显示灯条矩形框
#define SHOW_ARMOR                     //显示装甲板不规则四边形框
#define SHOW_DISTANCE                  //显示PNP距离与角度
#define SHOW_CLASSIFIER_IMAGE          //显示输入分类器的图像
#define CLASSIFIER_OUTPUT              //输出分类器结果截图到"Video/image/dst/negative"或"Video/image/dst/positive"
```

## 三、文件结构目录

``` Files Structure
.
├── armor                   // 存放自瞄主要算法代码
│   ├── include             // 自瞄头文件
│   └── src                 // 自瞄源码
├── CMakeLists.txt          // cmake工程文件
├── image                   // 存放测试代码使用的图像
│   ├── dst                 // 存放原图像
│   └── src                 // 存放经代码处理后的图像
├── main.cpp                // 主函数
├── other                   // 存放一些其他代码，如计时器、drawText
│   └── include             // other头文件
├── README.md               // 代码手册
└── Video                   // 存放调试代码所需的视频文件以及分类器输出的截图
```

## 四、关键类与函数解析  

### Armor_Finder

#### 1.image preprocess

|关键类与函数|解释|
|-|-|
|void hsvColorFilter(cv::Mat& inputImage, cv::Mat& outputImage)|进行HSV二值化|

#### 2.find lightbox  

|关键类与函数|解释|
|-|-|
|std::vector<cv::RotatedRect> findLightBox(cv::Mat& mat, cv::Mat& mat_real)|寻找灯条主函数<br />mat是已经经过了HSV二值化的图像，mat_real是摄像头探测到的原图|
|void cv::findContours(mat,contours, CV_RETR_LIST,CV_CHAIN_APPROX_NONE）|将二值化后的Mat进行边缘检测|
|cv::RotatedRect minAreaRect(contours[i])|获取轮廓的最小旋转矩形|
|cv::Point2f vertices[4]<br />rect.points(vertices)|获取旋转矩形的四个顶点|
|bool lightbox_isok(mat_imagepart, rect)|通过长宽比和HSV亮度筛选灯条矩形|
|cv::Mat rotateRectToMat(mat_real, rect)|将旋转矩形所对应的区域内的图像转为正向|

#### 3.match lightbox_i_j

|关键类与函数|解释|
|-|-|
|bool armorbox_isok(const cv::RotatedRect rect_i, const cv::RotatedRect rect_j)|判断两灯条能否匹配主函数|
|bool angle_isok(const cv::RotatedRect rect_i, const cv::RotatedRect rect_j)          |角度差匹配|
|bool center_height_isok(const cv::RotatedRect rect_i, const cv::RotatedRect rect_j)  |中心高度差匹配|
|bool center_distance_isok(const cv::RotatedRect rect_i, const cv::RotatedRect rect_j)|中心距离和长度比匹配|
|bool length_rate_isok(const cv::RotatedRect rect_i, const cv::RotatedRect rect_j)    |灯条长宽比匹配|

#### 4.get armorbox

|关键类与函数|解释|
|-|-|
|get_armor(cv::Mat& mat_real, const cv::RotatedRect rect_i, const cv::RotatedRect rect_j)<br />|将灯条对的四个外顶点向上下按比例扩大至装甲板边缘，得出装甲板四边形|

#### 5.PNP_distance  

|关键类与函数|解释|
|-|-|
|double m[3][3] = {{1056.641597953005, 0, 958.1078670170519}, { 0, 1055.821668018513, 558.7308899751256}, {0, 0, 1}}  <br />cv::Mat cam= cv::Mat(3, 3, CV_64F, m) <br />double n[5]={-0.4101674487087525,0.2060326419033235, 0.0007446583829303401, -0.0009071701383907473, -0.05685326030318479}|相机内参|
|cv::Mat dis=cv::Mat(1,5, CV_64F, n)|畸变参数|  
|pts coordinate|不规则矩形像素坐标|
|world coordinate|定义世界坐标|  
|cv::solvePnP(obj,pnts,cam,dis,rVec,tVec,false,cv::SOLVEPNP_ITERATIVE)|利用solvepnp解算出平移向量和旋转向量|  
|cv::Rodrigues(rVec, rotM)|罗德里格斯变化将旋转向量变化成旋转矩阵|
|double theta_x = atan2(rotM.at(2, 1), rotM.at(2, 2)) <br />double theta_y = atan2(-rotM.at(2, 0), sqrt(rotM.at(2, 1)*rotM.at(2, 1) + rotM.at(2, 2)*rotM.at(2, 2))) <br />double theta_z = atan2(rotM.at(1, 0), rotM.at(0, 0))|解出角度|  

#### 6.classier  

|关键类与函数|解释|
|-|-|
|cv::Mat quadrilateralToMat(cv::Mat& mat_real, cv::Point2f vertices_armor[4])|将装甲板不规则四边形所对应的区域内的图像仿射变换转为正向|
|int classifier(const cv::Mat& src, std::string template_filename_list)|分类器主函数，将装甲板图像与模板比较，并打分，累计分数，返回装甲板ID|

#### 7.other

|关键类与函数|解释|
|-|-|
|class timer  <br /> void reset() <br /> std::time_t getTimeStamp()|程序计时器|
|void drawText_quadrilateral(cv::Mat& mat_real, const cv::Point2f point_pre, const std::string str)|在装甲板四边形顶点处写入文本框|

## 五、装甲板识别程序运行流程

- 首先对图像进行hsv二值化：将符合装甲板灯条hsv颜色的像素赋值为白色，其他像素赋值为黑色，并使用中值滤波使图像平滑
- 使用边缘提取获得可能是灯条的旋转矩形区域
- 根据旋转矩形的长宽比、旋转矩形对应原图区域像素的hsv明度筛选出灯条矩形
- 对所有可能的灯条进行两两匹配，根据两灯条夹角、两灯条的高度比、两灯条中心的高度差和两灯条中心的距离与灯条高度的比值进行筛选，得出符合条件的灯条对。
- 将灯条对的四个外顶点向上下按比例扩大至装甲板边缘，得出装甲板四边形
- 将装甲板四边形区域截图进行仿射变换得到装甲板图像并交给分类器判断，得出装甲板及其数字id
- 最后将框出的装甲板四边形放入PNP进行距离和角度的解算

![avatar](./自瞄流程图.png)

## 六、代码命名规范

- 函数名：使用首字母小写的驼峰命名法
- 类型名：使用首字母大写的驼峰命名法
- 变量名：使用下划线分割命名法

