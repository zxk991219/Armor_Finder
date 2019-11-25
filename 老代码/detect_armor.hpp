//
// Created by yzy on 18-3-12.
//
#ifndef ARMOR_DETECTARMOR_H
#define ARMOR_DETECTARMOR_H

#include <algorithm>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

#include "camera.hpp"
#include "tools.hpp"

#define MODEL_PATH "../data/model.xml"

// #define RED
#define BLUE

#define BUBING

// 扩展小图, todo: 宏定义图片尺寸
void getBoundingRect(Target &tar, cv::Rect &rect, bool extendFlag = false) {
    rect.x = (int) cv::min(tar.pixelPts2f[0].x, tar.pixelPts2f[1].x); //tl和bl的x小值
    rect.y = (int) cv::min(tar.pixelPts2f[0].y, tar.pixelPts2f[3].y); //tl和tr的y小值
    rect.width = (int) cv::max(tar.pixelPts2f[2].x, tar.pixelPts2f[3].x) - rect.x; 
    rect.height = (int) cv::max(tar.pixelPts2f[1].y, tar.pixelPts2f[2].y) - rect.y;
    // printf("rect %d, %d, %d, %d \n", rect.x, rect.y, rect.width, rect.height);
    // for (const auto &pts: tar.pixelPts2f) {
    //     std::cout << pts << std::endl;
    // }
    if (extendFlag) {
        
        rect.x -= rect.width * 3;
        rect.y -= rect.height * 2.5;
        rect.width *= 7;
        rect.height *= 6;
        
        rect.width = rect.width > 640 ? 640 : rect.width;
        rect.height = rect.height > 480 ? 480 : rect.height;
        
        rect.width = rect.width < 80 ? 80 : rect.width;
        rect.height = rect.height < 37 ? 37 : rect.height;
        
        rect.x = rect.x < 0 ? 0 : rect.x;
        rect.y = rect.y < 0 ? 0 : rect.y;
        
        rect.width = rect.x + rect.width > 640 ? 640 - rect.x : rect.width;
        rect.height = rect.y + rect.height > 480 ? 480 - rect.y : rect.height;
        
    }
}


class DetectArmor {
private:
    static constexpr float maxAngleL2L = 11;                          // 两灯最大角度差
    static constexpr double maxAngleT2B = std::cos(M_PI / 180 * 17);  // 无灯两边最大角度差
    static constexpr float maxRatioL2L = 2.0;                         // 两灯长度最大比值
    static constexpr float maxRatioC2L = 5.7;                         // 中心距与灯长最大比值
    static constexpr float minDeltaXL2L = 2;                          // 两灯横向最小距离

    static constexpr unsigned char minBrightness = 100;  // 候选区域最小亮度
    static constexpr float maxOverlap = 0.1;  // mser检测结果去重的重叠阈值

    constexpr static float alpha = 0.5;  // 欧式距离与旋转向量变动值的线性组合权重
    // 以下2个为跟踪装甲片的阈值, 即阈值范围内的装甲片为同一个装甲片
    constexpr static float thresholdSimi = 3.0;  // 装甲片移动旋转向量变动阈值
#ifdef BUBING
    constexpr static float thresholdEularDist = 3830;  // 装甲片移动空间欧式距离阈值
#else
    constexpr static float thresholdEularDist = 7500;  // 装甲片移动空间欧式距离阈值
#endif
    constexpr static float thresholdYaw = 5.0;  // 装甲片移动相对车辆的 yaw ( global yaw has been considered )
    int currentId;  // 当前要传给上位机的那个装甲片目标的id
    int idGrow = 17;  // 装甲片id, 累加
    std::deque<Target> targetsHistory;  // target, id ,count
    #ifdef BUBING
    static constexpr int maxHistory = 2;  // 历史最大存储多少帧的装甲片
    #else
    static constexpr int maxHistory = 4;  // 历史最大存储多少帧的装甲片
    #endif
    Target latestShootTarget;
    int frameCounter;  // track函数调用计数, 作为预测函数的时间值

    cv::HOGDescriptor hog;
    cv::Ptr<cv::MSER> mser;
    cv::Ptr<cv::ml::RTrees> forest;
    cv::Ptr<cv::ml::SVM> svm;
    std::vector<cv::Point2f> armorFigureStd;  // 装甲板实际比例缩放后取的图片尺寸, 所有截取区域都会被重投影成这个尺寸
    cv::Mat gray;  // 灰度图

    int imgSaveCounter;  // 样本图片保存的文件名, 累加

    void sortLights(std::vector<Light> &arr) {
        for (int i = 1; i < arr.size(); i++) {
            int j = i;
            while (j > 0 && arr[j - 1].centerPoint.x < arr[j - 1].centerPoint.x) {
                std::swap(arr[j], arr[j - 1]);
                j--;
            }
        }
    }

    // mser重叠区域剔除
    float overlap(const cv::Rect &box1, const cv::Rect &box2) {
        if (box1.x > box2.x + box2.width) {
            return 0.0;
        }
        if (box1.y > box2.y + box2.height) {
            return 0.0;
        }
        if (box1.x + box1.width < box2.x) {
            return 0.0;
        }
        if (box1.y + box1.height < box2.y) {
            return 0.0;
        }

        float colInt = std::min(box1.x + box1.width, box2.x + box2.width) - std::max(box1.x, box2.x);
        float rowInt = std::min(box1.y + box1.height, box2.y + box2.height) - std::max(box1.y, box2.y);

        float intersection = colInt * rowInt;
        float area1 = box1.width * box1.height;
        float area2 = box2.width * box2.height;
        return intersection / (area1 + area2 - intersection);
    }

    // 旧hog特征提取函数
    cv::Mat getFeature(cv::Mat &crop) {
//        cv::Mat roi;
//        if (roi.channels() == 3) {
//            cv::cvtColor(crop, roi, cv::COLOR_RGB2GRAY);
//            cv::resize(roi, roi, cv::Size(32, 32), 0, 0);
//        } else
//            cv::resize(crop, roi, cv::Size(32, 32), 0, 0);

        std::vector<float> descriptors;
        std::vector<cv::Point> locations;
        hog.compute(crop, descriptors, cv::Size(8, 8));

        cv::Mat ret = cv::Mat::zeros(1, (int) descriptors.size(), CV_32FC1);
        for (int k = 0; k < descriptors.size(); k++)
            ret.at<float>(0, k) = descriptors[k];
        // for(int r = 0; r<roi.rows; r++)
        //	for(int c=0; c<roi.cols; c++)
        //    ret.at<float>(0,k++) = roi.at<unsigned char>(r,c)/255.f;
        // ret.at<float>(descriptors.size()) = (float)roi.cols/(float)roi.rows;
        return ret;
    }

    // 计算多边形内角
    void rectAngle(std::vector<cv::Point2d> &pts, std::vector<double> &angles) {
        std::vector<cv::Mat> vs;
        for (int i = 0; i < pts.size(); i++) {
            if (i == pts.size() - 1)
                vs.emplace_back(cv::Mat(pts[i] - pts[0]));
            else
                vs.emplace_back(cv::Mat(pts[i] - pts[i + 1]));
        }
        // std::vector<double> angles;
        for (int i = 0; i < vs.size(); i++) {
            if (i == vs.size() - 1)
                angles.emplace_back(acos(vs[i].dot(vs[0]) / norm(vs[i]) / norm(vs[0])) / M_PI * 180.0);  // 0~180
            else
                angles.emplace_back(acos(vs[i].dot(vs[i + 1]) / norm(vs[i]) / norm(vs[i + 1])) / M_PI * 180.0);
        }
        // return angles;
    };

    // 遍历vector是否满足条件,返回真值
    bool vectorFilter(std::vector<double> &obj, bool (*filterFunc)(double)) {
        if (!obj.empty()) {
            for (const double i : obj) {
                if (!filterFunc(i))
                    return false;
            }
        } else
            return false;
        return true;
    }


public:
    bool beginTrack = false;

    int findHeroCounter = 0;
    
    std::vector<float> resultTemp;

    DetectArmor()
            : hog(cv::HOGDescriptor(cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), cv::Size(8, 8), 9)),
              forest(cv::ml::RTrees::load("../data/zhuangjia_Rtrees4_3.xml")), //初始化随机森林
              svm(cv::ml::SVM::load("../data/SVM.xml")), //初始化svm
              frameCounter(0),
              idGrow(0),
              currentId(0),
              mser(cv::MSER::create(15, 5, 400, 0.7)),
              imgSaveCounter(0) {
        if (forest->empty()) {
            printf("failed load xml \n");
            exit(1);
        }
        latestShootTarget.count = -1;
        // 以下按图像坐标
        // 0 3
        // 1 2
        armorFigureStd.emplace_back(cv::Point(0, 0));
        armorFigureStd.emplace_back(cv::Point(0, 55));
        armorFigureStd.emplace_back(cv::Point(125, 55));
        armorFigureStd.emplace_back(cv::Point(125, 0));

        printf("Detector init finish\n");
    }

    // 检测, 提出crop
    void detect(const cv::Mat &frame, std::vector<Target> &totalTargets) {
        // 颜色分割 ////////////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("cvtColor");
        cv::Mat hsv;
        if (frame.channels() == 3) { //检测是否是三通道图像，是三通道图像则转化hsv和灰度
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else { //不是三通道图像直接结束程序
            printf("frame's channel != 3 \n");
            exit(0);
        }
        is.clock("cvtColor"); 
        is.addImg("gray", gray); //将灰度图显示在调整后的gray窗口中

        // MSER ///////////////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("mser");
        int imWidth = gray.size().width;
        int imHeight = gray.size().height;
        std::vector<std::vector<cv::Point>> regions; //区域点集容器
        std::vector<cv::Rect> bBoxes; //矩形容器
        mser->setPass2Only(true); 
        mser->detectRegions(gray, regions, bBoxes);
        is.clock("mser");
        is.addEvent("mser", regions);

        // 去除太小的轮廓, 位于gray边缘的轮廓, 亮度 ///////////////////////////////////////////////////////////////////////////////////////////
        is.clock("too small");
        std::vector<int> ids; //计数器容器
        for (int i = 0; i < regions.size(); i++) { //遍历轮廓
            cv::Rect &box = bBoxes[i];
            if (box.x < 4 || box.y < 4 || ((box.x + box.width) > (imWidth - 4)) ||
                ((box.y + box.height) > (imHeight - 4)) || box.height / box.width < 2)
                //bbox的位置不能在gray图像的边缘4像素内且高宽比需要大于2
                continue;
            // 亮度筛选, 可删除
            // int sumGray = std::accumulate(regions[i].cbegin(), regions[i].cend(), 0,
            //                               [&](int a, cv::Point b) { return a + gray.at<uchar>(b); });
            // if (sumGray < minBrightness * regions[i].size())
            //     continue;
            if (regions[i].size() < 3) continue;  // 去太小,去除只有3个像素点的轮廓
            ids.emplace_back(i); //符合位置大小要求的轮廓被放入ids容器
        }
        is.clock("too small");
        // printf("去太小\n");

        // 去重 ////////////////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("Overlap");
        std::sort(ids.begin(), ids.end(), //将轮廓点集按照点集点数升序排列
                  [&regions](const int &a, const int &b) { return regions[a].size() < regions[b].size(); });
        std::vector<int> idsNoRepeat; //没有重复容器
        while (!ids.empty()) { 
            idsNoRepeat.emplace_back(ids.back()); //将最大的点集放入idsNoRepeat
            ids.pop_back();
            for (auto iter = ids.begin(); iter != ids.end();) {
                if (overlap(bBoxes[*iter], bBoxes[idsNoRepeat.back()]) > maxOverlap) //当当前遍历的点集与idsNoRepeat末尾点集重叠度大于阈值时
                    iter = ids.erase(iter); //将当前遍历点集从ids删除
                else
                    iter++;
            }
        }
        is.clock("Overlap");
        is.addEvent("mser no repeat", regions, idsNoRepeat, true);
        // printf("去重\n");

        // 筛颜色, 遍历region todo: 参数修改!!!! /////////////////////////////////////////////////////////////////////////
        is.clock("Color Filter");
        std::vector<int> idsColorFilter;
        for (const auto &_id : idsNoRepeat) { //遍历idsNoRepeat的轮廓
            // printf("area: %d \n", regions[_id].size());
            int okPtCount = 0;
            auto okMinCount = regions[_id].size() * 0.4; //轮廓点数的0.4倍
            okMinCount = okMinCount > 100 ? 100 : okMinCount; //轮廓点数的0.4倍大于100则赋值为100，小于100则用原值
            okMinCount = okMinCount < 2 ? 2 : okMinCount; //轮廓点数的0.4倍小于2则赋值为2，大于2则用原值 
            bool colorFound = false;
#ifdef RED
            //  int sumH = std::accumulate(regions[_id].cbegin(), regions[_id].cend(), 0,
            //                             [&hsv](int a, cv::Point bp) {
            //                                 if (hsv.at<cv::Vec3b>(bp)[0] < 15 || hsv.at<cv::Vec3b>(bp)[0] > 165)
            //                                     return a + 1;
            //                                 else return a;
            //                             });  // 累加合格的点
            //  float ratio_ = (float) sumH / regions[_id].size();
            // // printf("ratio_ %.2f \n", ratio_);
            //  if (ratio_ < 0.3)
            //      continue;

            for (const auto &_reg:regions[_id]) { //遍历regions[_id]的点
                if (okPtCount > okMinCount) { //当符合红色的点的个数大于总个数的0.4倍时colorFound=true，中断本遍历点循环
                    colorFound = true;
                    break;
                }
                if (hsv.at<cv::Vec3b>(_reg)[0] < 15 || hsv.at<cv::Vec3b>(_reg)[0] > 165) { //当符合红色时okPtCount++
                    okPtCount += 1;
                }
            }
            if (!colorFound) continue; //当未发现符合红色的点的个数大于总个数的0.4倍时colorFound=true，进入下一个轮廓的循环
#endif
#ifdef BLUE //蓝色同红色
            for (const auto &_reg:regions[_id]) {
                if (okPtCount > okMinCount) {
                    colorFound = true;
                    break;
                }
                if (hsv.at<cv::Vec3b>(_reg)[0] > 65 && hsv.at<cv::Vec3b>(_reg)[0] < 130 &&
                hsv.at<cv::Vec3b>(_reg)[1] > 10 &&
                cv::abs(frame.at<cv::Vec3b>(_reg)[0] - frame.at<cv::Vec3b>(_reg)[2]) > 8) {
                    okPtCount += 1;
                }
            }
            if (!colorFound) continue;

            // int sumH = std::accumulate(regions[_id].cbegin(), regions[_id].cend(), 0, [&hsv](int a, cv::Point b) {
            //     if (hsv.at<cv::Vec3b>(b)[0] > 65 && hsv.at<cv::Vec3b>(b)[0] < 140)
            //         return a + 1;
            //     else
            //         return a;
            // });  // 累加合格的点
            //
            // // sumH /= regions[_id].size();  // 百分比
            // // printf("sumH: %d \n", sumH);
            //
            // float ratio_ = (float) sumH / regions[_id].size();
            // if (ratio_ < 0.3)
            //     continue;
#endif
            idsColorFilter.emplace_back(_id); //将符合颜色的轮廓id放入idsColorFilter
        }
        // printf("\n");
        is.clock("Color Filter");
        is.addEvent("Color Filter", regions, idsColorFilter, true);

        // printf("筛颜色\n");

        // 筛长宽比, minAreaRect ///////////////////////////////////////////////////////////////////////////////////////
        is.clock("h/w");
        std::vector<Light> totalLights; //灯条容器
        for (const auto &_id : idsColorFilter) { //遍历hsv筛过的轮廓
            cv::RotatedRect rRect;
            rRect = cv::minAreaRect(regions[_id]); //获取轮廓的最小公旋转矩形
            float ratio = rRect.size.height / rRect.size.width; //高宽比
            if (ratio > 0.5 && ratio < 2.0) { //如果高宽比小于2，则进入下一循环
                continue;
            }
            
            Light light;
            cv::Point2f topPoint;
            cv::Point2f bottomPoint;
            cv::Point2f pts[4];
            rRect.points(pts); //获取旋转矩形4个顶点
            if (rRect.size.width > rRect.size.height) { //右倾
                bottomPoint = (pts[2] + pts[3]) / 2; //底部中点
                topPoint = (pts[0] + pts[1]) / 2; //顶部中点
                light.angle = cv::abs(rRect.angle);
            } else { //左倾
                bottomPoint = (pts[1] + pts[2]) / 2; //底部重点
                topPoint = (pts[0] + pts[3]) / 2; //顶部中点
                light.angle = cv::abs(rRect.angle - 90); //角度？？？
            }
            // light.topPoint = topPoint;
            // light.bottomPoint = bottomPoint;
            if (topPoint.y > bottomPoint.y) {
		        light.topPoint = bottomPoint;
		        light.bottomPoint = topPoint;
            } else {
		        light.topPoint = topPoint;
		        light.bottomPoint = bottomPoint; //选定灯条上下中心
            }
            light.length = cv::norm(bottomPoint - topPoint); //上下两中心点间距
            light.centerPoint = rRect.center; //矩形中心点
            
            /*std::cout << light.topPoint << std::endl;
            std::cout << light.bottomPoint << std::endl;
            std::cout << light.angle << std::endl;
            std::cout << std::endl;*/
            #ifndef BUBING
            if(light.bottomPoint.y < 40) continue; //保证底部中心点在图像中不能过高
            #endif

            totalLights.emplace_back(light);
        }
        is.clock("h/w");
        is.addEvent("Total Lights", totalLights, true);
        // printf("筛长宽比\n");

        // 配对 ///////////////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("pair");
#ifdef BUBING
        std::sort(totalLights.begin(), totalLights.end(), //以灯条中心的x坐标从左到右排序灯条
                  [&totalLights](Light &a, Light &b) { return a.centerPoint.x < b.centerPoint.x; });
#else
        sortLights(totalLights); //以灯条中心的x坐标从左到右排序灯条
#endif
        for (size_t i = 0; i < totalLights.size(); i++) {
        
            for (size_t j = i + 1; j < totalLights.size(); j++) {
                Light lightA = totalLights[i];
                Light lightB = totalLights[j];
                float angleL2L = cv::abs(lightA.angle - lightB.angle); //获取两灯夹角

                cv::Point2f pointTA2B = lightA.topPoint - lightB.topPoint; //两灯条顶部中点
                cv::Point2f pointBA2B = lightA.bottomPoint - lightB.bottomPoint; //两灯条底部中点
                double angleT2B = pointTA2B.dot(pointBA2B) / cv::norm(pointBA2B) / cv::norm(pointTA2B); //没用到//没看懂
                double ratioL2L = lightA.length / lightB.length; //两灯条长度之比

                double centerP2P = cv::norm(lightA.centerPoint - lightB.centerPoint); //两灯条中心点距离
                double ratioL2C = centerP2P / (lightA.length + lightB.length) * 2; //灯条中心距离与两灯条长度均值的比值

                // printf("length:%f %f %f\n", lightA.length, lightB.length, centerP2P);
                /*printf("AngleL2L:%.1f>%.1f, AngleT2B:%.3f<%.3f, RatioL2L:%.1f<>%.1f, RatioL2C:%.1f>%.1f, deltaXL2L:%.1f<%.1f x/y %.2f\n",
                       angleL2L, maxAngleL2L, angleT2B, maxAngleT2B,
                       ratioL2L, maxRatioL2L, ratioL2C, maxRatioC2L,
                       cv::abs(lightA.centerPoint.x - lightB.centerPoint.x), minDeltaXL2L, pointTA2B.x / pointTA2B.y
                );*/
                
                // 筛选灯角度差，两灯条长度比值，灯条中心距离与两灯条长度均值的比值，两灯条横向间距，左灯条倾斜角，灯条顶线的斜率
                if (angleL2L > maxAngleL2L || ratioL2L > maxRatioL2L ||
                    ratioL2L < 1 / maxRatioL2L || ratioL2C > maxRatioC2L ||
                    cv::abs(lightA.centerPoint.x - lightB.centerPoint.x) < minDeltaXL2L ||
                    cv::abs(lightA.angle + 90) < 40 || cv::abs(pointTA2B.x / pointTA2B.y) < 1.6) {
                    continue;
                }
                
                Target target;
                std::vector<cv::Point> pts;
                std::vector<cv::Point2f> pts2f;
                // 以下重排，获取灯条匹配矩形的四个顶点
                if (lightA.topPoint.x < lightB.topPoint.x) {
                    pts2f.emplace_back(lightA.topPoint); //tl
                    pts2f.emplace_back(lightA.bottomPoint); //bl
                    pts2f.emplace_back(lightB.bottomPoint); //br
                    pts2f.emplace_back(lightB.topPoint); //tr
                } else {
                    pts2f.emplace_back(lightB.topPoint);
                    pts2f.emplace_back(lightB.bottomPoint);
                    pts2f.emplace_back(lightA.bottomPoint);
                    pts2f.emplace_back(lightA.topPoint);
                }
                target.pixelPts2f = pts2f; //将顶点放入target类
                target.ratioL2C = ratioL2C;  // C / LL //灯条中心距离与两灯条长度均值的比值
                totalTargets.emplace_back(target); //将target放入totalTargets容器
                continue;
            }
        }
        is.clock("pair");
        // printf("配对\n");
    }

    // todo: 自瞄方案2
    std::vector<Target> detect2(const cv::Mat &frame) {
        // 颜色分割  ////////////////////////////////////////////////////////////////////////////////////////////////////
        cv::Mat red, blue, ycc;
        if (frame.channels() == 3)
            cv::cvtColor(frame, ycc, cv::COLOR_BGR2YCrCb);
        else {
            printf("frame's channel != 3 \n");
            exit(0);
        }
        std::vector<cv::Mat> yccs;
        cv::split(ycc, yccs);
        gray = yccs[0];
        red = yccs[1];
        blue = yccs[2];
        is.addImg("gray", gray);
        is.addImg("red", red);
        is.addImg("blue", blue);

        // threshold ///////////////////////////////////////////////////////////////////////////////////////////////////
        cv::Mat dst;
        cv::threshold(gray, dst, 120, 255, CV_THRESH_BINARY);
        is.addImg("threshold", dst);

        std::vector<Target> totalTargets;

        return totalTargets;
    }
    
    void exportCandidates(std::vector<Target> &totalTargets, std::vector<float> &result) {
        for (auto &target : totalTargets) {
            for (int i = 0; i < result.size(); i++) {
	            // 构建crop
			    //std::cout << target.pixelPts2f << std::endl;
			    cv::Mat trans_mat = cv::getPerspectiveTransform(target.pixelPts2f, armorFigureStd);
			    cv::Mat crop;
			    cv::warpPerspective(gray, crop, trans_mat, cv::Size(125, 55));
            	if (result[i] == 1) {
				    // is.addImg("crop", crop);
				    cv::imwrite("../data/sample/1/" + std::to_string(imgSaveCounter++) + ".png", crop);
			    } else {
				    cv::imwrite("../data/sample/0/" + std::to_string(imgSaveCounter++) + ".png", crop);
			    }
            }
        }
    }

    // 分类
    int classify(std::vector<Target> &totalTargets, std::vector<Target> &finalTargets) {
        if (totalTargets.empty())
            return -1;
        // 随机森林 /////////////////////////////////////////////////////////////////////////////////////////////////////
        std::vector<cv::Mat> img_lst; //图像容器
        for (const auto &target : totalTargets) { //遍历totalTargets
            // 构建crop裁图
            cv::Mat trans_mat = cv::getPerspectiveTransform(target.pixelPts2f, armorFigureStd); //仿射变换将装甲板截图变换为正图
            cv::Mat crop;
            // if (cv::mean(crop)[0] > 170) {
            //     continue;
            // };
            // getFeature(crop);
            cv::warpPerspective(gray, crop, trans_mat, cv::Size(125, 55)); //仿射变换将装甲板截图变换为正图
            // cv::warpPerspective(gray, crop, trans_mat, cv::Size(32, 32));
            // cv::resize(crop, crop, cv::Size(32, 32));
            cv::resize(crop, crop, cv::Size(64, 32)); //重置大小
            is.addImg("crop", crop); //显示crop
            img_lst.emplace_back(crop); //将crop放入容器
        }
        std::vector<cv::Mat> gradient_lst; 
        computeHOGs(img_lst, gradient_lst, false); //获得图像的梯度描述值
        cv::Mat data; //Mat to be used by OpenCV Machine Learning algorithms
        convert_to_ml(gradient_lst, data); //* Convert training/testing set to be used by OpenCV Machine Learning algorithms.
        cv::Mat result; //建立结果矩阵
        // std::vector<float> result;
        // forest->predict(data, result);
        forest->getVotes(data, result, cv::ml::DTrees::PREDICT_MAX_VOTE); //获取随机森林分类投票结果

        std::vector<Target> heroTargets;
        
        // printf("data: %d, result: %d\n", data.rows, result.rows);
        for (int i = 1; i < data.rows + 1; i++) { 
            // printf("forest: %d, %d, %d \n", result.at<int>(i, 0), result.at<int>(i, 1));
            if (result.at<int>(i, 0)  > 40) { //分类器投票结果得分大于40->即识别为装甲板
	            // printf("ratioL2C: %.2f \n", totalTargets[i].ratioL2C);
            	if (totalTargets[i-1].ratioL2C < 3.1) { //一般目标
		            totalTargets[i-1].type = 0;//赋值为0
		            finalTargets.emplace_back(totalTargets[i-1]); //放入finalTargets
	            } else {  // 如果是英雄目标
	                totalTargets[i-1].type = 1; //赋值为1
#ifdef BUBING
                	finalTargets.emplace_back(totalTargets[i-1]); //放入finalTargets
#else
                	heroTargets.emplace_back(totalTargets[i-1]); //放入heroTargets
#endif
	            }
            }
        }
        
        /*
        int qq = cv::waitKey(0);
        if (qq  == int('s')) {
        	exportCandidates(totalTargets, result);
        	printf("saved\n");
    	}
        else printf("pass\n");
        */
        /*
        for (int i = 0; i < result.size(); i++) {
            // printf("res: %.2f\n", result[i]);
            if (result[i] == 0) {
            printf("ratioL2C: %.2f \n", totalTargets[i].ratioL2C);
            	if (totalTargets[i].ratioL2C < 3.1) {
		            totalTargets[i].type = 0;
		            finalTargets.emplace_back(totalTargets[i]);
	            } else {  // 如果是英雄
	                totalTargets[i].type = 1;
#ifdef BUBING
                	finalTargets.emplace_back(totalTargets[i]);
#else
                	heroTargets.emplace_back(totalTargets[i]);
#endif
	            }
            }
        }
        */

        if (!heroTargets.empty()) {
            findHeroCounter += 1;  // 发现英雄
            if (findHeroCounter > 3) { //连续3帧发现英雄
                finalTargets = heroTargets; //直接用heroTargets覆盖finalTargets
                return 1;  // 打英雄
            } else { //未连续三帧发现英雄
                finalTargets.insert(finalTargets.end(), heroTargets.begin(), heroTargets.end()); //将heroTargets加到finalTargets后面
                return 0;  //
            }
        } else if (!finalTargets.empty()) { //未发现英雄但发现普通目标
            findHeroCounter = 0; //将英雄帧数计数器清零
            return 0;  // 发现常规单位
        }
        return -1;  // 啥也没有
    }


    // 预测函数, 二次函数拟合yaw, todo: 拟合对象选择
    // 二次拟合用的最小二乘法, 结果经 numpy.poly1d, scipy.optimize.curve_fit 验证, 结果近似
    float predict(std::deque<Target> &armorHistory) {
        const auto N = (int) armorHistory.size();  // 行数
        const int n = 2;  // 最高次数, n + 1 = 列数
        // 构造矩阵 X
        // [1, x, x^2, x^3, ...], ${采样点}行, ${最高次+1}列
        cv::Mat X = cv::Mat::zeros(N, n + 1, CV_64FC1);
        for (int i = 0; i < n + 1; i++) {
            for (int k = 0; k < N; k++) {
                X.at<double>(k, i) = std::pow(armorHistory[k].count, i);
            }
        }
        // std::cout << X << std::endl;

        // 构造矩阵 Y
        // [y0, y1, y2, ...].T
        cv::Mat Y = cv::Mat::zeros(N, 1, CV_64FC1);
        for (int k = 0; k < N; k++) {
            Y.at<double>(k, 0) = armorHistory[k].actualYaw;  // todo: 改预测对象
        }
        // std::cout << Y << std::endl;

        // 公式: X_T * X * A = X_T * Y
        cv::Mat X_t = X.t();
        cv::Mat X_inv = (X_t * X).inv();
        cv::Mat A = X_inv * X_t * Y;
        // std::cout << A << std::endl;

        double nextCount = -1.0;  // todo: 0为当前, -1为下一张图片, 改参数
        cv::Mat predict = (cv::Mat_<double>(1, n + 1) << 1, nextCount, nextCount * nextCount) * A;
        float predictYaw = (float) predict.at<double>(0);
        return predictYaw;
    }

    int track(std::vector<Target> &newTargets, float &pitch, float &yaw, bool openTrack = true) {
        // 1. 旧记录自增, 删除6次之前记录
        for (auto iter = targetsHistory.begin(); iter != targetsHistory.end();) { //遍历targetsHistory
            if (iter->count > maxHistory) { //如果count大于maxHistory
                iter = targetsHistory.erase(iter); //删除targetsHistory中的该元素
            } else {
                iter->count++; //该元素count++
                iter++;
            }
        }

        if (targetsHistory.empty() || latestShootTarget.count == -1 || !openTrack) {  // 2-a 认为还没有选定打击对象
            // 找yaw最近的
            printf("################### yaw最近的 #################################\n");
            auto closestTarget = std::min_element(newTargets.begin(), newTargets.end(),
                                                  [](Target &a, Target &b) {
                                                      return cv::abs(a.yaw) < cv::abs(b.yaw);
                                                  });
            if (closestTarget != newTargets.end()) {
                // closestTarget->length = cv::arcLength(closestTarget->pixelPts2f, true);
                // closestTarget->area = cv::contourArea(closestTarget->pixelPts2f);
                latestShootTarget = *closestTarget;  // 结构体拷贝??
                targetsHistory.emplace_back(latestShootTarget);
            }
        } else {  // 2-b 否则已经有打击对象了
            // 找到当前图片里的打击对象
            printf("################### 跟踪 #################################\n");
            float minDist = 999999;
            auto sameTarget = newTargets.end(); //？？？？？？？？？？？？？？？？？？？？？？？？？？？
            // Use a special formula to calculate，then find the minor one
            for (auto tar = newTargets.begin(); tar != newTargets.end(); tar++) {
                // float rotDist = (float) cv::norm(latestShootTarget.rvec - tar->rvec);
                // float yawDist = cv::abs(latestShootTarget.actualYaw - tar->actualYaw);
                float eularDist = (float) cv::norm(latestShootTarget.locationPt - tar->locationPt);  // 空间欧氏距离
                // 轮廓匹配, 0~1, 0, 是一模一样
                double matchShapeRes = cv::matchShapes(latestShootTarget.pixelPts2f, tar->pixelPts2f,
                                                       cv::CONTOURS_MATCH_I3, 0.0);
                // tar->length = cv::arcLength(tar->pixelPts2f, true);
                // tar->area = cv::contourArea(tar->pixelPts2f);
                // double deltaLength =
                //         cv::abs(latestShootTarget.length - tar->length) / (latestShootTarget.length + tar->length);
                // double deltaArea = cv::abs(latestShootTarget.area - tar->area) / (latestShootTarget.area + tar->area);

                // printf("yawDist: %.2f, eularDist: %.2f, rotDist: %.2f angleDist: %.2f\n", yawDist, eularDist, rotDist,
                //        angleDist, deltaLength, deltaArea);
                // printf("eularDist%.2f, matchS: %.2f, L: %.7f, S: %.7f\n", eularDist, matchShapeRes, deltaLength,
                //        deltaArea);

                // 阈值外的不要了
                if (eularDist > thresholdEularDist || matchShapeRes > 0.3) // || deltaLength > 0.01 || deltaArea > 0.02)
                    continue;

                float finalDist = eularDist + 500.0f * (float) matchShapeRes;
                if (minDist > finalDist) {
                    minDist = finalDist;
                    sameTarget = tar;
                }
            }
            if (minDist < 999999) {
                latestShootTarget = *sameTarget;
                targetsHistory.emplace_back(latestShootTarget);
            } else {
                latestShootTarget.count += 1;  // 这帧没找到, todo: 但是哨兵要停止打击
            }
        }

        // 3. 总结一下, 回传值
        if (!targetsHistory.empty()) {  // 有历史值, 说明原目标还得去瞄着
/*#ifdef PREDICT
            float predictYaw = 0.0f;
            // for (const auto &tar:targetsHistory) {
            //     printf("count: %d, yaw%.2f\n", tar.count, tar.actualYaw);
            // }

            if (targetsHistory.size() >= 5) {

# pragma omp critical
                {
                    predictYaw = predict(targetsHistory) + current_global_yaw;
                }
                float deltaYaw = predictYaw - latestShootTarget.yaw;
                if (cv::abs(deltaYaw) < 0.7)
                    deltaYaw = 0.0f;
                if (deltaYaw > 11)
                    deltaYaw = 11.0f;
                if (deltaYaw < -11)
                    deltaYaw = -11.0f;
                yaw = latestShootTarget.yaw + deltaYaw;
                printf("yaw: %.2f, predict yaw: %.2f\n", latestShootTarget.yaw, yaw);
            } else {
                yaw = latestShootTarget.yaw;
            }
#else
            yaw = latestShootTarget.yaw;
#endif*/
            yaw = latestShootTarget.yaw;
            pitch = latestShootTarget.pitch;
            is.addEvent("shoot", latestShootTarget, true);
            if (latestShootTarget.count == 0) {
                return 6;  // 这帧找到了
            } else
                return 0;  // 这帧没找到, 但还是去瞄着历史位置
        } else {
            return -1;  // 没找到, 也没的去瞄
        }
    }

    // 重置跟踪
    void clearTrack() {
        targetsHistory.clear();
        latestShootTarget.count = -1;
    }

    // 打击策略
    unsigned int shootStrategy() {
        if (targetsHistory.size() >= 2 && latestShootTarget.count == 0) {
            // todo: 改参数
            size_t targetsHistorySize = targetsHistory.size();
            printf("target.z: %.2f \n", latestShootTarget.locationPt.z);
            float shootOKYaw = 15 - 0.00208 * latestShootTarget.locationPt.z;
            shootOKYaw = shootOKYaw < 1 ? 1 : shootOKYaw;
            shootOKYaw = shootOKYaw > 14 ? 14 : shootOKYaw;
            if (targetsHistory[targetsHistorySize - 2].count == 1 &&
                latestShootTarget.locationPt.z < 5700 &&
                cv::abs(targetsHistory[targetsHistorySize - 1].yaw) < shootOKYaw &&
                cv::abs(targetsHistory[targetsHistorySize - 2].yaw) < shootOKYaw * 1.5)
                return 1;
        }
        return 0;
    }

    // 导出要送入分类器的图片, 并保存在 ../data/sample/ 下
    void exportCandidates(std::vector<Target> &totalTargets) {
        for (auto &target : totalTargets) {
            // 构建crop
           // std::cout << target.pixelPts2f << std::endl;
            cv::Mat trans_mat = cv::getPerspectiveTransform(target.pixelPts2f, armorFigureStd);
            cv::Mat crop;
            cv::warpPerspective(gray, crop, trans_mat, cv::Size(125, 55));
            // is.addImg("crop", crop);
            cv::imwrite("../data/sample/" + std::to_string(imgSaveCounter++) + "_extra0.png", crop);
        }
    }

    // 主执行函数
    int run(const cv::Mat &frame, float &pitch, float &yaw, cv::Point &startPt) {

        // 1. 检测 /////////////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("1.detect");
        std::vector<Target> totalTargets;
        detect(frame, totalTargets); //detect函数返回找到的totalTargets目标容器
        // printf("detect complete\n");
        is.addEvent("Total Targets", totalTargets);
        is.clock("1.detect");

        // 1.5 截样本  /////////////////////////////////////////////////////////////////////////////////////////////////
        // if (cvWaitKey(8) == 'j')
        // exportCandidates(totalTargets);

        // 2. 分类器 ///////////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("2.classify");
        std::vector<Target> finalTargets;
        //将totalTargets的目标放入分类器并获得finalTargets
        int classifyFlag = classify(totalTargets, finalTargets);   // -1 = None; 0 = normal; 1 = hero
        // if (classifyFlag < 0)
        //     return -1;
        // 第一次发现英雄
        if (classifyFlag > 0 && latestShootTarget.type != 1) {
            clearTrack();
        }
        // if (!finalTargets.empty()) {
        // 	cv::waitKey(0);
        // }

        // && latestShootTarget.count != -1 &&
        is.clock("2.classify");
        // 3. 计算物理坐标 //////////////////////////////////////////////////////////////////////////////////////////////
        is.clock("3.location calculate");
        for (auto &target : finalTargets) { //遍历finalTarget
            for (auto &pt : target.pixelPts2f) { //遍历finalTarget的四点坐标
                pt += (cv::Point2f) startPt; //对finalTarget的四点坐标增加偏移量
            }
            getArmorLocation(target);
            std::cout << "locationPt" << target.locationPt.z << std::endl;
        }
        is.clock("3.location calculate");
        is.addEvent("After Classify", finalTargets, true);

        // 4. 跟踪并返回: -1 = 啥也没有, 0 = 这次没找到但有历史值, 6 = 这次找到了 /////////////////////////////////////////////
        is.clock("4.track");
        int flag = track(finalTargets, pitch, yaw);
        is.clock("4.track");
        return flag;
    }

    int runPlus(const cv::Mat &frame, float &pitch, float &yaw) {
        if (latestShootTarget.count >= 0 && latestShootTarget.count <= 1) {  // 说明之前有找到目标装甲板, 类初始化的时候是-1,
            // 得到小图矩形区域
            cv::Rect latestShootBRect;
            getBoundingRect(latestShootTarget, latestShootBRect, true);
            is.addEvent("rect", latestShootBRect, true); //在非0mode下将event打印在图像上
            // 裁图
            cv::Mat crop = frame(latestShootBRect); 
            cv::Point startPoint = latestShootBRect.tl();
            int flag = run(crop, pitch, yaw, startPoint);
            if (flag < 0) {  // 小图没找到
                return run(frame, pitch, yaw, zeroPt);
            } else
                return flag;
        } else  // 上次没找到
            return run(frame, pitch, yaw, zeroPt);
    }
};

#endif  // ARMOR_DETECTARMOR_H
