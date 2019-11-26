# TJ-ROBOMASTER-AUTOMATIC-AIMING

| 作者   | 负责部分       |
|:------:|:--------------:|
| 韩煦源 |  |
| 曾宪坤 |  |

## 一、代码运行环境

|操作系统|运行库|
|-------|--------|
|Ubuntu16.04<br />Windows WSL|OpenCV 3.4.7<br />cmake 3.15.4|

- 本代码统一使用**640×480**大小的图像进行处理

## 二、程序编译及运行

### Ubuntu16.04（在项目文件夹下）

```shell

```

### 程序所用宏介绍

```C++

```

## 三、文件结构目录

``` Files Structure

```

## 四、关键类与函数解析

## 五、装甲板识别程序运行流程

## 六、代码命名规范

- 函数名：使用首字母小写的驼峰命名法
- 类型名：使用首字母大写的驼峰命名法
- 变量名：使用下划线分割命名法

## 七、问题

- main.cpp line29 什么意思:portName[11] = portName[11] == '0' ? '1' : '0';
- main.cpp line39 // log工具模式选择, 0: 纯输出, 1: 窗口带最终结果, 2: 窗口详细输出 
- main.cpp line40 什么意思:is.setCostPrint(true);  // 耗时输出
- detect_armor.hpp line83: #ifdef BUBING 定义不明
- is.clock("cvtColor"); 是什么意思
- detect_armor.hpp line293 为什么大于100赋值为100
- detect_armor.hpp line378 cv::abs(rRect.angle - 90)应该是+90
- detect_armor.hpp line423 angleT2B没有用到也没看懂是什么东西
- detect_armor.hpp line553 为什么通过ratioL2C可以判断出是否是英雄

## 八、改进

- 将所有判断mode == 0的代码全部用宏代替，减少代码调用
- 两灯条匹配矩形可以扩大到整个装甲板区域再放入分类器
