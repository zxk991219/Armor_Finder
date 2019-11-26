# TJ-ROBOMASTER-AUTOMATIC-AIMING

## 一、代码运行环境

|操作系统|运行库|
|-------|--------|
|Ubuntu16.04<br />Windows WSL|OpenCV 3.4.7<br />cmake 3.15.4|

- 本代码统一使用**640×480**大小的图像进行处理

## 二、程序编译及运行

### Ubuntu16.04（在项目文件夹下）

```shell
cd build
cmake ..
make  # make -j2
./armor  # 运行主程序
# ./train  # 运行训练程序
# ./serial_test  # 运行串口通讯测试程序
# ./record_video  # 单独录视频
```

## log工具显示窗口说明(if mode != 0)

- 绘制形状的颜色和左上角文字说明的颜色对应，同时显示数量。
- 模式为 **2** 时，有多窗口，标题栏文字是对其内容的描述。
- ***有数字的为传给上位机的目标。***
- 窗口按键绑定（即命令行内的第一句输出）：
  - `space`： 暂停/继续，暂停时循环长等待，仅空格键能继续。
  - `q`： 退出。
  - `r`： 恢复初始窗口位置大小, 即运行时窗口可拖动，每个窗口仅在创建时被 resize 和 move。

## 三、文件结构目录

``` shell
armor/..
├── build/..  # 放置编译的可执行文件
├── data/..
│   ├── train/..  # 训练文件夹
│   │   ├── 0/..  # 样本0文件夹
│   │   └── 1/..  # 样本1文件夹
│   ├── sample  # 保存送入分类器的截图，文件夹
│   ├── model.xml  # 老唐的分类器文件
│   ├── RF.xml  # 随机森林模型文件
│   ├── SVM.xml  # SVM模型文件
│   └── cam_prom.xml  # 相机参数
├── source/..
│   ├── main.cpp  # 主测试程序
│   ├── tools.hpp  # 自定义数据结构, 打包解包帧, HOG计算, 绘图等
│   ├── detect_armor.hpp  # 装甲板识别
│   ├── camera.hpp  # 相机相关函数, zdw维护
│   ├── Structure.cpp  # 共享内存读图片功能函数
│   ├── Structure.h  # 同上
│   ├── train.cpp  # 分类器训练程序
│   ├── record_video.cpp  # 录视频程序
│   ├── crc_table.hpp  # crc校验相关函数
│   ├── serial_listen.py  # Python 串口通讯函数
│   ├── serial_port.hpp  # 串口通讯类
│   ├── serial_test.cpp  # 串口通讯测试程序
│   ├── socket_communication.hpp  # 网口通讯类
│   └── socket_test.cpp  # 网口通讯测试程序
├── CMakeLists.txt
├── README.md
└── .clang-format  # clang-format配置文件
```

## 四、装甲板识别与跟踪射击程序运行流程

见“老代码自瞄流程图-韩煦源.pdf”

## 五、代码命名规范

- 函数名：使用首字母小写的驼峰命名法
- 类型名：使用首字母大写的驼峰命名法
- 变量名：使用下划线分割命名法

## 六、问题

- detect_armor.hpp line297 okMinCount没必要大于100时赋值为100
- detect_armor.hpp line423 angleT2B没有用到，我也没看懂是什么东西

## 七、改进

- 将所有判断mode == 0的代码全部用宏代替，减少if代码的调用，节省程序耗时
- 两灯条匹配矩形可以扩大到整个装甲板区域再放入分类器
