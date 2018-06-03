

1. Ubuntu 下训练haar cascade classifier

​          A. Ubuntu opencv 环境配置

​               a. 自己编译opencv，安装tbb多线程，编译opencv时开启 tbb（虽然测试下来没啥用）

​                   参见  <https://github.com/BVLC/caffe/wiki/OpenCV-3.3-Installation-Guide-on-Ubuntu-16.04>

​               或者 直接安装opencv:  sudo apt-get install libopencv-dev python-opencv

​         B. 数据准备  训练数据样本包括待检测正样本数据集和负样本数据集。

​               a. 负样本数据集（背景）是只需不包含待检测物体的任意图片，图片大小大于训练窗口

​               b. 正样本数据集是包含待检测物体的图片集， 在windows下可采用framer从视屏中截取视屏帧（参见framer和framer readme.pdf，下载地址见notes(4)

​               c. 标记正样本中的感兴趣区域：在windows利用haar training压缩包内的工具，并参见 readme.pdf，下载地址见notes(4)

​               d. 生成 .vec文件： 在...\Haar Training\training文件夹中，点击sample_creation.bat（可修改），生成的.vec文件在vector文件夹中

​         C. 在ubuntu下通过opencv_trainingcascade 应用训练 haar cascade(阅读R1, R2)

​               a. 主要参见R1，git clone <https://github.com/mrnugget/opencv-haar-classifier-training> 下载训练文件夹

​               b. 将负样本放在 .../opencv-haar-classifier-training/negative_images，把正样本的样本描述文件发在.../opencv-haar-classifier-training文件夹下。

​               c. 终端索引到.../opencv-haar-classifier-training文件夹下，运行：

find ./negative_images -iname "*.jpg" > negatives.txt 然后运行

​              

2. 最终训练指令例子：

 opencv_traincascade -data classifier -vec samples.vec -bg negatives.txt\

  -numStages 20 -minHitRate 0.999 -maxFalseAlarmRate 0.5 -numPos 3600\

  -numNeg 21671 -w 20 -h 20 -mode ALL -precalcValBufSize 22528\

  -precalcIdxBufSize 22528 -maxWeakCount 1000 --numThreads 16

Notes:

[1] If you come accross any tutorial mentioning the old opencv_haartraining tool (which is deprecated and still using the OpenCV1.x interface), then please ignore that tutorial and stick to the opencv_traincascade tool. This tool is a newer version, written in C++ in accordance to the OpenCV 2.x and OpenCV 3.x API. The opencv_traincascade supports both HAAR like wavelet features and LBP (Local Binary Patterns) features. LBP features yield integer precision in contrast to HAAR features, yielding floating point precision, so both training and detection with LBP are several times faster then with HAAR features. Regarding the LBP and HAAR detection quality, it mainly depends on the training data used and the training parameters selected. It's possible to train a LBP-based classifier that will provide almost the same quality as HAAR-based one, within a percentage of the training time.

[2] The newer cascade classifier detection interface from OpenCV 2.x and OpenCV 3.x ([cv::CascadeClassifier](https://docs.opencv.org/3.3.0/d1/de5/classcv_1_1CascadeClassifier.html)) supports working with both old and new model formats. opencv_traincascade can even save (export) a trained cascade in the older format if for some reason you are stuck using the old interface. At least training the model could then be done in the most stable interface.

[3] The opencv_traincascade application can use TBB for multi-threading. To use it in multicore mode OpenCV must be built with TBB support enabled.

(4) https://pan.baidu.com/s/1NWyug1dk8yEoj3zTmuoTQA

[*] opencv_traincascade 的参数设置参见R3. 设置-numPos参数的时候注意数值小于等于正样本的一半

Reference

[1] <http://coding-robin.de/2013/07/22/train-your-own-opencv-haar-classifier.html>

[2] <http://note.sonots.com/SciSoftware/haartraining.html#w0a08ab4>

[3 ] <https://docs.opencv.org/3.3.0/dc/d88/tutorial_traincascade.html>