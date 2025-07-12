#include <opencv2/opencv.hpp>
#include "System.h"
#include <string>
#include <chrono>   // for time stamp
#include <iostream>
using namespace std;
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "Examples/Monocular/hnu.yaml";
string vocFile = "./Vocabulary/ORBvoc.txt";
// 视频文件，若不同请修改
string videoFile = "/media/zjj/Elements/CQU_ZJJ/hnu/slam/Video2ROSbag/25_hnu.mp4";
int main(int argc, char **argv) {
 // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, false);
    // 获取视频图像
    cv::VideoCapture cap(videoFile);    // change to 0 if you want to use USB camera.
    // 打开视频文件
    // cv::VideoCapture cap(videoFile);
    if (!cap.isOpened()) {
        cerr << "❌ 错误：无法打开视频文件 " << videoFile << endl;
        return -1;
    }
  // 记录系统时间
  auto start = chrono::system_clock::now();

while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据

        // 如果视频播放完毕，则退出循环
        if (frame.empty()) {
            cout << "🎬 视频播放完毕，程序退出。" << endl;
            break;
        }
        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(640,360));
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame_resized, double(timestamp.count())/1000.0);
        cv::waitKey(40);
    }
    SLAM.SaveKeyFrameTrajectoryTUM("trajectory.txt");
    cout << "📌 轨迹已保存至 trajectory.txt" << endl;
    return 0;


}

