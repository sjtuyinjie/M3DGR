/**
* ROS driver for DM-VIO written by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
* This file is in part based on the file main_dso_pangolin.cpp of the project DSO as well as the ROS driver for DSO,
* both written by Jakob Engel.
*
* Copyright 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <locale.h>
#include <stdlib.h>
#include <stdio.h>

#include "IOWrapper/Output3DWrapper.h"

#include "util/Undistort.h"


#include <boost/thread.hpp>
#include "dso/util/settings.h"
#include "dso/util/globalCalib.h"
#include "util/TimeMeasurement.h"

#include "FullSystem/FullSystem.h"

#include <util/SettingsUtil.h>

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include "util/MainSettings.h"
#include "live/FrameSkippingStrategy.h"
#include "live/IMUInterpolator.h"
#include "ROSOutputWrapper.h"

#include <live/FrameContainer.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#ifdef STACKTRACE

#include <boost/stacktrace.hpp>
#include <signal.h>

#endif

using namespace dso;

#ifdef STACKTRACE

void segfault_handler(int sig)
{
    std::cerr << "Caught signal " << sig << ", printing stacktrace:" << std::endl;
    std::cerr << boost::stacktrace::stacktrace() << std::endl;
    exit(1);
}

#endif

dmvio::FrameContainer frameContainer;
dmvio::IMUInterpolator imuInt(frameContainer, nullptr);
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;
dmvio::FrameSkippingSettings frameSkippingSettings;
std::unique_ptr<Undistort> undistorter;
bool stopSystem = false;
int start = 2;
double timeshift = 0.0;
std::string rosbagFile = "";
bool loadRosbagThread = false;
bool finishedLoading = false;

std::string imageTopic, imuTopic;

void vidCb(const sensor_msgs::ImageConstPtr img);
void imuCb(const sensor_msgs::ImuConstPtr imu);
void loadFromRosbag();

void run(IOWrap::PangolinDSOViewer* viewer)
{
    bool linearizeOperation = rosbagFile != ""; // linearize operation (non-rt) if we load from rosbag.
    if(linearizeOperation && setting_minFramesBetweenKeyframes < 0)
    {
        setting_minFramesBetweenKeyframes = -setting_minFramesBetweenKeyframes;
        std::cout << "Using setting_minFramesBetweenKeyframes=" << setting_minFramesBetweenKeyframes
                  << " because of non-realtime mode." << std::endl;
    }

    std::thread rosbagLoadThread;
    if(rosbagFile != "")
    {
        std::cout << "Loading images from ROSBAG instead of subscribing to the topics." << std::endl;
        if(loadRosbagThread)
        {
            rosbagLoadThread = std::thread(loadFromRosbag);
        }else
        {
            std::cout << "Loading all ROS messages..." << std::endl;
            loadFromRosbag();
            std::cout << "Finished loading." << std::endl;
        }
    }

    auto fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);

    if(setting_photometricCalibration > 0 && undistorter->photometricUndist == nullptr)
    {
        printf("ERROR: dont't have photometric calibration. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }

    if(undistorter->photometricUndist != nullptr)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    if(viewer)
    {
        fullSystem->outputWrapper.push_back(viewer);
    }

    dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);
    // frameSkipping registers as an outputWrapper to get notified of changes of the system status.
    fullSystem->outputWrapper.push_back(&frameSkipping);

    // This will handle publishing to ROS topics.
    dmvio::ROSOutputWrapper rosOutput;
    fullSystem->outputWrapper.push_back(&rosOutput);

    int ii = 0;
    int lastResetIndex = 0;

    while(!stopSystem)
    {
        // Skip the first few frames if the start variable is set.
        if(start > 0 && ii < start)
        {
            auto pair = frameContainer.getImageAndIMUData(0);

            ++ii;
            continue;
        }


        int numSkipFrames = frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize());
        if(rosbagFile != "")
        {
            numSkipFrames = 0; // never skip frames when loading from rosbag.
        }
        auto pair = frameContainer.getImageAndIMUData(numSkipFrames);

        if(!pair.first) continue;

        fullSystem->addActiveFrame(pair.first.get(), ii, &(pair.second), nullptr);

        if(fullSystem->initFailed || setting_fullResetRequested)
        {
            if(ii - lastResetIndex < 250 || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                fullSystem.reset();
                for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

                fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
                if(undistorter->photometricUndist != nullptr)
                {
                    fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
                }
                fullSystem->outputWrapper = wraps;

                setting_fullResetRequested = false;
                lastResetIndex = ii;
            }
        }

        if(viewer != nullptr && viewer->shouldQuit())
        {
            std::cout << "User closed window -> Quit!" << std::endl;
            break;
        }

        if(fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }

        ++ii;

        // If running from ROSBAG exit automatically when finished.
        if(rosbagFile != "" && finishedLoading && frameContainer.getQueueSize() == 0)
        {
            break;
        }

    }

    fullSystem->blockUntilMappingIsFinished();

    fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);
    fullSystem->printResult(imuSettings.resultsPrefix + "resultScaled.txt", false, true, true);

    dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
    }

    if(rosbagLoadThread.joinable()) rosbagLoadThread.join();

    printf("DELETE FULLSYSTEM!\n");
    fullSystem.reset();

    ros::shutdown();

    printf("EXIT NOW!\n");
}

double convertStamp(const ros::Time& time)
{
    // We need the timstamp in seconds as double
    return time.sec * 1.0 + time.nsec / 1000000000.0;
}

void vidCb(const sensor_msgs::ImageConstPtr img)
{
    double stamp = convertStamp(img->header.stamp) + timeshift;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    MinimalImageB minImg((int) cv_ptr->image.cols, (int) cv_ptr->image.rows, (unsigned char*) cv_ptr->image.data);
    // Unfortunately the image message does not contain exposure. This means that you cannot use photometric
    // mode 1. But mode 0 will entirely disable the vignette which is far from optimal for fisheye cameras.
    // You can use the new mode 3 however which uses vignette, but does not assume that a full photometric
    // calibration is available.
    // Alternatively, if exposure is published on a different topic you can synchronize them an pass the exposure to
    // undistorter->undistort in the next line.
    std::unique_ptr<ImageAndExposure> undistImg(undistorter->undistort<unsigned char>(&minImg, 1.0, stamp, 1.0f));

    imuInt.addImage(std::move(undistImg), stamp);
}

void imuCb(const sensor_msgs::ImuConstPtr imu)
{
    std::vector<float> accData;
    accData.push_back(imu->linear_acceleration.x);
    accData.push_back(imu->linear_acceleration.y);
    accData.push_back(imu->linear_acceleration.z);

    std::vector<float> gyrData;
    gyrData.push_back(imu->angular_velocity.x);
    gyrData.push_back(imu->angular_velocity.y);
    gyrData.push_back(imu->angular_velocity.z);

    ros::Time time = imu->header.stamp;
    double timestamp = convertStamp(time);
    imuInt.addAccData(accData, timestamp);
    imuInt.addGyrData(gyrData, timestamp);
}

void loadFromRosbag()
{
    dmvio::TimeMeasurement meas("RosbagLoading");
    rosbag::Bag bag;
    bag.open(rosbagFile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(imageTopic);
    topics.push_back(imuTopic);
    for(auto&& m : rosbag::View(bag, rosbag::TopicQuery(topics)))
    {
        if(m.getTopic() == imageTopic || ("/" + m.getTopic() == imageTopic))
        {
            sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
            if(img != nullptr)
            {
                vidCb(img);
            }
        }else if(m.getTopic() == imuTopic || ("/" + m.getTopic() == imuTopic))
        {
            sensor_msgs::ImuConstPtr imuData = m.instantiate<sensor_msgs::Imu>();
            if(imuData != nullptr)
            {
                imuCb(imuData);
            }
        }
    }
    bag.close();
    finishedLoading = true;
}

int main(int argc, char** argv)
{
#ifdef STACKTRACE
    // Set up the segfault handler
    struct sigaction sa;
    sa.sa_handler = segfault_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigaction(SIGSEGV, &sa, NULL);
#endif

    ros::init(argc, argv, "DMVIO_ros");
    ros::NodeHandle nh;

    setlocale(LC_ALL, "C");

#ifdef DEBUG
    std::cout << "DEBUG MODE!" << std::endl;
#endif

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    // Create Settings files.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);
    frameSkippingSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("start", start);

    auto normalizeCamSize = std::make_shared<double>(0.0);
    settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);
    // timeshift from camera to imu: timestamp_imu = timestamp_cam + timeshift
    settingsUtil->registerArg("timeshift", timeshift);
    // If set, data is loaded from rosbag instead of subscribing live.
    settingsUtil->registerArg("rosbag", rosbagFile);
    // If true, data is loaded from rosbag in separate thread. (Only active if rosbagFile is set).
    settingsUtil->registerArg("loadRosbagThread", loadRosbagThread);

    // This call will parse all commandline arguments and potentially also read a settings yaml file if passed.
    mainSettings.parseArguments(argc, argv, *settingsUtil);

    // Print settings to commandline and file.
    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    undistorter.reset(
            Undistort::getUndistorterForFile(mainSettings.calib, mainSettings.gammaCalib, mainSettings.vignette));

    setGlobalCalib(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    imuCalibration.loadFromFile(mainSettings.imuCalibFile);

    std::unique_ptr<IOWrap::PangolinDSOViewer> viewer;

    if(!disableAllDisplay)
    {
        viewer = std::make_unique<IOWrap::PangolinDSOViewer>(wG[0], hG[0], true, settingsUtil, normalizeCamSize);
    }

    imageTopic = nh.resolveName("/camera/color/image_raw");
    imuTopic = nh.resolveName("/camera/imu");
    // imageTopic = nh.resolveName("/camera/color/image_raw");
    // imuTopic = nh.resolveName("/camera/imu");
    std::cout << "Image topic: " << imageTopic << std::endl;
    std::cout << "IMU topic: " << imuTopic << std::endl;

    std::thread runThread = std::thread(boost::bind(run, viewer.get()));

    ros::Subscriber imageSub;
    ros::Subscriber imuSub;
    if(rosbagFile == "")
    {
        imageSub = nh.subscribe("/camera/color/image_raw", 3, &vidCb);
        imuSub = nh.subscribe("/camera/imu", 50, &imuCb);
    }

    ros::spin();
    stopSystem = true;
    frameContainer.stop();

    // Make sure that the destructor of FullSystem, etc. finishes, so all log files are properly flushed.
    runThread.join();

    return 0;
}