#include "linefeature_tracker.h"
// #include "line_descriptor/src/precomp_custom.hpp"

LineFeatureTracker::LineFeatureTracker()
{
    allfeature_cnt = 0;
    frame_cnt = 0;
    sum_time = 0.0;
}
//line
void LineFeatureTracker::readIntrinsicParameterline(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());

    n_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    K_ = n_camera->initUndistortRectifyMap(undist_map1_, undist_map2_);
}

// vector<Line> LineFeatureTracker::undistortedLineEndPoints()
// {
//     vector<Line> un_lines;
//     un_lines = curframe_->vecLine;
//     float fx = K_.at<float>(0, 0);
//     float fy = K_.at<float>(1, 1);
//     float cx = K_.at<float>(0, 2);
//     float cy = K_.at<float>(1, 2);
//     for (unsigned int i = 0; i < curframe_->vecLine.size(); i++)
//     {
//         un_lines[i].StartPt.x = (curframe_->vecLine[i].StartPt.x - cx) / fx;
//         un_lines[i].StartPt.y = (curframe_->vecLine[i].StartPt.y - cy) / fy;
//         un_lines[i].EndPt.x = (curframe_->vecLine[i].EndPt.x - cx) / fx;
//         un_lines[i].EndPt.y = (curframe_->vecLine[i].EndPt.y - cy) / fy;
//     }
//     return un_lines;
// }

cv::Mat last_unsuccess_image;
vector<KeyLine> last_unsuccess_keylsd;
vector<int> last_unsuccess_id;
Mat last_unsuccess_lbd_descr;

//void LineFeatureTracker::readImage(const cv::Mat &_img)
//const map<int, vector<pair<int, Vector4d>>>

map<int, vector<pair<int, Vector4d>>> LineFeatureTracker::trackImagewithline(double _cur_time, const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_p;
    frame_cnt++;

    cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);

    //    cv::imshow("lineimg",img);
    //    cv::waitKey(1);
    //ROS_INFO("undistortImage costs: %fms", t_p.toc());
    if (EQUALIZE) // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines); // 初始化一个新的帧
        forwframe_->img = img;
    }
    TicToc t_li;
    Ptr<line_descriptor::LSDDetectorC> lsd_ = line_descriptor::LSDDetectorC::createLSDDetectorC();
    // lsd parameters
    line_descriptor::LSDDetectorC::LSDOptions opts;
    opts.refine = 1;                //1     	The way found lines will be refined
    opts.scale = 0.5;               //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
    opts.sigma_scale = 0.6;         //0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
    opts.quant = 2.0;               //2.0   Bound to the quantization error on the gradient norm
    opts.ang_th = 22.5;             //22.5	Gradient angle tolerance in degrees
    opts.log_eps = 1.0;             //0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
    opts.density_th = 0.6;          //0.7	Minimal density of aligned region points in the enclosing rectangle.
    opts.n_bins = 1024;             //1024 	Number of bins in pseudo-ordering of gradient modulus.
    double min_line_length = 0.125; // Line segments shorter than that are rejected
    // opts.refine       = 1;
    // opts.scale        = 0.5;
    // opts.sigma_scale  = 0.6;
    // opts.quant        = 2.0;
    // opts.ang_th       = 22.5;
    // opts.log_eps      = 1.0;
    // opts.density_th   = 0.6;
    // opts.n_bins       = 1024;
    // double min_line_length = 0.125;
    opts.min_length = min_line_length * (std::min(img.cols, img.rows));

    std::vector<KeyLine> lsd, keylsd;
    //void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, int scale, int numOctaves, const std::vector<Mat>& masks ) const
    lsd_->detect(img, lsd, 2, 1, opts);
    // visualize_line(img,lsd);
    // step 1: line extraction
    // TicToc t_li;
    // std::vector<KeyLine> lsd, keylsd;
    // Ptr<LSDDetector> lsd_;
    // lsd_ = cv::line_descriptor::LSDDetector::createLSDDetector();
    // lsd_->detect( img, lsd, 2, 2 );

    sum_time += t_li.toc();
    //ROS_INFO("line detect costs: %fms", t_li.toc());

    Mat lbd_descr, keylbd_descr;
    // step 2: lbd descriptor
    TicToc t_lbd;
    Ptr<BinaryDescriptor> bd_ = BinaryDescriptor::createBinaryDescriptor();

    bd_->compute(img, lsd, lbd_descr);
    // std::cout<<"lbd_descr = "<<lbd_descr.size()<<std::endl;
    //////////////////////////
    for (int i = 0; i < (int)lsd.size(); i++)
    {
        if (lsd[i].octave == 0 && lsd[i].lineLength >= 60)
        {
            keylsd.push_back(lsd[i]);
            keylbd_descr.push_back(lbd_descr.row(i));
        }
    }
    // std::cout<<"lbd_descr = "<<lbd_descr.size()<<std::endl;
    //    ROS_INFO("lbd_descr detect costs: %fms", keylsd.size() * t_lbd.toc() / lsd.size() );
    sum_time += keylsd.size() * t_lbd.toc() / lsd.size();
    ///////////////

    forwframe_->keylsd = keylsd;
    forwframe_->lbd_descr = keylbd_descr;

    for (size_t i = 0; i < forwframe_->keylsd.size(); ++i)
    {
        if (first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1); // give a negative id
    }

    // if(!first_img)
    // {
    //     std::vector<DMatch> lsd_matches;
    //     Ptr<BinaryDescriptorMatcher> bdm_;
    //     bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
    //     bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);
    //     visualize_line_match(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, lsd_matches);
    //     // std::cout<<"lsd_matches = "<<lsd_matches.size()<<" forwframe_->keylsd = "<<keylbd_descr.size()<<" curframe_->keylsd = "<<keylbd_descr.size()<<std::endl;
    // }

    if (curframe_->keylsd.size() > 0)
    {
        /* compute matches */
        TicToc t_match;
        std::vector<DMatch> lsd_matches;
        Ptr<BinaryDescriptorMatcher> bdm_;
        bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);
        //        ROS_INFO("lbd_macht costs: %fms", t_match.toc());
        sum_time += t_match.toc();
        mean_time = sum_time / frame_cnt;
        // ROS_INFO("line feature tracker mean costs: %fms", mean_time);
        /* select best matches */
        std::vector<DMatch> good_matches;
        std::vector<KeyLine> good_Keylines;
        good_matches.clear();
        for (int i = 0; i < (int)lsd_matches.size(); i++)
        {
            if (lsd_matches[i].distance < 30)
            {

                DMatch mt = lsd_matches[i];
                KeyLine line1 = forwframe_->keylsd[mt.queryIdx];
                KeyLine line2 = curframe_->keylsd[mt.trainIdx];
                Point2f serr = line1.getStartPoint() - line2.getEndPoint();
                Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
                // std::cout<<"11111111111111111 = "<<abs(line1.angle-line2.angle)<<std::endl;
                if ((serr.dot(serr) < 200 * 200) && (eerr.dot(eerr) < 200 * 200) && abs(line1.angle - line2.angle) < 0.1) // 线段在图像里不会跑得特别远
                    good_matches.push_back(lsd_matches[i]);
            }
        }

        vector<int> success_id;
        // std::cout << forwframe_->lineID.size() <<" " <<curframe_->lineID.size();
        for (int k = 0; k < good_matches.size(); ++k)
        {
            DMatch mt = good_matches[k];
            forwframe_->lineID[mt.queryIdx] = curframe_->lineID[mt.trainIdx];
            success_id.push_back(curframe_->lineID[mt.trainIdx]);
        }

        //visualize_line_match(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, good_matches);

        //把没追踪到的线存起来

        vector<KeyLine> vecLine_tracked, vecLine_new;
        vector<int> lineID_tracked, lineID_new;
        Mat DEscr_tracked, Descr_new;
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->keylsd.size(); ++i)
        {
            if (forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.push_back(forwframe_->keylsd[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
                Descr_new.push_back(forwframe_->lbd_descr.row(i));
            }

            else
            {
                vecLine_tracked.push_back(forwframe_->keylsd[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
                DEscr_tracked.push_back(forwframe_->lbd_descr.row(i));
            }
        }

        vector<KeyLine> h_Line_new, v_Line_new;
        vector<int> h_lineID_new, v_lineID_new;
        Mat h_Descr_new, v_Descr_new;
        for (size_t i = 0; i < vecLine_new.size(); ++i)
        {
            if ((((vecLine_new[i].angle >= 3.14 / 4 && vecLine_new[i].angle <= 3 * 3.14 / 4)) || (vecLine_new[i].angle <= -3.14 / 4 && vecLine_new[i].angle >= -3 * 3.14 / 4)))
            {
                h_Line_new.push_back(vecLine_new[i]);
                h_lineID_new.push_back(lineID_new[i]);
                h_Descr_new.push_back(Descr_new.row(i));
            }
            else
            {
                v_Line_new.push_back(vecLine_new[i]);
                v_lineID_new.push_back(lineID_new[i]);
                v_Descr_new.push_back(Descr_new.row(i));
            }
        }
        int h_line, v_line;
        h_line = v_line = 0;
        for (size_t i = 0; i < vecLine_tracked.size(); ++i)
        {
            if ((((vecLine_tracked[i].angle >= 3.14 / 4 && vecLine_tracked[i].angle <= 3 * 3.14 / 4)) || (vecLine_tracked[i].angle <= -3.14 / 4 && vecLine_tracked[i].angle >= -3 * 3.14 / 4)))
            {
                h_line++;
            }
            else
            {
                v_line++;
            }
        }
        int diff_h = 35 - h_line;
        int diff_v = 35 - v_line;

        // std::cout<<"h_line = "<<h_line<<" v_line = "<<v_line<<std::endl;
        if (diff_h > 0) // 补充线条
        {
            int kkk = 1;
            if (diff_h > h_Line_new.size())
                diff_h = h_Line_new.size();
            else
                kkk = int(h_Line_new.size() / diff_h);
            for (int k = 0; k < diff_h; ++k)
            {
                vecLine_tracked.push_back(h_Line_new[k]);
                lineID_tracked.push_back(h_lineID_new[k]);
                DEscr_tracked.push_back(h_Descr_new.row(k));
            }
            // std::cout  <<"h_kkk = " <<kkk<<" diff_h = "<<diff_h<<" h_Line_new.size() = "<<h_Line_new.size()<<std::endl;
        }
        if (diff_v > 0) // 补充线条
        {
            int kkk = 1;
            if (diff_v > v_Line_new.size())
                diff_v = v_Line_new.size();
            else
                kkk = int(v_Line_new.size() / diff_v);
            for (int k = 0; k < diff_v; ++k)
            {
                vecLine_tracked.push_back(v_Line_new[k]);
                lineID_tracked.push_back(v_lineID_new[k]);
                DEscr_tracked.push_back(v_Descr_new.row(k));
            } // std::cout  <<"v_kkk = " <<kkk<<" diff_v = "<<diff_v<<" v_Line_new.size() = "<<v_Line_new.size()<<std::endl;
        }
        // int diff_n = 50 - vecLine_tracked.size();  // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
        // if( diff_n > 0)    // 补充线条
        // {
        //     for (int k = 0; k < vecLine_new.size(); ++k) {
        //         vecLine_tracked.push_back(vecLine_new[k]);
        //         lineID_tracked.push_back(lineID_new[k]);
        //         DEscr_tracked.push_back(Descr_new.row(k));
        //     }
        // }

        forwframe_->keylsd = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;
        forwframe_->lbd_descr = DEscr_tracked;
    }

    // 将opencv的KeyLine数据转为季哥的Line
    for (int j = 0; j < forwframe_->keylsd.size(); ++j)
    {
        Line l;
        KeyLine lsd = forwframe_->keylsd[j];
        l.StartPt = lsd.getStartPoint();
        l.EndPt = lsd.getEndPoint();
        l.length = lsd.lineLength;
        forwframe_->vecLine.push_back(l);
    }
    curframe_ = forwframe_;



    //visualization
    vector<Line> visual_lines;
    visual_lines = curframe_->vecLine;
    for (unsigned int i = 0; i < curframe_->vecLine.size(); i++)
    {
        visual_lines[i].StartPt.x = curframe_->vecLine[i].StartPt.x;
        visual_lines[i].StartPt.y = curframe_->vecLine[i].StartPt.y;
        visual_lines[i].EndPt.x = curframe_->vecLine[i].EndPt.x;
        visual_lines[i].EndPt.y = curframe_->vecLine[i].EndPt.y;
    }
    if (SHOW_TRACK)
        drawTrackline(img, visual_lines);



    //my new below
    vector<Line> un_lines;
    un_lines = curframe_->vecLine;
    float fx = K_.at<float>(0, 0);
    float fy = K_.at<float>(1, 1);
    float cx = K_.at<float>(0, 2);
    float cy = K_.at<float>(1, 2);
    for (unsigned int i = 0; i < curframe_->vecLine.size(); i++)
    {
        un_lines[i].StartPt.x = (curframe_->vecLine[i].StartPt.x - cx) / fx;
        un_lines[i].StartPt.y = (curframe_->vecLine[i].StartPt.y - cy) / fy;
        un_lines[i].EndPt.x = (curframe_->vecLine[i].EndPt.x - cx) / fx;
        un_lines[i].EndPt.y = (curframe_->vecLine[i].EndPt.y - cy) / fy;
    }

    //-----------------------------------------------------------------------------
    //map<int, vector<pair<int, Eigen::Matrix<double, 5, 1>>>> linefeatureFrame; //line
    map<int, vector<pair<int, Vector4d>>> linefeatureFrame;

    //auto &cur_lines = trackerData.curframe_->vecLine;
    auto &ids = curframe_->lineID;
    vector<set<int>> hash_ids(1); //num of cam

    for (unsigned int j = 0; j < ids.size(); j++)
    {

        int p_id = ids[j];
        hash_ids[0].insert(p_id);
        //geometry_msgs::Point32 p;
        // p.x = un_lines[j].StartPt.x;
        // p.y = un_lines[j].StartPt.y;
        // p.z = 1;
        double px = un_lines[j].StartPt.x;
        double py = un_lines[j].StartPt.y;

        // feature_lines->points.push_back(p);
        // id_of_line.values.push_back(p_id * NUM_OF_CAM + i);

        // u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
        // v_of_endpoint.values.push_back(un_lines[j].EndPt.y);

        //test
        Vector4d input;
        input << px, py, un_lines[j].EndPt.x, un_lines[j].EndPt.y;
        int camera_line_id = 0;
        linefeatureFrame[p_id].emplace_back(camera_line_id, input);
        //cout<<px<<" "<<py<<un_lines[j].EndPt.x<<un_lines[j].EndPt.y<<endl;
    }

    return linefeatureFrame;
}

void LineFeatureTracker::drawTrackline(cv::Mat &img, vector<Line> visual_lines)
{
    imTrackline = img.clone();
    cv::cvtColor(imTrackline, imTrackline, CV_GRAY2RGB);

    for (int i = 0; i < visual_lines.size(); i++)
    {

        cv::line(imTrackline, visual_lines[i].StartPt, visual_lines[i].EndPt, cv::Scalar(0, 0, 255), 2, 8);
    }
    cv::imshow("line_tracking", imTrackline);
    cv::waitKey(2);

    // //int rows = imLeft.rows;
    // int cols = imLeft.cols;
    // if (!imRight.empty() && (stereo_cam))
    //     cv::hconcat(imLeft, imRight, imTrack);
    // else
    //     imTrack = imLeft.clone();
    // cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    // for (size_t j = 0; j < curLeftPts.size(); j++)
    // {
    //     double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
    //     cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2); //point
    // }
    // if (!imRight.empty() && (stereo_cam))
    // {
    //     for (size_t i = 0; i < curRightPts.size(); i++)
    //     {
    //         cv::Point2f rightPt = curRightPts[i];
    //         rightPt.x += cols;
    //         cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
    //         //cv::Point2f leftPt = curLeftPtsTrackRight[i];
    //         //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
    //     }
    // }

    // map<int, cv::Point2f>::iterator mapIt;
    // for (size_t i = 0; i < curLeftIds.size(); i++)
    // {
    //     int id = curLeftIds[i];
    //     mapIt = prevLeftPtsMap.find(id);
    //     if (mapIt != prevLeftPtsMap.end())
    //     {
    //         cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2); //green line shows changes
    //     }
    // }

    // //draw prediction
    // /*
    // for(size_t i = 0; i < predict_pts_debug.size(); i++)
    // {
    //     cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    // }
    // */
    // //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    // //cv::Mat imCur2Compress;
    // //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));

    // cv::imshow("point_tracking", imTrack);
    // cv::waitKey(2);
}
