#pragma once

// c++
#include <iostream>
#include <math.h>

// eigen 
#include <Eigen/Core>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "lioOptimization.h"
#include "lkpyramid.h"
#include "utility.h"

#ifdef EMPTY_ANSI_COLORS
    #define ANSI_COLOR_RED ""
    #define ANSI_COLOR_RED_BOLD ""
    #define ANSI_COLOR_GREEN ""
    #define ANSI_COLOR_GREEN_BOLD ""
    #define ANSI_COLOR_YELLOW ""
    #define ANSI_COLOR_YELLOW_BOLD ""
    #define ANSI_COLOR_BLUE ""
    #define ANSI_COLOR_BLUE_BOLD ""
    #define ANSI_COLOR_MAGENTA ""
    #define ANSI_COLOR_MAGENTA_BOLD ""
#else
    #define ANSI_COLOR_RED "\x1b[0;31m"
    #define ANSI_COLOR_RED_BOLD "\x1b[1;31m"
    #define ANSI_COLOR_RED_BG "\x1b[0;41m"

    #define ANSI_COLOR_GREEN "\x1b[0;32m"
    #define ANSI_COLOR_GREEN_BOLD "\x1b[1;32m"
    #define ANSI_COLOR_GREEN_BG "\x1b[0;42m"

    #define ANSI_COLOR_YELLOW "\x1b[0;33m"
    #define ANSI_COLOR_YELLOW_BOLD "\x1b[1;33m"
    #define ANSI_COLOR_YELLOW_BG "\x1b[0;43m"

    #define ANSI_COLOR_BLUE "\x1b[0;34m"
    #define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"
    #define ANSI_COLOR_BLUE_BG "\x1b[0;44m"

    #define ANSI_COLOR_MAGENTA "\x1b[0;35m"
    #define ANSI_COLOR_MAGENTA_BOLD "\x1b[1;35m"
    #define ANSI_COLOR_MAGENTA_BG "\x1b[0;45m"

    #define ANSI_COLOR_CYAN "\x1b[0;36m"
    #define ANSI_COLOR_CYAN_BOLD "\x1b[1;36m"
    #define ANSI_COLOR_CYAN_BG "\x1b[0;46m"

    #define ANSI_COLOR_WHITE "\x1b[0;37m"
    #define ANSI_COLOR_WHITE_BOLD "\x1b[1;37m"
    #define ANSI_COLOR_WHITE_BG "\x1b[0;47m"

    #define ANSI_COLOR_BLACK "\x1b[0;30m"
    #define ANSI_COLOR_BLACK_BOLD "\x1b[1;30m"
    #define ANSI_COLOR_BLACK_BG "\x1b[0;40m"

    #define ANSI_COLOR_RESET "\x1b[0m"

    #define ANSI_DELETE_LAST_LINE "\033[A\33[2K\r"
    #define ANSI_DELETE_CURRENT_LINE "\33[2K\r"
    #define ANSI_SCREEN_FLUSH std::fflush(stdout);

    #define SET_PRINT_COLOR( a ) cout << a ;

#endif

struct _Scope_color
{
    _Scope_color( const char * color )
    {
        std::cout << color;
    }

    ~_Scope_color()
    {
        std::cout << ANSI_COLOR_RESET;
    }
};

#define scope_color(a) _Scope_color _scope(a);

class cloudFrame;
class rgbMapTracker;

class opticalFlowTracker
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cv::Mat old_image, old_gray;
	cv::Mat cur_image, cur_gray;
	cv::Mat cur_mask;

	unsigned int image_idx = 0;
	double last_image_time, current_image_time;

	std::vector<int> cur_ids, old_ids;
	unsigned int maximum_tracked_points;

	cv::Mat m_ud_map1, m_ud_map2;
	cv::Mat intrinsic, dist_coeffs;

	std::vector<cv::Point2f> last_tracked_points, cur_tracked_points;

    std::vector<void *> rgb_points_ptr_vec_in_last_image;
    std::map<void *, cv::Point2f> map_rgb_points_in_last_image_pose;
    std::map<void *, cv::Point2f> map_rgb_points_in_cur_image_pose;

    std::map<int, std::vector<cv::Point2f>> map_id_points_vec;
    std::map<int, std::vector<int>> map_id_points_image;
    std::map<int, std::vector<cv::Point2f>> map_image_points;

    Eigen::Quaterniond last_quat;
    Eigen::Vector3d last_trans;

    std::shared_ptr<LKOpticalFlowKernel> lk_optical_flow_kernel;

    opticalFlowTracker();

    void setIntrinsic(Eigen::Matrix3d intrinsic_, Eigen::Matrix<double, 5, 1> dist_coeffs_, cv::Size image_size_);

    void trackImage(cloudFrame *p_frame, double distance);

    void init(cloudFrame *p_frame, std::vector<rgbPoint*> &rgb_points_vec, std::vector<cv::Point2f> &points_2d_vec);

    void setTrackPoints(cv::Mat &image, std::vector<rgbPoint*> &rgb_points_vec, std::vector<cv::Point2f> &points_2d_vec);

    template <typename T> void reduce_vector(std::vector< T > &v, std::vector<uchar> status)
    {
        int j = 0;

        for (unsigned int i = 0; i < v.size(); i++)
            if (status[ i ])
                v[j++] = v[i];

        v.resize(j);
    }

    void updateAndAppendTrackPoints(cloudFrame *p_frame, rgbMapTracker *map_tracker, double mini_distance = 10.0, int minimum_frame_diff = 3e8);

    void rejectErrorTrackingPoints(cloudFrame *p_frame, double dis = 2.0);

    void updateLastTrackingVectorAndIds();

    bool removeOutlierUsingRansacPnp(cloudFrame *p_frame, int if_remove_ourlier = 1);

    /*
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector< cv::Mat >     m_img_vec;
    char                       m_temp_char[ 1024 ];
    cv::Mat                    m_old_frame, m_old_gray;
    cv::Mat                    frame_gray, m_current_frame;
    cv::Mat                    m_current_mask;
    unsigned int               m_frame_idx = 0;
    double                     m_last_frame_time, m_current_frame_time;
    std::vector< int >         m_current_ids, m_old_ids;
    int                        if_debug_match_img = 0;
    unsigned int               m_maximum_vio_tracked_pts = 300;
    cv::Mat                    m_ud_map1, m_ud_map2;
    cv::Mat                    m_intrinsic, m_dist_coeffs;
    std::vector< cv::Point2f > m_last_tracked_pts, m_current_tracked_pts;
    std::vector< cv::Scalar >  m_colors;
    std::vector< void * >      m_rgb_pts_ptr_vec_in_last_frame;
    std::map< void *, cv::Point2f > m_map_rgb_pts_in_last_frame_pos;
    std::map< void *, cv::Point2f > m_map_rgb_pts_in_current_frame_pos;

    std::map< int, std::vector< cv::Point2f > > m_map_id_pts_vec;
    std::map< int, std::vector< int > >         m_map_id_pts_frame;
    std::map< int, std::vector< cv::Point2f > > m_map_frame_pts;
    cv::Mat                                   m_debug_track_img;
    eigen_q                                   q_last_estimated_q = eigen_q::Identity();
    vec_3                                     t_last_estimated = vec_3( 0, 0, 0 );
    std::shared_ptr< LKOpticalFlowKernel > m_lk_optical_flow_kernel;
    Rgbmap_tracker();
    ~Rgbmap_tracker(){};

    void set_intrinsic( Eigen::Matrix3d cam_K, Eigen::Matrix< double, 5, 1 > dist, cv::Size imageSize )
    {
        cv::eigen2cv( cam_K, m_intrinsic );
        cv::eigen2cv( dist, m_dist_coeffs );
        initUndistortRectifyMap( m_intrinsic, m_dist_coeffs, cv::Mat(), m_intrinsic, imageSize, CV_16SC2, m_ud_map1, m_ud_map2 );
    }

    void update_last_tracking_vector_and_ids()
    {
        int idx = 0;
        m_last_tracked_pts.clear();
        m_rgb_pts_ptr_vec_in_last_frame.clear();
        m_colors.clear();
        m_old_ids.clear();
        for ( auto it = m_map_rgb_pts_in_last_frame_pos.begin(); it != m_map_rgb_pts_in_last_frame_pos.end(); it++ )
        {
            m_rgb_pts_ptr_vec_in_last_frame.push_back( it->first );
            m_last_tracked_pts.push_back( it->second );
            m_colors.push_back( ( ( RGB_pts * ) it->first )->m_dbg_color );
            m_old_ids.push_back( idx );
            idx++;
        }
    }

    void set_track_pts( cv::Mat &img, std::vector< std::shared_ptr< RGB_pts > > &rgb_pts_vec, std::vector< cv::Point2f > &pts_proj_img_vec )
    {
        m_old_frame = img.clone();
        cv::cvtColor( m_old_frame, m_old_gray, cv::COLOR_BGR2GRAY );
        m_map_rgb_pts_in_last_frame_pos.clear();
        for ( unsigned int i = 0; i < rgb_pts_vec.size(); i++ )
        {
            m_map_rgb_pts_in_last_frame_pos[ ( void * ) rgb_pts_vec[ i ].get() ] = pts_proj_img_vec[ i ];
        }
        update_last_tracking_vector_and_ids();
    }

    void init( const std::shared_ptr< Image_frame > &img_with_pose, std::vector< std::shared_ptr< RGB_pts > > &rgb_pts_vec, std::vector< cv::Point2f > &pts_proj_img_vec )
    {
        set_track_pts( img_with_pose->m_img, rgb_pts_vec, pts_proj_img_vec );
        m_current_frame_time = img_with_pose->m_timestamp;
        m_last_frame_time = m_current_frame_time;
        std::vector< uchar > status;
        m_lk_optical_flow_kernel->track_image( img_with_pose->m_img_gray, m_last_tracked_pts, m_current_tracked_pts, status );
    }

    void update_points( std::vector< cv::Point2f > &pts_vec, std::vector< int > &pts_ids )
    {
        for ( unsigned int i = 0; i < pts_vec.size(); i++ )
        {
            m_map_id_pts_vec[ pts_ids[ i ] ].push_back( pts_vec[ i ] );
            m_map_id_pts_frame[ pts_ids[ i ] ].push_back( m_frame_idx );
            m_map_frame_pts[ m_frame_idx ].push_back( pts_vec[ i ] );
        }
    }

    void undistort_image( cv::Mat &img )
    {
        cv::Mat temp_img;
        temp_img = img.clone();
        remap( temp_img, img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR );
    }

    void update_and_append_track_pts( std::shared_ptr< Image_frame > &img_pose, Global_map &map_rgb, double mini_dis = 10.0, int minimum_frame_diff = 3e8 );
    void reject_error_tracking_pts( std::shared_ptr< Image_frame > &img_pose, double dis = 2.0 );

    template < typename T > void reduce_vector( std::vector< T > &v, std::vector< uchar > status )
    {
        int j = 0;
        for ( unsigned int i = 0; i < v.size(); i++ )
            if ( status[ i ] )
                v[ j++ ] = v[ i ];
        v.resize( j );
    }

    void track_img( std::shared_ptr< Image_frame > &img_pose, double dis = 2.0, int if_use_opencv = 1 );
    int get_all_tracked_pts( std::vector< std::vector< cv::Point2f > > *img_pt_vec = nullptr );
    int remove_outlier_using_ransac_pnp( std::shared_ptr< Image_frame > &img_pose, int if_remove_ourlier = 1 );
    */
};