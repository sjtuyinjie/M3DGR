#include "cloudMap.h"

cv::RNG g_rng = cv::RNG(0);

rgbPoint::rgbPoint(const Eigen::Vector3d &position_)
{
	position = position_.cast<float>();
	reset();
}

void rgbPoint::reset()
{
	for (int i = 0; i < 3; i++) rgb[i] = 0;

    N_rgb = 0;
    is_out_lier_count = 0;
    observe_distance = 0;
    last_observe_time = 0;
}

void rgbPoint::setPosition(const Eigen::Vector3d &position_)
{
	position = position_.cast<float>();
}

Eigen::Vector3d rgbPoint::getPosition()
{
	return position.cast<double>();
}

Eigen::Matrix3d rgbPoint::getCovRgb()
{
	Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();

	for (int i = 0; i < 3; i++)
		cov_mat(i, i) = cov_rgb(i, 0);

	return cov_mat;
}

Eigen::Vector3d rgbPoint::getRgb()
{
    return Eigen::Vector3d(rgb[0], rgb[1], rgb[2]);
}

pcl::PointXYZI rgbPoint::getPositionROS()
{
    pcl::PointXYZI point;
    point.x = position[0];
    point.y = position[1];
    point.z = position[2];

    return point;
}

const double image_obs_cov = 15;
const double process_noise_sigma = 0.1;

int rgbPoint::updateRgb(const Eigen::Vector3d &rgb_, const double observe_distance_, const Eigen::Vector3d observe_sigma_, const double observe_time_)
{
    if (observe_distance != 0 && (observe_distance_ > observe_distance * 1.2))
    {
        return 0;
    }

    if (N_rgb == 0)
    {
        last_observe_time = observe_time_;
        observe_distance = observe_distance_;

        for (int i = 0; i < 3; i++)
        {
            rgb[0] = round(rgb_[0]);
            rgb[1] = round(rgb_[1]);
            rgb[2] = round(rgb_[2]);
            cov_rgb = observe_sigma_.cast<float>();
        }

        N_rgb = 1;
        return 0;
    }

    for(int i = 0 ; i < 3; i++)
    {
        cov_rgb(i) = (cov_rgb(i, 0) + process_noise_sigma * (observe_time_ - last_observe_time)); // Add process noise
        double old_sigma = cov_rgb(i, 0);
        cov_rgb(i) = sqrt(1.0 / (1.0 / (cov_rgb(i) * cov_rgb(i)) + 1.0 / (observe_sigma_(i) * observe_sigma_(i))));
        rgb[i] = cov_rgb(i) * cov_rgb(i) * (rgb[i] / (old_sigma * old_sigma) + rgb_(i) / (observe_sigma_(i) * observe_sigma_(i)));
    }

    if (observe_distance_ < observe_distance)
    {
        observe_distance = observe_distance_;
    }

    last_observe_time = observe_time_;
    N_rgb++;

    return 1;
}