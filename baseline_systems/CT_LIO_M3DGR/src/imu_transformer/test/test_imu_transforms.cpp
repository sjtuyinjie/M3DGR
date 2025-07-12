/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <imu_transformer/tf2_sensor_msgs.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void compareCovariances(const boost::array<double, 9>& c1, const boost::array<double, 9>& c2)
{
  for (size_t i = 0; i < 9; ++i)
    EXPECT_NEAR(c1[i], c2[i], 1e-6) << "Wrong value at position " << i;
}

TEST(Covariance, Transform)
{
  boost::array<double, 9> in = {1, 0, 0, 0, 2, 0, 0, 0, 3};
  boost::array<double, 9> expectedOut = {1, 0, 0, 0, 2, 0, 0, 0, 3};
  boost::array<double, 9> out{};
  Eigen::Quaterniond q(1, 0, 0, 0);
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
  
  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1)));
  expectedOut = {2, 0, 0, 0, 1, 0, 0, 0, 3};
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = q.inverse();
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
  
  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 0, 0)));
  expectedOut = {1, 0, 0, 0, 3, 0, 0, 0, 2};
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);

  q = q.inverse();
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
  
  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 1, 0)));
  expectedOut = {3, 0, 0, 0, 2, 0, 0, 0, 1};
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
  
  q = q.inverse();
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
  
  q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 1, 1)));
  expectedOut = {2.5, -0.5, 3, 1, 0, -1, -1.5, 2, 0.5};
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
  
  q = q.inverse();
  expectedOut = {1.5, -1, 1, 2, 2, -1.5, -0.5, 3, -0.5};
  tf2::transformCovariance(in, out, q);
  compareCovariances(expectedOut, out);
}

TEST(Imu, GetTimestamp)
{
  sensor_msgs::Imu msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;
  
  EXPECT_EQ(msg.header.stamp, tf2::getTimestamp(msg));
}

TEST(Imu, GetFrameId)
{
  sensor_msgs::Imu msg;
  msg.header.frame_id = "test";
  
  EXPECT_EQ(msg.header.frame_id, tf2::getFrameId(msg));
}

void prepareImuMsg(sensor_msgs::Imu& msg)
{
  msg.header.frame_id = "test2";
  msg.header.stamp.sec = 1;
  msg.angular_velocity.x = 1;
  msg.angular_velocity.y = 2;
  msg.angular_velocity.z = 3;
  msg.angular_velocity_covariance = {1, 0, 0, 0, 2, 0, 0, 0, 3};
  msg.linear_acceleration.x = 1;
  msg.linear_acceleration.y = 2;
  msg.linear_acceleration.z = 3;
  msg.linear_acceleration_covariance = {1, 0, 0, 0, 2, 0, 0, 0, 3};
  msg.orientation.w = 1;
  msg.orientation_covariance = {1, 0, 0, 0, 2, 0, 0, 0, 3};
}

void prepareTf(geometry_msgs::TransformStamped& tf)
{
  tf.header.frame_id = "test";
  tf.header.stamp.sec = 1;
  tf.child_frame_id = "test2";
  tf.transform.translation.x = 1e6;
  tf.transform.translation.y = 2e6;
  tf.transform.translation.z = -3e6;
  tf.transform.rotation.w = 1;
}

TEST(Imu, DoTransformYaw)
{
  // Q = +90 degrees yaw
  
  sensor_msgs::Imu msg;
  prepareImuMsg(msg);
  
  geometry_msgs::TransformStamped tf;
  prepareTf(tf);

  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);
  
  sensor_msgs::Imu out;
  tf2::doTransform(msg, out, tf);
  
  tf2::Quaternion rot;
  
  EXPECT_EQ("test", out.header.frame_id);
  EXPECT_EQ(msg.header.stamp, out.header.stamp);
  EXPECT_NEAR(-msg.angular_velocity.y, out.angular_velocity.x, 1e-6);
  EXPECT_NEAR(msg.angular_velocity.x, out.angular_velocity.y, 1e-6);
  EXPECT_NEAR(msg.angular_velocity.z, out.angular_velocity.z, 1e-6);
  EXPECT_NEAR(-msg.linear_acceleration.y, out.linear_acceleration.x, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.x, out.linear_acceleration.y, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.z, out.linear_acceleration.z, 1e-6);
  // Transforming orientation means expressing the attitude of the new frame in the same world frame (i.e. you have
  // data in imu frame and want to ask what is the world-referenced orientation of the base_link frame that is attached
  // to this IMU). This is why the orientation change goes the other way than the transform.
  tf2::convert(out.orientation, rot);
  EXPECT_NEAR(0, rot.angleShortestPath(q.inverse()), 1e-6);

  compareCovariances({2, 0, 0, 0, 1, 0, 0, 0, 3}, out.angular_velocity_covariance);
  compareCovariances({2, 0, 0, 0, 1, 0, 0, 0, 3}, out.linear_acceleration_covariance);
  // Orientation covariance stays as it is measured regarding the fixed world frame 
  compareCovariances(msg.orientation_covariance, out.orientation_covariance);
}

TEST(Imu, DoTransformEnuNed)
{
  // Q = ENU->NED transform
  
  sensor_msgs::Imu msg;
  prepareImuMsg(msg);
  
  geometry_msgs::TransformStamped tf;
  prepareTf(tf);

  tf2::Quaternion q;
  q.setRPY(M_PI, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);
  
  sensor_msgs::Imu out;
  tf2::doTransform(msg, out, tf);
  
  tf2::Quaternion rot;
  
  EXPECT_EQ("test", out.header.frame_id);
  EXPECT_EQ(msg.header.stamp, out.header.stamp);
  EXPECT_NEAR(msg.angular_velocity.y, out.angular_velocity.x, 1e-6);
  EXPECT_NEAR(msg.angular_velocity.x, out.angular_velocity.y, 1e-6);
  EXPECT_NEAR(-msg.angular_velocity.z, out.angular_velocity.z, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.y, out.linear_acceleration.x, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.x, out.linear_acceleration.y, 1e-6);
  EXPECT_NEAR(-msg.linear_acceleration.z, out.linear_acceleration.z, 1e-6);
  // Transforming orientation means expressing the attitude of the new frame in the same world frame (i.e. you have
  // data in imu frame and want to ask what is the world-referenced orientation of the base_link frame that is attached
  // to this IMU). This is why the orientation change goes the other way than the transform.
  tf2::convert(out.orientation, rot);
  EXPECT_NEAR(0, rot.angleShortestPath(q.inverse()), 1e-6);

  compareCovariances({2, 0, 0, 0, 1, 0, 0, 0, 3}, out.angular_velocity_covariance);
  compareCovariances({2, 0, 0, 0, 1, 0, 0, 0, 3}, out.linear_acceleration_covariance);
  // Orientation covariance stays as it is measured regarding the fixed world frame 
  compareCovariances(msg.orientation_covariance, out.orientation_covariance);
}

TEST(Mag, GetTimestamp)
{
  sensor_msgs::MagneticField msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nsec = 2;

  EXPECT_EQ(msg.header.stamp, tf2::getTimestamp(msg));
}

TEST(Mag, GetFrameId)
{
  sensor_msgs::MagneticField msg;
  msg.header.frame_id = "test";

  EXPECT_EQ(msg.header.frame_id, tf2::getFrameId(msg));
}

void prepareMagMsg(sensor_msgs::MagneticField& msg)
{
  msg.header.frame_id = "test2";
  msg.header.stamp.sec = 1;
  msg.magnetic_field.x = 1;
  msg.magnetic_field.y = 2;
  msg.magnetic_field.z = 3;
  msg.magnetic_field_covariance = {1, 0, 0, 0, 2, 0, 0, 0, 3};
}

TEST(Mag, DoTransformYaw)
{
  // Q = +90 degrees yaw

  sensor_msgs::MagneticField msg;
  prepareMagMsg(msg);

  geometry_msgs::TransformStamped tf;
  prepareTf(tf);

  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  sensor_msgs::MagneticField out;
  tf2::doTransform(msg, out, tf);

  EXPECT_EQ("test", out.header.frame_id);
  EXPECT_EQ(msg.header.stamp, out.header.stamp);
  EXPECT_NEAR(-msg.magnetic_field.y, out.magnetic_field.x, 1e-6);
  EXPECT_NEAR(msg.magnetic_field.x, out.magnetic_field.y, 1e-6);
  EXPECT_NEAR(msg.magnetic_field.z, out.magnetic_field.z, 1e-6);

  compareCovariances({2, 0, 0, 0, 1, 0, 0, 0, 3}, out.magnetic_field_covariance);
}

TEST(Mag, DoTransformEnuNed)
{
  // Q = ENU->NED transform

  sensor_msgs::MagneticField msg;
  prepareMagMsg(msg);

  geometry_msgs::TransformStamped tf;
  prepareTf(tf);

  tf2::Quaternion q;
  q.setRPY(M_PI, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  sensor_msgs::MagneticField out;
  tf2::doTransform(msg, out, tf);

  EXPECT_EQ("test", out.header.frame_id);
  EXPECT_EQ(msg.header.stamp, out.header.stamp);
  EXPECT_NEAR(msg.magnetic_field.y, out.magnetic_field.x, 1e-6);
  EXPECT_NEAR(msg.magnetic_field.x, out.magnetic_field.y, 1e-6);
  EXPECT_NEAR(-msg.magnetic_field.z, out.magnetic_field.z, 1e-6);

  compareCovariances({2, 0, 0, 0, 1, 0, 0, 0, 3}, out.magnetic_field_covariance);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
