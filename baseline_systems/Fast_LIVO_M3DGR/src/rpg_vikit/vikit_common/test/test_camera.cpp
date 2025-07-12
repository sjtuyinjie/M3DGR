/*
 * camera_pinhole_test.cpp
 *
 *  Created on: Oct 26, 2012
 *      Author: cforster
 */

#include <string>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>
#include <vikit/sample.h>

using namespace std;
using namespace Eigen;


void testTiming(vk::AbstractCamera* cam)
{
  Vector3d xyz;
  Vector2d px(320.64, 253.54);
  vk::Timer t;
  t.start();
  for(size_t i=0; i<1000; ++i)
  {
    xyz = cam->cam2world(px);
  }
  t.stop();
  cout << "Time unproject = " << t.getTime()*1000 << "ms" << endl;

  t.start();
  for(size_t i=0; i<1000; ++i)
  {
    px = cam->world2cam(xyz);
  }
  t.stop();
  cout << "Time project = " << t.getTime()*1000 << "ms" << endl;
}

void testAccuracy(vk::AbstractCamera* cam)
{
  double error = 0.0;
  vk::Timer t;
  for(size_t i=0; i<1000; ++i)
  {
    Vector2d px(1.0/100.0 * vk::Sample::uniform(0, cam->width()*100), 
                1.0/100.0 * vk::Sample::uniform(0, cam->height()*100));
    Vector3d xyz = cam->cam2world(px);
    Vector2d px2 = cam->world2cam(xyz);
    error += (px-px2).norm();
  }
  cout << "Reprojection error = " << error << " (took " << t.stop()*1000 << "ms)" << endl;

}

int main(int argc, char **argv)
{
  vk::AbstractCamera* cam_pinhole = 
    new vk::PinholeCamera(640, 480, 
                          323.725240539365, 323.53310403533,
                          336.407165453746, 235.018271952295,
                          -0.258617082313663, 0.0623042373522829, 0.000445967802619555, -0.000269839440982019);

  vk::AbstractCamera* cam_atan = 
    new vk::ATANCamera(752, 480, 0.511496, 0.802603, 0.530199, 0.496011, 0.934092);

  printf("\nPINHOLE CAMERA:\n");
  testTiming(cam_pinhole);
  testAccuracy(cam_pinhole);

  printf("\nATAN CAMERA:\n");
  testTiming(cam_atan);
  testAccuracy(cam_atan);

  return 0;
}