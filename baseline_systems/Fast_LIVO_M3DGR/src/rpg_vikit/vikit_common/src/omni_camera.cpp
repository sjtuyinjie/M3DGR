/*
 * OcamProjector.cpp
 *
 *  Created on: Sep 22, 2010
 *      Author: laurent kneip
 */

#include <stdio.h>
#include <math.h>
#include <vikit/omni_camera.h>

namespace vk {

OmniCamera::
OmniCamera( string calibFile )
{
  double *pol        = ocamModel.pol;
  double *invpol     = ocamModel.invpol;
  double *xc         = &ocamModel.xc;
  double *yc         = &ocamModel.yc;
  double *c          = &ocamModel.c;
  double *d          = &ocamModel.d;
  double *e          = &ocamModel.e;
  int *width         = &ocamModel.width;
  int *height        = &ocamModel.height;
  int *length_pol    = &ocamModel.length_pol;
  int *length_invpol = &ocamModel.length_invpol;
  FILE *f;
  char buf[CMV_MAX_BUF];
  int i;

  printf("Initialize OmniCamera: Read Calibration %s\n", calibFile.c_str());

  //Open file
  if( !( f = fopen( (char*) calibFile.c_str(), "r" ) ) )
  {
    printf("Initialize OmniCamera: Cannot read calibration file.");
    return;
  }

  //Read polynomial coefficients
  char* dummy = fgets( buf, CMV_MAX_BUF, f );
  int result = fscanf( f, "\n" );
  result = fscanf( f, "%d", length_pol );
  for( i = 0; i < *length_pol; i++ )
          result = fscanf( f, " %lf", &pol[i] );

  //Read inverse polynomial coefficients
  result = fscanf( f, "\n" );
  dummy = fgets( buf, CMV_MAX_BUF, f );
  result = fscanf( f, "\n" );
  result = fscanf( f, "%d", length_invpol );
  for( i = 0; i < *length_invpol; i++ )
          result = fscanf( f, " %lf", &invpol[i] );

  //Read center coordinates
  result = fscanf( f, "\n" );
  dummy = fgets( buf, CMV_MAX_BUF, f );
  result = fscanf( f, "\n" );
  result = fscanf( f, "%lf %lf\n", xc, yc );

  //Read affine coefficients
  dummy = fgets( buf, CMV_MAX_BUF, f );
  result = fscanf( f, "\n" );
  result = fscanf( f, "%lf %lf %lf\n", c, d, e );

  //Read image size
  dummy = fgets( buf, CMV_MAX_BUF, f );
  result = fscanf( f, "\n" );
  result = fscanf( f, "%d %d", height, width );

  fclose(f);

  width_ = *width;
  height_ = *height;
  error_multiplier_ = computeErrorMultiplier();
}

OmniCamera::
~OmniCamera()
{}

Vector3d OmniCamera::
cam2world(const double& u, const double& v) const
{
  Vector3d xyz;

  // Important: we exchange x and y since regular pinhole model is working with x along the columns and y along the rows
  // Davide's framework is doing exactly the opposite

  double invdet  = 1 / ( ocamModel.c - ocamModel.d * ocamModel.e );

  xyz[0] = invdet * ( ( v - ocamModel.xc ) - ocamModel.d * ( u - ocamModel.yc ) );
  xyz[1] = invdet * ( -ocamModel.e * ( v - ocamModel.xc ) + ocamModel.c * ( u - ocamModel.yc ) );

  double r = sqrt(  pow( xyz[0], 2 ) + pow( xyz[1], 2 ) ); //distance [pixels] of  the point from the image center
  xyz[2] = ocamModel.pol[0];
  double r_i = 1;

  for( int i = 1; i < ocamModel.length_pol; i++ )
  {
    r_i *= r;
    xyz[2] += r_i * ocamModel.pol[i];
  }

  xyz.normalize();

  // change back to pinhole model:
  double temp = xyz[0];
  xyz[0] = xyz[1];
  xyz[1] = temp;
  xyz[2] = -xyz[2];

  return xyz;
}

Vector3d OmniCamera::
cam2world (const Vector2d& px) const
{
  return cam2world(px[0], px[1]);
}

Vector2d OmniCamera::
world2cam(const Vector3d& xyz_c) const
{
  Vector2d uv;

  // transform world-coordinates to Davide's camera frame
  Vector3d worldCoordinates_bis;
  worldCoordinates_bis[0] = xyz_c[1];
  worldCoordinates_bis[1] = xyz_c[0];
  worldCoordinates_bis[2] = -xyz_c[2];

  double norm = sqrt( pow( worldCoordinates_bis[0], 2 ) + pow( worldCoordinates_bis[1], 2 ) );
  double theta = atan( worldCoordinates_bis[2]/norm );

  // Important: we exchange x and y since Pirmin's stuff is working with x along the columns and y along the rows,
  // Davide's framework is doing exactly the opposite
  double rho;
  double t_i;
  double x;
  double y;

  if(norm != 0)
  {
    rho = ocamModel.invpol[0];

    t_i = 1;

    for( int i = 1; i < ocamModel.length_invpol; i++ )
    {
      t_i *= theta;
      rho += t_i * ocamModel.invpol[i];
    }

    x = worldCoordinates_bis[0] * rho/norm;
    y = worldCoordinates_bis[1] * rho/norm;

    // we exchange 0 and 1 in order to have pinhole model again
    uv[1] = x * ocamModel.c + y * ocamModel.d + ocamModel.xc;
    uv[0] = x * ocamModel.e + y + ocamModel.yc;
  }
  else
  {
    // we exchange 0 and 1 in order to have pinhole model again
    uv[1] = ocamModel.xc;
    uv[0] = ocamModel.yc;
  }

  return uv;
}

Vector2d OmniCamera::
world2cam(const Vector2d& uv) const
{
  return world2cam(unproject2d(uv).normalized());
}

double OmniCamera::
computeErrorMultiplier()
{
  Vector3d vector1 = cam2world( .5*width_, .5*height_ );
  Vector3d vector2 = cam2world( .5*width_ + .5, .5*height_ );
  vector1 = vector1/vector1.norm();
  vector2 = vector2/vector2.norm();

  double factor1 = .5/( 1 - vector1.dot(vector2) );

  vector1 = cam2world( width_, .5*height_ );
  vector2 = cam2world( -.5 + (double) width_ , .5*height_ );
  vector1 = vector1/vector1.norm();
  vector2 = vector2/vector2.norm();

  double factor2 = .5/( 1 - vector1.dot(vector2) );

  return ( factor2 + factor1 ) * .5;
}

} // end namespace vk
