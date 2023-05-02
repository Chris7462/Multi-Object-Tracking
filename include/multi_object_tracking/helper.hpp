#pragma once
#include <cstdint>
#include <string>

constexpr int maxTruncation {0};
constexpr int maxOcclusion {2};

struct GPSIMU
{
  double lat; // latitude of the oxts-unit (deg)
  double lon; // longitude of the oxts-unit (deg)
  double alt;  // altitude of the oxts-unit (m)
  double roll; // roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  double pitch; // pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  double yaw; // heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  double vn; // velocity towards north (m/s)
  double ve; // velocity towards east (m/s)
  double vf; // forward velocity, i.e. parallel to earth-surface (m/s)
  double vl; // leftward velocity, i.e. parallel to earth-surface (m/s)
  double vu; // upward velocity, i.e. perpendicular to earth-surface (m/s)
  double ax; // acceleration in x, i.e. in direction of vehicle front (m/s^2)
  double ay; // acceleration in y, i.e. in direction of vehicle left (m/s^2)
  double az; // acceleration in z, i.e. in direction of vehicle top (m/s^2)
  double af; // forward acceleration (m/s^2)
  double al; // leftward acceleration (m/s^2)
  double au; // upward acceleration (m/s^2)
  double wx; // angular rate around x (rad/s)
  double wy; // angular rate around y (rad/s)
  double wz; // angular rate around z (rad/s)
  double wf; // angular rate around forward axis (rad/s)
  double wl; // angular rate around leftward axis (rad/s)
  double wu; // angular rate around upward axis (rad/s)
  double posacc; // velocity accuracy (north/east in m)
  double velacc; // velocity accuracy (north/east in m/s)
  int navstat; // navigation status
  int numsats; // number of satellites tracked by primary GPS receiver
  int posmode; // position mode of primary GPS receiver
  int velmode; // velocity mode of primary GPS receiver
  int orimode; // orientation mode of primary GPS receiver
};

struct LABEL
{
  std::string label; //  Describes the type of object
  int truncation; // (0,1,2) indicating the level of truncation.
  int occlusion; // Indicating occlusion state: 0 = fully visible,
                 // 1 = partly occluded, 2 = largely occluded, 3 = unknown
  double alpha; // Observation angle of object, ranging [-pi..pi]

  // bbox --- 2D bounding box of object in the image
  double left; // left[pixel]
  double top; // top[pixel]
  double right; // right[pixel]
  double bottom; // bottom[pixel]

  // dimensions --- 3D object dimensions: height, width, length (in meters)
  double height; // height[meter]
  double width; // width[meter]
  double length; // lenght[meter]

  // location --- 3D object location x,y,z in camera coordinates (in meters)
  double x; // x[meter]
  double y; // y[meter]
  double z; // z[meter]

  double rotation_y; // Rotation ry around Y-axis in camera coordinates [-pi..pi]
                     //needed for p/r curves, higher is better.
};
