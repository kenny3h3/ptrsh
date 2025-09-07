#include "spot_micro_kinematics/utils.h"

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Geometry>

using namespace Eigen;

namespace smk
{ // Start smk namespace

Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang)
{
  // Create 3d transformation, and execute x, y, and z rotations
  Transform<float, 3, Affine> t = Transform<float,3,Affine>::Identity();
  t.rotate(AngleAxisf(x_ang, Vector3f::UnitX()));
  t.rotate(AngleAxisf(y_ang, Vector3f::UnitY()));
  t.rotate(AngleAxisf(z_ang, Vector3f::UnitZ()));

  return t.matrix();
}


Matrix4f homogTransXyz(float x, float y, float z)
{
  // Create a linear translation homogenous transformation matrix
  Transform<float, 3, Affine> t;
  t = Translation<float, 3> (Vector3f(x,y,z));

  return t.matrix();
}


Matrix4f homogInverse(const Matrix4f& ht)
{
//The inverse of a homogeneous transformation matrix can be represented as a
//    a matrix product of the following:
//
//                -------------------   -------------------
//                |           |  0  |   | 1   0   0  -x_t |
//    ht_inv   =  |   R^-1    |  0  | * | 0   1   0  -y_t |
//                |___________|  0  |   | 0   0   1  -z_t |
//                | 0   0   0 |  1  |   | 0   0   0   1   |
//                -------------------   -------------------
//
//    Where R^-1 is the inverse of the rotation matrix portion of the homogeneous
//    transform (the first three rows and columns). Note that the inverse is equal to
//    the transpose for rotation matrices. And -x_t, -y_t, -z_t are the negated linear
//    translation values of the homogenous transform (the first three values of the
//    last column)

  // Initialize return value
  Matrix4f ht_inv = Matrix4f::Identity();

  // Extract rotation matrix
  Matrix3f r = ht.block<3,3>(0,0);

  // Extract translation vector
  Vector3f t = ht.block<3,1>(0,3);

  // Set return value
  ht_inv.block<3,3>(0,0) = r.transpose();
  ht_inv.block<3,1>(0,3) = -r.transpose() * t;

  return ht_inv;
}


Matrix4f htLegRightBack(float body_length, float body_width)
{
  // Create homogeneous transformation matrix representing the transformation
  // from a robot body center to the right back leg coordinate system

  // Translate to half body length in x direction, half body width in negative
  // z direction
  return homogTransXyz(-body_length/2.0f, 0.0f, -body_width/2.0f);
}


Matrix4f htLegRightFront(float body_length, float body_width)
{
  // Create homogeneous transformation matrix representing the transformation
  // from a robot body center to the right front leg coordinate system

  // Translate to half body length in x direction, half body width in negative
  // z direction
  return homogTransXyz(body_length/2.0f, 0.0f, -body_width/2.0f);
}


Matrix4f htLegLeftFront(float body_length, float body_width)
{
  // Create homogeneous transformation matrix representing the transformation
  // from a robot body center to the left front leg coordinate system

  // Translate to half body length in x direction, half body width in positive
  // z direction
  return homogTransXyz(body_length/2.0f, 0.0f, body_width/2.0f);
}


Matrix4f htLegLeftBack(float body_length, float body_width)
{
  // Create homogeneous transformation matrix representing the transformation
  // from a robot body center to the left back leg coordinate system

  // Translate to half body length in x direction, half body width in positive
  // z direction
  return homogTransXyz(-body_length/2.0f, 0.0f, body_width/2.0f);
}


Matrix4f ht0To1(float rot_ang, float link_length)
{
  // Returns the homogeneous transformation matrix representing the
  // transformation from a quadruped leg's coordinate system to the first joint
  // The input angle is in radians

  // The transform consists of a rotation about the y axis, and a translation
  // along the rotated x axis
  return (homogRotXyz(0.0f, rot_ang, 0.0f) *
          homogTransXyz(link_length, 0.0f, 0.0f));
}


Matrix4f ht1To2()
{
  // Returns the homogeneous transformation matrix representing the
  // transformation from a quadruped leg's joint 1 to joint 2
  // This is a constant transformation, a 90 degree rotation about the z axis

  return homogRotXyz(0.0f, 0.0f, M_PI/2.0f);
}


Matrix4f ht2To3(float rot_ang, float link_length)
{
  // Returns the homogeneous transformation matrix representing the
  // transformation from a quadruped leg's joint 2 to joint 3
  // The input angle is in radians

  // The transform consists of a rotation about the y axis, and a translation
  // along the rotated x axis
  return (homogRotXyz(0.0f, rot_ang, 0.0f) *
          homogTransXyz(link_length, 0.0f, 0.0f));
}


Matrix4f ht3To4(float rot_ang, float link_length)
{
  // Returns the homogeneous transformation matrix representing the
  // transformation from a quadruped leg's joint 3 to joint 4
  // The input angle is in radians

  // The transform consists of a rotation about the y axis, and a translation
  // along the rotated x axis
  return (homogRotXyz(0.0f, rot_ang, 0.0f) *
          homogTransXyz(link_length, 0.0f, 0.0f));
}


Matrix4f ht0To4(const JointAngles& joint_angles,
                       const LinkLengths& link_lengths)
{
  // Returns the homogeneous transformation matrix representing the
  // transformation from a quadruped leg's joint 0 to joint 4 (the foot)

  // The transform consists of a rotation about the y axis, and a translation
  // along the rotated x axis
  return (ht0To1(joint_angles.ang1, link_lengths.l1) *
          ht1To2() *
          ht2To3(joint_angles.ang2, link_lengths.l2) *
          ht3To4(joint_angles.ang3,  link_lengths.l3));
}


JointAngles ikine(const Point& point, const LinkLengths& link_lengths, bool is_leg_12) {
  using namespace std;

  // Initialize return struct
  JointAngles joint_angles;

  // Convenience variables for math
  float x4 = point.x;
  float y4 = point.y;
  float z4 = point.z;
  float l1 = link_lengths.l1;
  float l2 = link_lengths.l2;
  float l3 = link_lengths.l3;

  // Supporting variable D
  float D = (x4*x4 + y4*y4 + z4*z4 - l1*l1 - l2*l2 - l3*l3) /
            (2*l2*l3);

  // Poor man's inverse kinematics reachability protection:
  // Limit D to a maximum value of 1, otherwise the square root functions
  // below (sqrt(1 - D^2)) will attempt a square root of a negative number
  if (D > 1.0f) {
    D = 1.0f;
  } else if (D < -1.0f) {
    D = -1.0f;
  }

  if (is_leg_12) {
    joint_angles.ang3 = atan2(sqrt(1 - D*D), D);
  } else {
    joint_angles.ang3 = atan2(-sqrt(1 - D*D), D);
  }

  // Another poor mans reachability sqrt protection
  float protected_sqrt_val = x4*x4 + y4*y4 - l1*l1;
  if (protected_sqrt_val < 0.0f) { protected_sqrt_val = 0.0f;}

  joint_angles.ang2 = atan2(z4, sqrt(protected_sqrt_val)) -
         atan2(l3*sin(joint_angles.ang3), l2 + l3*cos(joint_angles.ang3));

  joint_angles.ang1 = atan2(y4, x4) + atan2(sqrt(protected_sqrt_val), -l1);

  return joint_angles;
}

} // End smk namespace
