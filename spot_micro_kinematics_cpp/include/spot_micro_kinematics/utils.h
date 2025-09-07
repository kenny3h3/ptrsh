#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace smk {

using Matrix4f = Eigen::Matrix4f;
using Vector3f = Eigen::Vector3f;

struct Point { float x{0.f}, y{0.f}, z{0.f}; };
struct JointAngles { float q1{0.f}, q2{0.f}, q3{0.f}; };
struct LinkLengths { float l1{0.f}, l2{0.f}, l3{0.f}; };
struct EulerAngles { float phi{0.f}, theta{0.f}, psi{0.f}; };

struct LegsJointAngles { JointAngles right_back, right_front, left_front, left_back; };
struct LegsFootPos     { Point right_back, right_front, left_front, left_back; };

struct BodyState {
  EulerAngles euler_angs;
  Point       xyz_pos;
  LegsFootPos leg_feet_pos;
};

Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang);
Matrix4f homogTransXyz(float x, float y, float z);
Matrix4f homogInverse(const Matrix4f& ht);

Matrix4f htLegRightBack (float body_length, float body_width);
Matrix4f htLegRightFront(float body_length, float body_width);
Matrix4f htLegLeftFront (float body_length, float body_width);
Matrix4f htLegLeftBack  (float body_length, float body_width);

Matrix4f ht0To1(float rot_ang, float link_length);
Matrix4f ht1To2();
Matrix4f ht2To3(float rot_ang, float link_length);
Matrix4f ht3To4(float rot_ang, float link_length);

Matrix4f ht0To4(const JointAngles& joint_angles,
                const LinkLengths& link_lengths,
                bool is_leg_12);

JointAngles ikine(const Point& point,
                  const LinkLengths& link_lengths,
                  bool is_leg_12);

} // namespace smk
