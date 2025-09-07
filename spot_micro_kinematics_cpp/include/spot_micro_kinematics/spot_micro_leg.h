#pragma once
#include "spot_micro_kinematics/utils.h"

namespace smk {

class SpotMicroLeg {
public:
  SpotMicroLeg(const JointAngles& joint_angles,
               const LinkLengths& link_lengths,
               bool is_leg_12);
  void setAngles(const JointAngles& joint_angles);
  void setFootPosLocalCoordinates(const Point& point);
  void setFootPosGlobalCoordinates(const Point& point, const Matrix4f& ht_leg_start);

  Point        getFootPosGlobalCoordinates(const Matrix4f& ht_leg_start);
  JointAngles  getLegJointAngles();

  Matrix4f getTransform0To1();
  Matrix4f getTransform1To3();
  Matrix4f getTransform3To4();

private:
  JointAngles  joint_angles_{};
  LinkLengths  link_lengths_{};
  bool         is_leg_12_{true};
};

} // namespace smk
