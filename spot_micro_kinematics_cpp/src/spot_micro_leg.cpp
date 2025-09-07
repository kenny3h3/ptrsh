#include "spot_micro_kinematics/spot_micro_leg.h"

namespace smk {

SpotMicroLeg::SpotMicroLeg(const JointAngles& ja,const LinkLengths& ll,bool is_leg_12)
: joint_angles_(ja), link_lengths_(ll), is_leg_12_(is_leg_12) {}

void SpotMicroLeg::setAngles(const JointAngles& ja){ joint_angles_ = ja; }

void SpotMicroLeg::setFootPosLocalCoordinates(const Point& p){
  joint_angles_ = ikine(p, link_lengths_, is_leg_12_);
}

void SpotMicroLeg::setFootPosGlobalCoordinates(const Point& p, const Matrix4f& ht_leg_start){
  Matrix4f inv = homogInverse(ht_leg_start);
  Eigen::Vector4f pg(p.x,p.y,p.z,1.f);
  Eigen::Vector4f pl = inv*pg;
  setFootPosLocalCoordinates(Point{pl.x(),pl.y(),pl.z()});
}

Point SpotMicroLeg::getFootPosGlobalCoordinates(const Matrix4f& ht_leg_start){
  Matrix4f ht = ht_leg_start * ht0To4(joint_angles_, link_lengths_, is_leg_12_);
  return { ht(0,3), ht(1,3), ht(2,3) };
}

JointAngles SpotMicroLeg::getLegJointAngles(){ return joint_angles_; }

Matrix4f SpotMicroLeg::getTransform0To1(){ return ht0To1(joint_angles_.q1,0.f); }
Matrix4f SpotMicroLeg::getTransform1To3(){ return ht0To1(joint_angles_.q1,0.f)*ht1To2()*ht2To3(joint_angles_.q2,link_lengths_.l2); }
Matrix4f SpotMicroLeg::getTransform3To4(){ return ht3To4(joint_angles_.q3,link_lengths_.l3); }

} // namespace smk
