#include "spot_micro_kinematics/spot_micro_kinematics.h"

namespace smk {

SpotMicroKinematics::SpotMicroKinematics(float x,float y,float z,
                                         float phi,float theta,float psi,
                                         float L,float W,
                                         const LinkLengths& ll)
: x_(x), y_(y), z_(z), phi_(phi), theta_(theta), psi_(psi),
  body_length_(L), body_width_(W), ll_(ll),
  right_back_leg_ (JointAngles{}, ll_, true),
  right_front_leg_(JointAngles{}, ll_, true),
  left_front_leg_ (JointAngles{}, ll_, false),
  left_back_leg_  (JointAngles{}, ll_, false) {}

Matrix4f SpotMicroKinematics::getBodyHt(){
  return homogTransXyz(x_,y_,z_) * homogRotXyz(phi_,theta_,psi_);
}

void SpotMicroKinematics::setLegJointAngles(const LegsJointAngles& a){
  right_back_leg_.setAngles (a.right_back);
  right_front_leg_.setAngles(a.right_front);
  left_front_leg_.setAngles (a.left_front);
  left_back_leg_.setAngles  (a.left_back);
}

void SpotMicroKinematics::setFeetPosGlobalCoordinates(const LegsFootPos& p){
  Matrix4f ht_rb = getBodyHt() * htLegRightBack (body_length_, body_width_);
  Matrix4f ht_rf = getBodyHt() * htLegRightFront(body_length_, body_width_);
  Matrix4f ht_lf = getBodyHt() * htLegLeftFront (body_length_, body_width_);
  Matrix4f ht_lb = getBodyHt() * htLegLeftBack  (body_length_, body_width_);

  right_back_leg_.setFootPosGlobalCoordinates (p.right_back , ht_rb);
  right_front_leg_.setFootPosGlobalCoordinates(p.right_front, ht_rf);
  left_front_leg_.setFootPosGlobalCoordinates (p.left_front , ht_lf);
  left_back_leg_.setFootPosGlobalCoordinates  (p.left_back  , ht_lb);
}

void SpotMicroKinematics::setBodyAngles(float phi,float theta,float psi){ phi_=phi; theta_=theta; psi_=psi; }
void SpotMicroKinematics::setBodyPosition(float x,float y,float z){ x_=x; y_=y; z_=z; }

void SpotMicroKinematics::setBodyState(const BodyState& s){
  setBodyAngles(s.euler_angs.phi, s.euler_angs.theta, s.euler_angs.psi);
  setBodyPosition(s.xyz_pos.x, s.xyz_pos.y, s.xyz_pos.z);
  setFeetPosGlobalCoordinates(s.leg_feet_pos);
}

LegsJointAngles SpotMicroKinematics::getLegsJointAngles(){
  LegsJointAngles a;
  a.right_back  = right_back_leg_.getLegJointAngles();
  a.right_front = right_front_leg_.getLegJointAngles();
  a.left_front  = left_front_leg_.getLegJointAngles();
  a.left_back   = left_back_leg_.getLegJointAngles();
  return a;
}

LegsFootPos SpotMicroKinematics::getLegsFootPos(){
  Matrix4f ht_rb = getBodyHt() * htLegRightBack (body_length_, body_width_);
  Matrix4f ht_rf = getBodyHt() * htLegRightFront(body_length_, body_width_);
  Matrix4f ht_lf = getBodyHt() * htLegLeftFront (body_length_, body_width_);
  Matrix4f ht_lb = getBodyHt() * htLegLeftBack  (body_length_, body_width_);
  LegsFootPos p;
  p.right_back  = right_back_leg_.getFootPosGlobalCoordinates (ht_rb);
  p.right_front = right_front_leg_.getFootPosGlobalCoordinates(ht_rf);
  p.left_front  = left_front_leg_.getFootPosGlobalCoordinates (ht_lf);
  p.left_back   = left_back_leg_.getFootPosGlobalCoordinates  (ht_lb);
  return p;
}

BodyState SpotMicroKinematics::getBodyState(){
  BodyState s;
  s.euler_angs = {phi_,theta_,psi_};
  s.xyz_pos    = {x_,y_,z_};
  s.leg_feet_pos = getLegsFootPos();
  return s;
}

AllRobotRelativeTransforms SpotMicroKinematics::getRobotTransforms(){
  AllRobotRelativeTransforms r;
  r.body = getBodyHt();
  r.legs.right_back  = htLegRightBack (body_length_, body_width_);
  r.legs.right_front = htLegRightFront(body_length_, body_width_);
  r.legs.left_front  = htLegLeftFront (body_length_, body_width_);
  r.legs.left_back   = htLegLeftBack  (body_length_, body_width_);
  return r;
}

} // namespace smk
