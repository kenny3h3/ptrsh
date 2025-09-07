#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

using namespace Eigen;

namespace smk { // Start smk namespace

SpotMicroKinematics::SpotMicroKinematics(float x, float y, float z,
                                         const SpotMicroConfig& smc)
    : x_(x),
      y_(y),
      z_(z),
      smc_(smc) {

  // Initialize other class attributes
  phi_ = 0.0f;
  theta_ = 0.0f;
  psi_ = 0.0f;

  // Create temporary structs for initializing leg's joint angles and lengths
  JointAngles joint_angles_temp = {0.0f, 0.0f, 0.0f};
  LinkLengths link_lengths_temp = {smc.hip_link_length,
                                   smc.upper_leg_link_length,
                                   smc.lower_leg_link_length};

  // Create legs
  right_back_leg_  = SpotMicroLeg(joint_angles_temp, link_lengths_temp, true);
  right_front_leg_ = SpotMicroLeg(joint_angles_temp, link_lengths_temp, true);
  left_front_leg_  = SpotMicroLeg(joint_angles_temp, link_lengths_temp, false);
  left_back_leg_   = SpotMicroLeg(joint_angles_temp, link_lengths_temp, false);
}

Matrix4f SpotMicroKinematics::getBodyHt() {
  // Euler angle order is phi, psi, theta because the axes of the robot are x
  // pointing forward, y pointing up, z pointing right
  return(homogTransXyz(x_, y_, z_) * homogRotXyz(phi_, psi_, theta_));
}

void SpotMicroKinematics::setLegJointAngles(
    const LegsJointAngles& four_legs_joint_angs) {
  // Call each leg's method to set joint angles
  right_back_leg_.setAngles(four_legs_joint_angs.right_back);
  right_front_leg_.setAngles(four_legs_joint_angs.right_front);
  left_front_leg_.setAngles(four_legs_joint_angs.left_front);
  left_back_leg_.setAngles(four_legs_joint_angs.left_back);
}

void SpotMicroKinematics::setFeetPosGlobalCoordinates(const LegsFootPos& four_legs_foot_pos) {
  // Get homogeneous transforms for leg starts
  Matrix4f ht_leg_right_back = getBodyHt() * htLegRightBack(smc_.body_length, smc_.body_width);
  Matrix4f ht_leg_right_front = getBodyHt() * htLegRightFront(smc_.body_length, smc_.body_width);
  Matrix4f ht_leg_left_front = getBodyHt() * htLegLeftFront(smc_.body_length, smc_.body_width);
  Matrix4f ht_leg_left_back = getBodyHt() * htLegLeftBack(smc_.body_length, smc_.body_width);

  // Call set foot position method for each leg
  right_back_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.right_back, ht_leg_right_back);
  right_front_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.right_front, ht_leg_right_front);
  left_front_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.left_front, ht_leg_left_front);
  left_back_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.left_back, ht_leg_left_back);
}

void SpotMicroKinematics::setBodyAngles(float phi, float theta, float psi) {
  phi_ = phi;
  theta_ = theta;
  psi_ = psi;
}

void SpotMicroKinematics::setBodyPosition(float x, float y, float z) {
  x_ = x;
  y_ = y;
  z_ = z;
}

void SpotMicroKinematics::setBodyState(const BodyState& body_state) {
  // Set body angles and position
  setBodyAngles(body_state.euler_angs.phi, body_state.euler_angs.theta, body_state.euler_angs.psi);
  setBodyPosition(body_state.xyz_pos.x, body_state.xyz_pos.y, body_state.xyz_pos.z);

  // Set feet position
  setFeetPosGlobalCoordinates(body_state.leg_feet_pos);
}

LegsJointAngles SpotMicroKinematics::getLegsJointAngles() {
  LegsJointAngles legs_joint_angles;
  legs_joint_angles.right_back = right_back_leg_.getLegJointAngles();
  legs_joint_angles.right_front = right_front_leg_.getLegJointAngles();
  legs_joint_angles.left_front = left_front_leg_.getLegJointAngles();
  legs_joint_angles.left_back = left_back_leg_.getLegJointAngles();
  return legs_joint_angles;
}

LegsFootPos SpotMicroKinematics::getLegsFootPos() {
  LegsFootPos legs_foot_pos;
  Matrix4f ht_leg_right_back = getBodyHt() * htLegRightBack(smc_.body_length, smc_.body_width);
  Matrix4f ht_leg_right_front = getBodyHt() * htLegRightFront(smc_.body_length, smc_.body_width);
  Matrix4f ht_leg_left_front = getBodyHt() * htLegLeftFront(smc_.body_length, smc_.body_width);
  Matrix4f ht_leg_left_back = getBodyHt() * htLegLeftBack(smc_.body_length, smc_.body_width);

  legs_foot_pos.right_back = right_back_leg_.getFootPosGlobalCoordinates(ht_leg_right_back);
  legs_foot_pos.right_front = right_front_leg_.getFootPosGlobalCoordinates(ht_leg_right_front);
  legs_foot_pos.left_front = left_front_leg_.getFootPosGlobalCoordinates(ht_leg_left_front);
  legs_foot_pos.left_back = left_back_leg_.getFootPosGlobalCoordinates(ht_leg_left_back);
  return legs_foot_pos;
}

BodyState SpotMicroKinematics::getBodyState() {
  BodyState body_state;
  body_state.euler_angs.phi = phi_;
  body_state.euler_angs.theta = theta_;
  body_state.euler_angs.psi = psi_;
  body_state.xyz_pos.x = x_;
  body_state.xyz_pos.y = y_;
  body_state.xyz_pos.z = z_;
  body_state.leg_feet_pos = getLegsFootPos();
  return body_state;
}

AllRobotRelativeTransforms SpotMicroKinematics::getRobotTransforms() {

  // Initialize structure
  AllRobotRelativeTransforms allTransforms;

  // Fill the structure in
  // Body center
  allTransforms.bodyCenter = getBodyHt();

  // Center to four leg corners
  allTransforms.centerToRightBack = htLegRightBack(smc_.body_length,
                                                   smc_.body_width);

  allTransforms.centerToRightFront = htLegRightFront(smc_.body_length,
                                                     smc_.body_width);

  allTransforms.centerToLeftFront = htLegLeftFront(smc_.body_length,
                                                   smc_.body_width);

  allTransforms.centerToLeftBack = htLegLeftBack(smc_.body_length,
                                                 smc_.body_width);

  // Right back leg
  allTransforms.rightBackLeg.t01 = right_back_leg_.getTransform0To1();
  allTransforms.rightBackLeg.t13 = right_back_leg_.getTransform1To3();
  allTransforms.rightBackLeg.t34 = right_back_leg_.getTransform3To4();

  // Right front leg
  allTransforms.rightFrontLeg.t01 = right_front_leg_.getTransform0To1();
  allTransforms.rightFrontLeg.t13 = right_front_leg_.getTransform1To3();
  allTransforms.rightFrontLeg.t34 = right_front_leg_.getTransform3To4();

  // Left front leg
  allTransforms.leftFrontLeg.t01 = left_front_leg_.getTransform0To1();
  allTransforms.leftFrontLeg.t13 = left_front_leg_.getTransform1To3();
  allTransforms.leftFrontLeg.t34 = left_front_leg_.getTransform3To4();

  // Left back leg
  allTransforms.leftBackLeg.t01 = left_back_leg_.getTransform0To1();
  allTransforms.leftBackLeg.t13 = left_back_leg_.getTransform1To3();
  allTransforms.leftBackLeg.t34 = left_back_leg_.getTransform3To4();

  return allTransforms;
}

} // End smk namespace
