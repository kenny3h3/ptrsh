#pragma once
#include "spot_micro_kinematics/spot_micro_leg.h"

namespace smk {

struct LegRelativeTransforms { Matrix4f right_back, right_front, left_front, left_back; };
struct AllRobotRelativeTransforms { LegRelativeTransforms legs; Matrix4f body; };

class SpotMicroKinematics {
public:
  SpotMicroKinematics(float x,float y,float z,
                      float phi,float theta,float psi,
                      float body_length,float body_width,
                      const LinkLengths& ll);

  Matrix4f getBodyHt();

  void setLegJointAngles(const LegsJointAngles& a);
  void setFeetPosGlobalCoordinates(const LegsFootPos& p);

  void setBodyAngles(float phi,float theta,float psi);
  void setBodyPosition(float x,float y,float z);
  void setBodyState(const BodyState& s);

  LegsJointAngles getLegsJointAngles();
  LegsFootPos     getLegsFootPos();
  BodyState       getBodyState();

  AllRobotRelativeTransforms getRobotTransforms();

private:
  float x_{0.f}, y_{0.f}, z_{0.f};
  float phi_{0.f}, theta_{0.f}, psi_{0.f};
  float body_length_{0.f}, body_width_{0.f};

  LinkLengths ll_{};
  SpotMicroLeg right_back_leg_ {JointAngles{}, ll_, true};
  SpotMicroLeg right_front_leg_{JointAngles{}, ll_, true};
  SpotMicroLeg left_front_leg_ {JointAngles{}, ll_, false};
  SpotMicroLeg left_back_leg_  {JointAngles{}, ll_, false};
};

} // namespace smk
