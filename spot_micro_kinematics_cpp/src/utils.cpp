#include "spot_micro_kinematics/utils.h"
#include <cmath>

namespace smk {

static inline float d2r(float deg){ return deg * static_cast<float>(M_PI/180.0); }

Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang){
  Eigen::AngleAxisf rx(x_ang, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf ry(y_ang, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rz(z_ang, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f R = rz * ry * rx;
  Matrix4f T = Matrix4f::Identity();
  T.block<3,3>(0,0) = R;
  return T;
}

Matrix4f homogTransXyz(float x,float y,float z){
  Matrix4f T = Matrix4f::Identity();
  T(0,3)=x; T(1,3)=y; T(2,3)=z;
  return T;
}

Matrix4f homogInverse(const Matrix4f& ht){
  Matrix4f inv = Matrix4f::Identity();
  Eigen::Matrix3f R = ht.block<3,3>(0,0);
  Eigen::Vector3f t = ht.block<3,1>(0,3);
  inv.block<3,3>(0,0) = R.transpose();
  inv.block<3,1>(0,3) = -R.transpose()*t;
  return inv;
}

Matrix4f htLegRightBack (float L,float W){ return homogTransXyz(+L/2.f,-W/2.f,0.f); }
Matrix4f htLegRightFront(float L,float W){ return homogTransXyz(-L/2.f,-W/2.f,0.f); }
Matrix4f htLegLeftFront (float L,float W){ return homogTransXyz(-L/2.f,+W/2.f,0.f); }
Matrix4f htLegLeftBack  (float L,float W){ return homogTransXyz(+L/2.f,+W/2.f,0.f); }

Matrix4f ht0To1(float rot_ang,float){ return homogRotXyz(0.f,0.f,rot_ang); }
Matrix4f ht1To2(){ return homogTransXyz(0.f,0.f,0.f); }
Matrix4f ht2To3(float rot_ang,float link){ return homogRotXyz(0.f,rot_ang,0.f)*homogTransXyz(link,0.f,0.f); }
Matrix4f ht3To4(float rot_ang,float link){ return homogRotXyz(0.f,rot_ang,0.f)*homogTransXyz(link,0.f,0.f); }

Matrix4f ht0To4(const JointAngles& q,const LinkLengths& ll,bool){
  return ht0To1(q.q1,0.f)*ht1To2()*ht2To3(q.q2,ll.l2)*ht3To4(q.q3,ll.l3);
}

JointAngles ikine(const Point& p,const LinkLengths& ll,bool){
  JointAngles q{};
  q.q1 = std::atan2(p.y,p.x);
  float R = std::sqrt(p.x*p.x + p.y*p.y);
  float x = R, z = p.z;
  float a = ll.l2, b = ll.l3;
  float c2 = (x*x + z*z - a*a - b*b)/(2.f*a*b);
  c2 = std::max(-1.f,std::min(1.f,c2));
  q.q3 = std::acos(c2);
  float k1 = a + b*std::cos(q.q3);
  float k2 = b*std::sin(q.q3);
  q.q2 = std::atan2(z,x) - std::atan2(k2,k1);
  return q;
}

} // namespace smk
