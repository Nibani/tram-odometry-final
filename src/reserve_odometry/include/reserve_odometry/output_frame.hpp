#pragma once
#include "reserve_odometry/route.hpp"

namespace reserve_odometry {
// Rigid ENU-to-Pathgraph transform. Translation applies only to position;
// body-frame velocities are invariant under this change of world frame.
struct OutputFrame {
 double yaw=0.;Vec3 translation{};
 Vec3 direction(const Vec3&v)const{
   double c=std::cos(yaw),s=std::sin(yaw);
   return {{c*v[0]-s*v[1],s*v[0]+c*v[1],v[2]}};
 }
 Vec3 position(const Vec3&p)const{return add(direction(p),translation);}
 BodyPose pose(BodyPose b)const{
   b.position=position(b.position);b.forward=direction(b.forward);
   b.left=direction(b.left);b.up=direction(b.up);return b;
 }
};
}
