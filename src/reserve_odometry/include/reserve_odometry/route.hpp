#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace reserve_odometry {
using Vec3=std::array<double,3>;
inline Vec3 add(Vec3 a,const Vec3& b){for(int i=0;i<3;++i)a[i]+=b[i];return a;}
inline Vec3 sub(Vec3 a,const Vec3& b){for(int i=0;i<3;++i)a[i]-=b[i];return a;}
inline Vec3 mul(Vec3 a,double k){for(auto& v:a)v*=k;return a;}
inline double dot(const Vec3&a,const Vec3&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline double norm(const Vec3&a){return std::sqrt(dot(a,a));}
inline Vec3 unit(Vec3 a){double n=norm(a);return mul(a,1./std::max(n,1e-12));}
inline Vec3 cross(const Vec3&a,const Vec3&b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline bool finite(const Vec3&a){return std::isfinite(a[0])&&std::isfinite(a[1])&&std::isfinite(a[2]);}
struct BodyPose {Vec3 position{},forward{{1,0,0}},left{{0,1,0}},up{{0,0,1}};};
inline BodyPose body_pose(Vec3 p,Vec3 direction){BodyPose b;b.position=p;b.forward=unit(direction);b.left=unit(Vec3{{-b.forward[1],b.forward[0],0}});b.up=cross(b.forward,b.left);return b;}
inline Vec3 antenna(const BodyPose&b,double x,double z=3.){return add(b.position,add(mul(b.forward,x),mul(b.up,z)));}
inline BodyPose from_antennas(const Vec3&m,const Vec3&r,double xm=-9.873,double z=3.){auto b=body_pose(m,sub(r,m));b.position=sub(m,add(mul(b.forward,xm),mul(b.up,z)));return b;}
inline Vec3 ecef(const Vec3& lla){constexpr double rad=0.017453292519943295;double lat=lla[0]*rad,lon=lla[1]*rad,e2=6.6943799901413165e-3,n=6378137./std::sqrt(1-e2*std::sin(lat)*std::sin(lat));return {(n+lla[2])*std::cos(lat)*std::cos(lon),(n+lla[2])*std::cos(lat)*std::sin(lon),(n*(1-e2)+lla[2])*std::sin(lat)};}
inline Vec3 enu(const Vec3&lla,const Vec3&origin={{55.805,37.425,170.}}){constexpr double rad=0.017453292519943295;double la=origin[0]*rad,lo=origin[1]*rad;auto d=sub(ecef(lla),ecef(origin));return {-std::sin(lo)*d[0]+std::cos(lo)*d[1],-std::sin(la)*std::cos(lo)*d[0]-std::sin(la)*std::sin(lo)*d[1]+std::cos(la)*d[2],std::cos(la)*std::cos(lo)*d[0]+std::cos(la)*std::sin(lo)*d[1]+std::sin(la)*d[2]};}

struct Projection {double cost=1e100,s=0.,distance=1e50;};
class Route {
 std::vector<Vec3> p_;std::vector<double> s_;double length_=7.55;bool closed_=false;
 public:
 Route()=default;
 explicit Route(const std::vector<Vec3>&points,double bogie_length=7.55,bool closed=false):length_(bogie_length),closed_(closed){
   if(!std::isfinite(length_)||length_<=0.)throw std::runtime_error("Invalid bogie length");
   for(auto p:points){if(!finite(p))throw std::runtime_error("Nonfinite map point");if(!p_.empty()&&norm(sub(p,p_.back()))<1e-6)continue;s_.push_back(p_.empty()?0.:s_.back()+norm(sub(p,p_.back())));p_.push_back(p);}
   if(p_.size()<2)throw std::runtime_error("Route needs two distinct points");
   if(closed_&&norm(sub(p_.front(),p_.back()))>1e-6)throw std::runtime_error("Closed route must repeat first point");
 }
 static Route load(const std::string&path,double length=7.55,bool closed=false){
   std::ifstream in(path);if(!in)throw std::runtime_error("Cannot open route: "+path);
   std::string line;std::getline(in,line);std::vector<Vec3>pts;double last=-1e100;
   while(std::getline(in,line)){if(line.empty())continue;std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);double s;Vec3 p;if(!(row>>s>>p[0]>>p[1]>>p[2])||!std::isfinite(s)||s<=last)throw std::runtime_error("Invalid route row");last=s;pts.push_back(p);}
   return Route(pts,length,closed);
 }
 double end()const{return s_.back();}
 bool closed()const{return closed_;}
 bool outside(double s)const{return !closed_&&(s<0.||s>end());}
 Vec3 at(double s)const{
   if(!std::isfinite(s))throw std::runtime_error("Nonfinite route distance");
   if(closed_){s=std::fmod(s,end());if(s<0)s+=end();}
   auto it=std::upper_bound(s_.begin(),s_.end(),s);auto i=std::clamp<std::ptrdiff_t>(it-s_.begin()-1,0,p_.size()-2);
   return add(p_[i],mul(sub(p_[i+1],p_[i]),(s-s_[i])/(s_[i+1]-s_[i])));
 }
 BodyPose pose(double s)const{
   auto front=at(s);double lo=s-2*length_,hi=s;
   for(int i=0;i<26;++i){double mid=.5*(lo+hi);if(norm(sub(front,at(mid)))>length_)lo=mid;else hi=mid;}
   return body_pose(front,sub(front,at(.5*(lo+hi))));
 }
 double speed_factor(double s,double x,double z)const{
   auto a=antenna(pose(s-.05),x,z),b=antenna(pose(s+.05),x,z);return std::hypot(b[0]-a[0],b[1]-a[1])/.1;
 }
 Projection project(const Vec3&p,const Vec3*heading=nullptr)const{
   struct C{double cost,s;};std::array<C,8>best;for(auto&c:best)c={1e100,0};
   for(std::size_t i=0;i+1<p_.size();++i){auto v=sub(p_[i+1],p_[i]);double w=std::clamp(dot(sub(p,p_[i]),v)/dot(v,v),0.,1.);auto d=sub(add(p_[i],mul(v,w)),p);double cost=dot(d,d);if(heading){double c=1-dot(unit(v),*heading);cost+=25*c*c;}if(cost<best.back().cost){best.back()={cost,s_[i]+w*(s_[i+1]-s_[i])};std::sort(best.begin(),best.end(),[](const C&a,const C&b){return a.cost<b.cost;});}}
   Projection out;
   for(const auto&c:best){auto b=pose(c.s);auto d=sub(b.position,p);double cost=dot(d,d);if(heading){double v=1-dot(b.forward,*heading);cost+=100*v*v;}if(cost<out.cost)out={cost,c.s,norm(d)};}
   return out;
 }
 // Horizontal matching ignores common GNSS altitude bias; map still supplies Z.
 Projection project_xy(const Vec3&p,const Vec3*heading=nullptr)const{
   struct C{double cost,s;};std::array<C,8>best;for(auto&c:best)c={1e100,0};
   for(std::size_t i=0;i+1<p_.size();++i){auto v=sub(p_[i+1],p_[i]);double den=v[0]*v[0]+v[1]*v[1];if(den<1e-12)continue;auto delta=sub(p,p_[i]);double w=std::clamp((delta[0]*v[0]+delta[1]*v[1])/den,0.,1.);auto d=sub(add(p_[i],mul(v,w)),p);double cost=d[0]*d[0]+d[1]*d[1];if(heading){double c=1-dot(unit(v),*heading);cost+=25*c*c;}if(cost<best.back().cost){best.back()={cost,s_[i]+w*(s_[i+1]-s_[i])};std::sort(best.begin(),best.end(),[](const C&a,const C&b){return a.cost<b.cost;});}}
   Projection out;
   for(const auto&c:best){auto b=pose(c.s);auto d=sub(b.position,p);double cost=d[0]*d[0]+d[1]*d[1];if(heading){double v=1-dot(b.forward,*heading);cost+=100*v*v;}if(cost<out.cost)out={cost,c.s,std::hypot(d[0],d[1])};}
   return out;
 }
 Projection project_antennas(const Vec3&m,const Vec3&r,double mx,double rx,double height,double dm=0.,double dr=0.)const{
   auto body=from_antennas(m,r,mx,height);auto heading=unit(Vec3{{r[0]-m[0],r[1]-m[1],0.}});auto seed=project(body.position,&heading);
   // Mean two-antenna XY squared residual; map supplies pitch, no GNSS-Z pitch.
   // dm/dr propagate the two sensor epochs using only past integrated motion.
   auto cost=[&](double s){auto pm=antenna(pose(s+dm),mx,height),pr=antenna(pose(s+dr),rx,height);double sum=0.;for(int k=0;k<2;++k)sum+=.5*((pm[k]-m[k])*(pm[k]-m[k])+(pr[k]-r[k])*(pr[k]-r[k]));double h=1-dot(pose(s).forward,heading);return sum+100*h*h;};
   double lo=seed.s-15.,hi=seed.s+15.;if(!closed_){lo=std::max(0.,lo);hi=std::min(end(),hi);}
   constexpr double g=.6180339887498949;double a=hi-g*(hi-lo),b=lo+g*(hi-lo),ca=cost(a),cb=cost(b);
   for(int i=0;i<30;++i){if(ca<cb){hi=b;b=a;cb=ca;a=hi-g*(hi-lo);ca=cost(a);}else{lo=a;a=b;ca=cb;b=lo+g*(hi-lo);cb=cost(b);}}
   double s=.5*(lo+hi),c=cost(s);return {c,s,std::sqrt(c)};
 }
};
}
