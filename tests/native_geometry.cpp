#include "reserve_odometry/pipeline.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace reserve_odometry;
void check(bool ok,const std::string&what){if(!ok)throw std::runtime_error(what);}
void close(double a,double b,double eps,const std::string&what){check(std::abs(a-b)<eps,what);}
int main(){try{
 Route straight({{{0,0,0}},{{100,0,0}}});
 auto b=straight.pose(25);close(b.position[0],25,1e-9,"line position");close(b.forward[0],1,1e-9,"line heading");
 auto m=antenna(b,-9.873),r=antenna(b,2.563);auto back=from_antennas(m,r);close(norm(sub(back.position,b.position)),0,1e-8,"antenna inverse");
 close(straight.at(110)[0],110,1e-9,"endpoint does not clamp");close(straight.at(-10)[0],-10,1e-9,"negative extrapolation");
 Route grade({{{0,0,0}},{{100,0,10}}});auto gb=grade.pose(30);m=antenna(gb,-9.873);r=antenna(gb,2.563);back=from_antennas(m,r);close(norm(sub(back.position,gb.position)),0,1e-8,"pitch and body-up inverse");
 std::vector<Vec3>points;double radius=30.;for(int i=0;i<=2000;++i){double a=i*.001;points.push_back({{radius*std::cos(a),radius*std::sin(a),0}});}Route circle(points);auto cb=circle.pose(30.);
 double front_angle=30./radius,expected_yaw=front_angle+1.5707963267948966-std::asin(7.55/(2*radius));
 close(std::atan2(cb.forward[1],cb.forward[0]),expected_yaw,2e-5,"circle chord heading");
 auto rear=sub(cb.position,mul(cb.forward,7.55));close(norm(rear),radius,3e-5,"rear pivot lies on circle");
 double factor=std::sqrt(radius*radius-std::pow(7.55/2,2)+std::pow(-9.873+7.55/2,2))/radius;
 close(circle.speed_factor(30,-9.873,3),factor,.0003,"master curved-track speed");
 auto pr=circle.project(cb.position,&cb.forward);close(pr.s,30.,.003,"directed projection");
 points.clear();for(int i=0;i<=4000;++i){double a=i*(6.283185307179586/4000);points.push_back({{radius*std::cos(a),radius*std::sin(a),0}});}points.back()=points.front();Route loop(points,7.55,true);
 close(norm(sub(loop.at(-.2),loop.at(loop.end()-.2))),0,1e-8,"periodic negative lookup");
 auto seam=loop.pose(.1);close(norm(sub(seam.position,mul(seam.forward,7.55))),radius,4e-5,"rear bogie across periodic seam");
 Settings cfg;cfg.compensate_velocity_lever_arm=false;Pipeline pipeline(cfg);Output out;std::int64_t t=10000000000LL;
 for(int i=0;i<200;++i){auto stamp=t+i*50000000LL;pipeline.wheel(0,stamp,18);pipeline.wheel(1,stamp,18);pipeline.command(stamp,0);check(pipeline.step(stamp,stamp,out),"step available");check(std::isfinite(out.velocity),"finite output");}
 close(out.velocity,5.,1e-6,"m/s conversion");close(out.distance,49.75,1e-5,"distance trapezoid");check(out.flags&8,"relative mode flagged");check(!(out.flags&16),"initial window closes with no map");
 check(!pipeline.wheel(0,t+20000000000LL,std::numeric_limits<double>::quiet_NaN()),"NaN rejected");check(!pipeline.command(t,20),"bad command rejected");check(!pipeline.step(t,t,out),"backward stamp rejected");
 pipeline.reset();check(pipeline.step(t,t,out),"reset resumes");
 std::cout<<"PASS 17 geometry, unit, boundary and streaming assertions\n";
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}return 0;}
