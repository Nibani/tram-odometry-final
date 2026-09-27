#include "reserve_odometry/output_frame.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace reserve_odometry;
void require(bool c,const char*m){if(!c)throw std::runtime_error(m);}
void near(double x,double y,double e,const char*m){require(std::isfinite(x)&&std::abs(x-y)<e,m);}
int main(){try{
 OutputFrame quarter;quarter.yaw=std::acos(-1.)/2;quarter.translation={{100.,200.,300.}};
 auto p=quarter.position({{2.,3.,4.}});near(p[0],97.,1e-12,"quarter-turn X");near(p[1],202.,1e-12,"quarter-turn Y");near(p[2],304.,1e-12,"translated height");
 auto d=quarter.direction({{2.,3.,4.}});near(d[0],-3.,1e-12,"direction has no translation");near(d[2],4.,1e-12,"direction height unchanged");
 auto b=body_pose({{2.,3.,4.}},{{1.,2.,.3}}),q=quarter.pose(b);
 near(dot(q.forward,q.left),0.,1e-12,"axes perpendicular");near(dot(q.forward,q.up),0.,1e-12,"forward/up perpendicular");near(norm(q.forward),1.,1e-12,"forward unit");near(norm(q.left),1.,1e-12,"left unit");near(norm(q.up),1.,1e-12,"up unit");
 auto world=Vec3{{4.,2.,1.}},rotated=quarter.direction(world);
 near(dot(world,b.forward),dot(rotated,q.forward),1e-12,"body forward velocity invariant");near(dot(world,b.left),dot(rotated,q.left),1e-12,"body lateral velocity invariant");near(dot(world,b.up),dot(rotated,q.up),1e-12,"body vertical velocity invariant");
 OutputFrame actual;actual.yaw=std::atan2(-.02307172830749481,.999733812248493);actual.translation={{101281.45326170324,85499.06949541977,169.99250112111588}};
 auto sample=Vec3{{2341.50,611.98,-3.62}},mapped=actual.position(sample);
 near(mapped[0],.999733812248493*sample[0]+.02307172830749477*sample[1]+actual.translation[0],1e-9,"calibration forward X");
 near(mapped[1],-.02307172830749481*sample[0]+.9997338122484932*sample[1]+actual.translation[1],1e-9,"calibration forward Y");near(mapped[2],166.37250112111588,1e-10,"calibration height offset");
 auto back=OutputFrame{-actual.yaw,{}}.direction(sub(mapped,actual.translation));near(norm(sub(back,sample)),0.,2e-11,"rigid transform inverse");
 std::cout<<"PASS 17 independent output-frame analytic assertions\n";
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}return 0;
}
