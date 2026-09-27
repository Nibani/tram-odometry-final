#include "reserve_odometry/pipeline.hpp"
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reserve_odometry;
using I = std::int64_t;
constexpr I T0=10000000000LL, DT=50000000LL;
constexpr double PI=3.14159265358979323846;
int failures=0;

void require(bool value,const std::string&label){if(!value)throw std::runtime_error(label);}
void near(double actual,double expected,double tolerance,const std::string&label){
 if(!std::isfinite(actual)||std::abs(actual-expected)>tolerance)
  throw std::runtime_error(label+" actual="+std::to_string(actual)+" expected="+std::to_string(expected));
}
void test(const std::string&name,const std::function<void()>&body){
 try{body();std::cout<<"PASS "<<name<<'\n';}
 catch(const std::exception&e){++failures;std::cout<<"FAIL "<<name<<": "<<e.what()<<'\n';}
}

// Independent inverse WGS84 transform to synthesize physical GNSS observations.
Vec3 observation_lla(const Vec3&p){
 const double lat0=55.805*PI/180.,lon0=37.425*PI/180.,e2=6.6943799901413165e-3;
 const auto origin=ecef({{55.805,37.425,170.}});
 const double x=origin[0]-std::sin(lon0)*p[0]-std::sin(lat0)*std::cos(lon0)*p[1]+std::cos(lat0)*std::cos(lon0)*p[2];
 const double y=origin[1]+std::cos(lon0)*p[0]-std::sin(lat0)*std::sin(lon0)*p[1]+std::cos(lat0)*std::sin(lon0)*p[2];
 const double z=origin[2]+std::cos(lat0)*p[1]+std::sin(lat0)*p[2];
 const double rho=std::hypot(x,y);double lat=std::atan2(z,rho*(1-e2)),height=0.;
 for(int i=0;i<20;++i){double n=6378137./std::sqrt(1-e2*std::sin(lat)*std::sin(lat));height=rho/std::cos(lat)-n;lat=std::atan2(z,rho*(1-e2*n/(n+height)));}
 return {{lat*180/PI,std::atan2(y,x)*180/PI,height}};
}
Settings settings(bool enabled){Settings c;c.sparse_gnss_correction=enabled;return c;}
Route line(){return Route({{{0,0,0}},{{1000,0,0}}});}
Route circle(){
 std::vector<Vec3>p;for(int i=0;i<=4000;++i){double a=i*2*PI/4000;p.push_back({{30*std::cos(a),30*std::sin(a),0}});}p.back()=p.front();return Route(p,7.55,true);
}
struct Fixture {
 Route route;Pipeline pipe;double start=50.;Output last;int index=-1;
 explicit Fixture(bool enabled=true,Route map=line(),double phase=50.):route(map),pipe(settings(enabled),{route}),start(phase){}
 Vec3 fix_lla(int channel,I stamp,double shift=0.,double cross=0.)const{
  auto p=antenna(route.pose(start+5.*double(stamp-T0)*1e-9+shift),channel==0?-9.873:2.563,3.);p[1]+=cross;return observation_lla(p);
 }
 Output tick(int i,bool initial=true){
  I t=T0+i*DT;require(pipe.wheel(0,t,18.,t),"front accepted");require(pipe.wheel(1,t,18.,t),"rear accepted");
  if(initial&&i<=56&&i%2==0)for(int channel=0;channel<2;++channel)require(pipe.fix(channel,t,t,fix_lla(channel,t)),"startup fix accepted");
  require(pipe.command(t,0,t),"command accepted");require(pipe.step(t,t,last),"output available");index=i;return last;
 }
 void through(int end){for(int i=index+1;i<=end;++i)tick(i);}
};
void speed_unchanged(const Output&a,const Output&b){
 require(a.bogie_velocity==b.bogie_velocity,"core velocity changed");require(a.acceleration==b.acceleration,"core acceleration changed");require(a.distance==b.distance,"integrated wheel distance changed");
}

int main(){std::cout<<std::setprecision(17);
 test("legacy_and_no_fix_invariance",[]{Fixture a(false),b(true);for(int i=0;i<=120;++i){auto x=a.tick(i),y=b.tick(i);speed_unchanged(x,y);require(x.route_s==y.route_s&&x.position==y.position&&x.body_velocity==y.body_velocity&&x.flags==y.flags,"disabled/no-fix trajectory differs");}require(a.last.initialized&&b.last.initialized,"startup failed");});
 for(int channel=0;channel<2;++channel)test(channel?"single_rover":"single_master",[channel]{Fixture a(false),b(true);a.through(79);b.through(79);I t=T0+80*DT;require(b.pipe.fix(channel,t,t,b.fix_lla(channel,t,8.)),"late single fix rejected");auto x=a.tick(80),y=b.tick(80);speed_unchanged(x,y);near(y.route_s-x.route_s,2.,1e-4,"single correction");require(y.gnss_accepted==1&&y.gnss_rejected==0,"single count");});
 test("delayed_fix_uses_measurement_epoch",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+74*DT,arrival=T0+80*DT;require(b.pipe.fix(0,stamp,arrival,b.fix_lla(0,stamp,8.)),"delayed fix rejected");auto x=a.tick(80),y=b.tick(80);near(y.route_s-x.route_s,2.,1e-4,"arrival substituted for measurement time");speed_unchanged(x,y);});
 test("future_fix_queued_until_epoch",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+81*DT,arrival=T0+80*DT;require(b.pipe.fix(0,stamp,arrival,b.fix_lla(0,stamp,8.)),"50ms future should queue");auto x=a.tick(80),y=b.tick(80);require(x.position==y.position&&y.gnss_accepted==0,"future fix used early");x=a.tick(81);y=b.tick(81);near(y.route_s-x.route_s,2.,1e-4,"future fix not processed at epoch");require(y.gnss_accepted==1,"future fix count");});
 test("future_over_tolerance_does_not_poison_order",[]{Fixture a(false),b(true);a.through(79);b.through(79);I now=T0+80*DT;require(!b.pipe.fix(0,now+50000001,now,b.fix_lla(0,now+50000001,8.)),"future >50ms accepted");require(b.pipe.fix(0,now,now,b.fix_lla(0,now,8.)),"future poisoned channel watermark");near(b.tick(80).route_s-a.tick(80).route_s,2.,1e-4,"valid following fix failed");});
 test("stale_at_arrival_rejected",[]{Fixture a(false),b(true);a.through(99);b.through(99);I now=T0+100*DT,stamp=now-1000000001;require(!b.pipe.fix(0,stamp,now,b.fix_lla(0,stamp,8.)),"stale arrival accepted");auto x=a.tick(100),y=b.tick(100);require(x.position==y.position&&y.gnss_accepted==0&&y.gnss_rejected==1,"stale arrival changed phase");});
 test("stale_at_processing_rejected",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+81*DT;require(b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp,8.)),"fresh queued fix rejected");auto x=a.tick(102,false),y=b.tick(102,false);require(x.position==y.position&&y.gnss_accepted==0&&y.gnss_rejected==1,"queued stale fix changed phase");});
 test("duplicate_after_consumption_rejected",[]{Fixture b(true);b.through(79);I stamp=T0+80*DT;auto lla=b.fix_lla(0,stamp,8.);require(b.pipe.fix(0,stamp,stamp,lla),"first fix rejected");auto y=b.tick(80);require(y.gnss_accepted==1,"first fix unused");require(!b.pipe.fix(0,stamp,stamp+DT,lla),"consumed duplicate accepted");require(!b.pipe.fix(0,stamp-DT,stamp+DT,lla),"older fix accepted");require(b.tick(81).gnss_accepted==1,"duplicate counted twice");});
 test("per_receiver_duplicate_watermark",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+80*DT;for(int channel=0;channel<2;++channel)require(b.pipe.fix(channel,stamp,stamp,b.fix_lla(channel,stamp,8.)),"other receiver same stamp rejected");auto x=a.tick(80),y=b.tick(80);near(y.route_s-x.route_s,3.5,1e-4,"two receiver correction sequence");require(y.gnss_accepted==2,"dual count");speed_unchanged(x,y);});
 test("innovation_gate",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+80*DT;b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp,31.));auto x=a.tick(80),y=b.tick(80);require(x.position==y.position&&y.gnss_accepted==0&&y.gnss_rejected==1,"31m innovation accepted");});
 test("cross_track_residual_gate",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+80*DT;b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp,8.,9.));auto x=a.tick(80),y=b.tick(80);require(x.position==y.position&&y.gnss_accepted==0&&y.gnss_rejected==1,"9m unmodelled cross-track accepted");});
 test("per_fix_total_correction_cap",[]{Fixture a(false),b(true);a.through(79);b.through(79);I stamp=T0+80*DT;b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp,29.));auto x=a.tick(80),y=b.tick(80);near(y.route_s-x.route_s,5.,1e-9,"total cap");require(y.gnss_accepted==1,"cap fix should pass");speed_unchanged(x,y);});
 test("invalid_fix_does_not_poison_order",[]{Fixture b(true);b.through(79);I stamp=T0+80*DT;auto valid=b.fix_lla(0,stamp,8.),invalid=valid;invalid[0]=std::numeric_limits<double>::quiet_NaN();require(!b.pipe.fix(0,stamp,stamp,invalid),"NaN accepted");require(!b.pipe.fix(0,stamp,stamp,valid,-1),"invalid status accepted");invalid=valid;invalid[1]=181.;require(!b.pipe.fix(0,stamp,stamp,invalid),"invalid longitude accepted");require(b.pipe.fix(0,stamp,stamp,valid),"invalid fix poisoned order");require(b.tick(80).gnss_accepted==1,"valid fix missing");});
 for(int channel=0;channel<2;++channel)test(channel?"circle_rover_lever_arm":"circle_master_lever_arm",[channel]{Fixture a(false,circle(),30.),b(true,circle(),30.);a.through(79);b.through(79);I stamp=T0+80*DT;b.pipe.fix(channel,stamp,stamp,b.fix_lla(channel,stamp));auto x=a.tick(80),y=b.tick(80);near(y.route_s-x.route_s,0.,.001,"zero true phase should be unchanged on bend");require(y.gnss_accepted==1,"circle fix rejected");double half=std::asin(7.55/60.);near(y.body_velocity[0],5.*std::cos(half),.002,"body-forward speed physical projection");speed_unchanged(x,y);});
 test("grade_roof_height_lever_arm",[]{Route grade({{{0,0,0}},{{1000,0,150}}});Fixture a(false,grade),b(true,grade);a.through(79);b.through(79);I stamp=T0+80*DT;b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp));auto x=a.tick(80),y=b.tick(80);near(y.route_s-x.route_s,0.,.001,"grade roof height introduced false phase");require(y.gnss_accepted==1,"grade fix rejected");near(y.body_velocity[0],5.,.001,"grade longitudinal speed");});
 test("reset_clears_correction_state",[]{Fixture b(true);b.through(79);I stamp=T0+80*DT;b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp,8.));b.tick(80);b.pipe.reset();b.index=-1;b.through(79);require(b.last.gnss_accepted==0&&b.last.gnss_rejected==0,"counters not reset");require(b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp,8.)),"watermark not reset");require(b.tick(80).gnss_accepted==1,"fresh sequence after reset failed");});
 test("missing_startup_gnss_preserves_relative_state",[]{Fixture b(true);for(int i=0;i<=80;++i)b.tick(i,false);require(!b.last.initialized&&(b.last.flags&8),"spurious absolute pose");I stamp=T0+81*DT;require(b.pipe.fix(0,stamp,stamp,b.fix_lla(0,stamp)),"fresh retry master should queue");require(b.pipe.fix(1,stamp,stamp,b.fix_lla(1,stamp)),"fresh retry rover should queue");require(b.tick(81,false).initialized,"fresh valid pair should initialize retry");});
 std::cout<<"RESULT failures="<<failures<<'\n';return failures?1:0;
}
