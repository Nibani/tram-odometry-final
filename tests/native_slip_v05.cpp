#include "reserve_odometry/pipeline.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
using namespace reserve_odometry;
using I=std::int64_t;
constexpr I T0=10000000000LL,DT=50000000LL;
int failures=0;
void require(bool value,const std::string&label){if(!value)throw std::runtime_error(label);}
void test(const std::string&name,const std::function<void()>&body){try{body();std::cout<<"PASS "<<name<<'\n';}catch(const std::exception&e){++failures;std::cout<<"FAIL "<<name<<": "<<e.what()<<'\n';}}
int profile=0;
double truth(double t){if(profile)return t<10.?15.:std::max(0.,15.-3.*(t-10.));return t<10.?.8*t:8.;}
int command(double t){if(profile)return t<10.?0:-15;return t<10.?6:0;}
struct Run{std::vector<Output>out;std::vector<double>t;double worst(double a,double b)const{double w=0.;for(std::size_t i=0;i<t.size();++i)if(t[i]>=a&&t[i]<=b)w=std::max(w,std::abs(out[i].velocity-truth(t[i])));return w;}
 bool any(double a,double b,int bit)const{for(std::size_t i=0;i<t.size();++i)if(t[i]>=a&&t[i]<=b&&(out[i].flags&bit))return true;return false;}
 const Output&at(double s)const{for(std::size_t i=0;i<t.size();++i)if(t[i]>=s)return out[i];return out.back();}};
Settings settings(double accel){Settings c;c.adhesion_accel_limit=accel;return c;}
Run simulate(Settings c,const std::function<double(double)>&gain,double seconds=60.,const std::function<double(int,double,double)>&raw={}){
 Pipeline p(c,{});Run r;
 for(int i=0;i*DT*1e-9<=seconds;++i){I s=T0+i*DT;double t=i*DT*1e-9;
  if(i%2==0)for(int ch=0;ch<2;++ch){double v=truth(t)*gain(t)*3.6;if(raw)v=raw(ch,t,v);p.wheel(ch,s-DT/5,v,s);}
  require(p.command(s,command(t),s),"command rejected");Output o;if(p.step(s,s,o)){r.out.push_back(o);r.t.push_back(t);}
 }
 return r;
}
double ramp(double t,double a,double b,double edge){return std::clamp(std::min((t-a)/edge,(b-t)/edge),0.,1.);}
int main(){
 auto clean=[](double){return 1.;};
 auto slip=[](double t){return 1.+.25*ramp(t,30.,33.,.4);};
 auto skid=[](double t){return 1.-.5*ramp(t,30.,32.,1.);};
 test("clean profile is bit exact with supervisor disabled",[&]{auto a=simulate(settings(1.65),clean),b=simulate(settings(0.),clean);require(a.out.size()==b.out.size(),"length");for(std::size_t i=0;i<a.out.size();++i)require(a.out[i].velocity==b.out[i].velocity&&a.out[i].distance==b.out[i].distance&&a.out[i].flags==b.out[i].flags&&!a.out[i].slip,"clean output changed");});
 test("common traction slip is flagged and bounded",[&]{auto a=simulate(settings(1.65),slip),b=simulate(settings(0.),slip);require(a.any(30.,33.5,2048),"slip flag missing");require(!b.any(0.,60.,2048),"disabled flagged");require(b.worst(30.,34.)>1.8,"injection too weak");require(a.worst(30.,34.)<.6*b.worst(30.,34.),"slip error not reduced");
  double ratio=0.;for(std::size_t i=0;i<a.t.size();++i)if(a.out[i].slip)ratio=std::max(ratio,a.out[i].slip_ratio);require(ratio>.1&&ratio<.5,"slip ratio "+std::to_string(ratio));require(a.at(40.).slip_events>=1,"event counter");});
 test("partial wheel slide keeps model speed and releases after readhesion",[&]{auto a=simulate(settings(1.65),skid),b=simulate(settings(0.),skid);require(a.any(30.,31.,2048)&&(a.at(31.).flags&3)==3,"slide not flagged");require(b.worst(30.,32.)>1.5,"injection too weak");require(a.worst(30.,32.5)<.5*b.worst(30.,32.),"slide error "+std::to_string(a.worst(30.,32.5)));require(!a.any(34.,60.,2048),"latch not released");require(a.worst(35.,60.)<.05,"estimate not recovered");});
 test("persistent slip is released by maximum latch",[&]{auto c=settings(1.65);c.slip_max_latch=4.;auto a=simulate(c,[](double t){return 1.+.3*ramp(t,20.,59.,.4);});require(a.any(20.,21.,2048),"slip not flagged");require(!a.any(25.,58.,2048),"latch exceeded limit");for(const auto&o:a.out)require(std::isfinite(o.velocity)&&std::isfinite(o.distance)&&o.velocity<20.,"divergent output");});
 test("nonfinite negative and huge wheel values are rejected",[&]{auto raw=[](int ch,double t,double v){if(t<25.||t>27.)return v;return ch?-5.:std::numeric_limits<double>::quiet_NaN();};auto a=simulate(settings(1.65),clean,40.,raw);
  auto b=simulate(settings(1.65),clean,40.,[](int,double t,double v){return t>=25.&&t<=27.?(t<26.?1e9:std::numeric_limits<double>::infinity()):v;});
  for(const auto*r:{&a,&b})for(const auto&o:r->out)require(std::isfinite(o.velocity)&&std::isfinite(o.distance)&&std::isfinite(o.slip_ratio),"nonfinite output");require(a.worst(25.,30.)<.3&&b.worst(25.,30.)<.3,"invalid values leaked");require(a.worst(35.,40.)<.05&&b.worst(35.,40.)<.05,"no recovery");});
 test("invalid adhesion settings rejected",[&]{for(double v:{-1.,std::numeric_limits<double>::quiet_NaN()}){auto c=settings(1.65);c.adhesion_decel_limit=v;bool thrown=false;try{Pipeline p(c,{});}catch(const std::exception&){thrown=true;}require(thrown,"decel accepted");}auto c=settings(1.65);c.slip_max_latch=0.;bool thrown=false;try{Pipeline p(c,{});}catch(const std::exception&){thrown=true;}require(thrown,"zero latch accepted");});
 test("emergency braking at 3 m/s2 is not a slide",[&]{profile=1;auto c=settings(1.65);auto a=simulate(c,clean,20.),b=simulate(settings(0.),clean,20.);profile=0;require(!a.any(0.,20.,2048),"false slide");require(std::abs(a.at(19.).distance-b.at(19.).distance)<.5,"distance diverged");require(a.worst(10.,16.)<=b.worst(10.,16.)+.05,"speed error "+std::to_string(a.worst(10.,16.))+" off "+std::to_string(b.worst(10.,16.)));});
 std::cout<<"SLIP_V05_RESULT failures="<<failures<<'\n';return failures?1:0;
}
