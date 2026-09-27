#pragma once
#include "reserve_odometry/drive_model.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>

namespace reserve_odometry {
struct Estimate {double velocity=0.,acceleration=0.;int flags=3;bool slip=false;double slip_ratio=0.;};
struct AdhesionLimits {double accel=0.,decel=0.,release_rate=.2,release_hold=3.,max_latch=10.;};
// Exact streaming port of observer_v3.py, selected before final holdout.
// All quantities are specific acceleration; physical mass/torque are not identified.
class Core {
 double d_=0.,v_=0.,elapsed_=0.,last_t_=0.;bool first_=true,initialized_=false,hard_common_=false;
 std::array<bool,2> bad_{{false,false}};
 std::array<double,2> recover_{{0.,0.}},prev_raw_{{0.,0.}},prev_source_{{-1e30,-1e30}},raw_rate_{{0.,0.}};
 AdhesionLimits limits_;bool slip_=false,readhered_=false;int slip_sign_=0;double slip_time_=0.,slip_recover_=0.,slip_ratio_=0.;std::uint64_t slip_events_=0;
 public:
 Core()=default;
 explicit Core(AdhesionLimits limits):limits_(limits){}
 Estimate step(double t,const std::array<float,26>& x) {
   if(!std::isfinite(t)) return {v_,0.,3};
   for(float a:x) if(!std::isfinite(a)) return {v_,0.,3};
   if(!first_ && t<last_t_) return {v_,0.,3};
   double dt=first_?.05:std::clamp(t-last_t_,0.,.5);first_=false;last_t_=t;
   std::array<double,2> z{{x[15],x[16]}},age{{x[5],x[6]}},slope{{x[9],x[10]}};
   std::array<bool,2> fresh{{age[0]<.35,age[1]<.35}},valid{{false,false}};
   if(!initialized_ && (fresh[0]||fresh[1])) {v_=((fresh[0]?z[0]:0.)+(fresh[1]?z[1]:0.))/(int(fresh[0])+int(fresh[1]));initialized_=true;}
   double am=std::clamp(drive::predict({x[18],v_,x[19],x[20],x[21]}),-2.2,1.6);
   double a=std::clamp(am+d_,-2.2,1.6),pred=std::max(0.,v_+a*dt);
   constexpr double gate=.35;
   double threshold=std::sqrt(gate*gate+2.25*elapsed_*elapsed_);
   for(int j=0;j<2;++j) {
     double source=t-age[j];
     if(fresh[j] && source>prev_source_[j]+1e-5) {
       if(prev_source_[j]>-1e20) raw_rate_[j]=(x[j]-prev_raw_[j])/std::max(source-prev_source_[j],1e-6);
       prev_raw_[j]=x[j];prev_source_[j]=source;
     }
   }
   if(fresh[0]&&fresh[1]&&std::abs(raw_rate_[0])>6.&&std::abs(raw_rate_[1])>6.&&raw_rate_[0]*raw_rate_[1]>0.&&std::abs(.5*(z[0]+z[1])-pred)>.5) hard_common_=true;
   if(hard_common_&&fresh[0]&&fresh[1]&&std::abs(.5*(z[0]+z[1])-pred)<gate)hard_common_=false;
   if(limits_.accel>0.&&fresh[0]&&fresh[1]){
     double reference=std::clamp(am+d_,-2.2,1.6);
     int sign=slope[0]>std::max(limits_.accel,reference+.5)&&slope[1]>std::max(limits_.accel,reference+.5)?1:slope[0]<std::min(-limits_.decel,reference-.5)&&slope[1]<std::min(-limits_.decel,reference-.5)?-1:0;
     bool implausible=sign!=0;
     if(slip_&&sign==-slip_sign_)readhered_=true;
     if(!slip_&&implausible&&initialized_){
       slip_=true;slip_sign_=sign;readhered_=false;slip_time_=slip_recover_=0.;++slip_events_;
       double onset=std::max(0.,std::max(0.,.5*((x[0]-.3*slope[0])+(x[1]-.3*slope[1])))+.3*reference);
       pred=slope[0]>0.?std::min(pred,onset):std::max(pred,onset);
     }
     if(slip_){
       double zc=.5*(z[0]+z[1]);slip_time_+=dt;
       double gap=std::abs(zc-pred);
       slip_recover_=!implausible&&std::abs(z[0]-z[1])<.15&&std::abs(slope[0]-slope[1])<.7?slip_recover_+dt:0.;
       if((slip_recover_>=.3&&(readhered_||gap<gate+limits_.release_rate*slip_time_))||slip_recover_>=limits_.release_hold||slip_time_>=limits_.max_latch){slip_=false;hard_common_=false;bad_={{false,false}};recover_={{1.,1.}};}
       else{v_=pred;elapsed_+=dt;slip_ratio_=(zc-v_)/std::max(v_,.5);return {v_,a,3,true,slip_ratio_};}
     }
   }else if(slip_){v_=pred;elapsed_+=dt;slip_time_+=dt;if(slip_time_>=limits_.max_latch)slip_=false;return {v_,a,3,true,slip_ratio_};}
   slip_ratio_=0.;
   bool coherent=fresh[0]&&fresh[1]&&std::abs(z[0]-z[1])<.15&&std::abs(slope[0]-slope[1])<.7&&!hard_common_;
   for(int j=0;j<2;++j) {
     bool plausible=slope[j]<1.65&&slope[j]>-2.35;
     if(coherent||(fresh[j]&&plausible&&std::abs(z[j]-pred)<threshold)) {
       recover_[j]+=dt;
       if(coherent||!bad_[j]||recover_[j]>=.15){bad_[j]=false;valid[j]=true;}
     }else{bad_[j]=true;recover_[j]=0.;}
   }
   if(valid[0]||valid[1]) {
     int n=int(valid[0])+int(valid[1]);v_=((valid[0]?z[0]:0.)+(valid[1]?z[1]:0.))/n;
     if(v_<.012&&pred<.08)v_=0.;
     double obs_a=((valid[0]?slope[0]:0.)+(valid[1]?slope[1]:0.))/n;
     if(std::abs(obs_a-am)<1.&&valid[0]&&valid[1])d_=std::clamp(d_+(1.-std::exp(-dt/3.))*((obs_a-am)-d_),-.6,.6);
     elapsed_=0.;
   }else {v_=pred;elapsed_+=dt;}
   return {v_,a,int(bad_[0])+2*int(bad_[1])};
 }
 double outage_seconds() const {return elapsed_;}
 std::uint64_t slip_events() const {return slip_events_;}
};
}
