#pragma once
#include "reserve_odometry/core.hpp"
#include "reserve_odometry/features.hpp"
#include "reserve_odometry/route.hpp"
#include <cstdint>
#include <deque>
#include <utility>

namespace reserve_odometry {
struct Settings {
 double wheel_divisor=3.6,init_seconds=3.,command_timeout=.5,max_speed=40.;
 double master_x=-9.873,rover_x=2.563,antenna_height=3.;
 double output_x=0.,output_z=0.,velocity_x=-9.873,velocity_z=3.;
 bool compensate_velocity_lever_arm=false;
 bool align_initial_position=false;double speed_scale=1.;
 bool allow_degraded_initialization=true;
 bool sparse_gnss_correction=false;
 bool gnss_xy_residual_correction=false;
 double gnss_gain=.25,gnss_max_step=5.,gnss_innovation_gate=30.,gnss_residual_gate=8.,gnss_max_age=1.;
 double offmap_departure=0.,offmap_angle=10.,offmap_min_span=5.,offmap_horizon=3.,offmap_max_distance=100.;
 double adhesion_accel_limit=0.,adhesion_decel_limit=3.2,slip_release_rate=.2,slip_release_hold=3.,slip_max_latch=10.;
 Vec3 origin{{55.805,37.425,170.}};
};
struct Output {
 std::int64_t stamp=0;double velocity=0.,bogie_velocity=0.,acceleration=0.,distance=0.,route_s=0.,init_rms=0.;
 BodyPose body;Vec3 position{},body_velocity{},body_angular_velocity{};int flags=0,route=-1;bool initialized=false;
 std::uint64_t gnss_accepted=0,gnss_rejected=0;double gnss_last_innovation=0.;
 bool slip=false;double slip_ratio=0.;std::uint64_t slip_events=0;
};
// Flags: bits0,1 wheel trust;4 outside map;8 no absolute init;16 init open;
// 32 degraded GNSS initialization;64 stale/missing controller;128 long output gap.
// 256 off-map;512 rejoining;1024 GNSS pair conflict;2048 adhesion supervisor active.
class Pipeline {
 struct Fix{std::int64_t stamp,arrival;Vec3 xyz;bool used=false;};
 struct Track{std::int64_t stamp;Vec3 xyz;double d,departure;};
 std::array<std::deque<Track>,2>tracks_;bool free_=false;Vec3 free_anchor_{},free_dir_{{1,0,0}};double free_d0_=0.;
 bool returning_=false;
 std::array<std::deque<Fix>,2>pair_history_;
 bool pair_conflict_=false;int good_pairs_=0;
 std::int64_t evidence_pair_=0,applied_pair_=0,rejoin_pair_=0;
 std::deque<double>rejoin_offsets_;
 Settings cfg_;Core core_;FeatureBuilder features_;std::vector<Route>routes_;
 std::array<std::deque<Fix>,2>fixes_;
 std::array<std::deque<Fix>,2>corrections_;
 std::array<std::int64_t,2>last_fix_stamp_{{0,0}};
 std::uint64_t gnss_accepted_=0,gnss_rejected_=0;double gnss_last_innovation_=0.;
 std::deque<std::pair<std::int64_t,double>>history_;
 std::vector<std::vector<std::pair<double,double>>>initial_candidates_;
 std::vector<std::vector<Vec3>>initial_shifts_;Vec3 map_shift_{},gnss_xy_bias_{};
 std::vector<std::vector<std::pair<double,double>>>degraded_candidates_;
 bool degraded_=false,retry_waiting_=false,retrying_=false;
 std::int64_t init_header0_=0,init_arrival0_=0;
 bool started_=false,frozen_=false,absolute_=false;int command_=0,route_=-1;
 std::int64_t t0_=0,last_=0,arrival0_=0,command_stamp_=-1,last_pair_=-1;
 double distance_=0.,previous_v_=0.,offset_=0.,init_rms_=0.;
 static double median(std::vector<double>x){if(x.empty())return 1e100;auto k=x.begin()+x.size()/2;std::nth_element(x.begin(),k,x.end());double v=*k;if(x.size()%2==0)v=.5*(v+*std::max_element(x.begin(),k));return v;}
 double travelled_at(std::int64_t t)const{
   if(history_.empty())return 0.;if(t<=history_.front().first)return history_.front().second;
   auto it=std::upper_bound(history_.begin(),history_.end(),t,[](std::int64_t v,const auto&a){return v<a.first;});
   if(it==history_.end())return history_.back().second;auto p=it-1;
   double w=double(t-p->first)/double(it->first-p->first);return p->second+w*(it->second-p->second);
 }
 void finish_free(double d){
   auto current=add(free_anchor_,mul(free_dir_,d-free_d0_));
   auto mapped=add(routes_[route_].pose(offset_+d).position,map_shift_);
   gnss_xy_bias_={{current[0]-mapped[0],current[1]-mapped[1],0.}};
   free_=false;returning_=true;
 }
 void recovery_track(int channel,const Fix&f){
   auto&w=tracks_[channel];
   if(!w.empty()&&f.stamp<=w.back().stamp)return;
   w.push_back({f.stamp,f.xyz,travelled_at(f.stamp),0.});
   while(w.size()>64||(f.stamp-w.front().stamp)*1e-9>cfg_.offmap_horizon)w.pop_front();
 }
 bool recovery_course(const std::deque<Track>&w,const Vec3&direction)const{
   if(w.size()<5||w.back().d-w.front().d<cfg_.offmap_min_span)return false;
   double n=double(w.size()),md=0.,mx=0.,my=0.,vd=0.,cx=0.,cy=0.;
   for(const auto&t:w){md+=t.d/n;mx+=t.xyz[0]/n;my+=t.xyz[1]/n;}
   for(const auto&t:w){vd+=(t.d-md)*(t.d-md);cx+=(t.d-md)*(t.xyz[0]-mx);cy+=(t.d-md)*(t.xyz[1]-my);}
   if(vd<=0.)return false;
   Vec3 g{{cx/vd,cy/vd,0.}};double gain=norm(g);
   if(gain<.8||gain>1.2||dot(unit(g),direction)<=.98)return false;
   double sse=0.,max_error=0.;
   for(const auto&t:w){double e=std::hypot(t.xyz[0]-mx-g[0]*(t.d-md),t.xyz[1]-my-g[1]*(t.d-md));sse+=e*e;max_error=std::max(max_error,e);}
   return max_error<=1.&&std::sqrt(sse/n)<=.5;
 }
 bool departed(int channel,const Fix&f,double x,double departure,const Fix*mate,bool other_live){
   auto&w=tracks_[channel];double d=travelled_at(f.stamp);
   BodyPose paired_body;bool paired=false;
   std::int64_t pair_epoch=mate?std::max(f.stamp,mate->stamp):0;
   if(!mate&&other_live)return free_;
   if(mate){
     Vec3 direction=free_?free_dir_:routes_[route_].pose(offset_+d).forward;
     auto aligned=add(mate->xyz,mul(direction,d-travelled_at(mate->stamp)));
     auto master=channel==0?f.xyz:aligned,rover=channel==0?aligned:f.xyz;
     if(std::abs(norm(sub(rover,master))-(cfg_.rover_x-cfg_.master_x))>.5){
       pair_conflict_=true;good_pairs_=0;tracks_[0].clear();tracks_[1].clear();
       if(free_){finish_free(distance_);++gnss_rejected_;return true;}return false;
     }
     paired_body=from_antennas(master,rover,cfg_.master_x,cfg_.antenna_height);paired=true;
     if(pair_epoch>evidence_pair_){evidence_pair_=pair_epoch;if(++good_pairs_>=3)pair_conflict_=false;}
   }
   if(pair_conflict_)return free_;
   // An applied pair remains consumed even if its first callback ended rejoin.
   if(paired&&pair_epoch<=applied_pair_)return true;
   if((free_||returning_)&&paired){
     // Rejoin is available after bridge expiry, before the local innovation gate.
     auto position=sub(paired_body.position,map_shift_);
     auto match=routes_[route_].project_xy(position,&paired_body.forward);
     bool on_map=match.distance<3.&&dot(routes_[route_].pose(match.s).forward,paired_body.forward)>.98;
     if(on_map&&pair_epoch>rejoin_pair_){
       double s=match.s;if(routes_[route_].closed())s+=routes_[route_].end()*std::round((offset_+d-s)/routes_[route_].end());
       rejoin_offsets_.push_back(s-d);while(rejoin_offsets_.size()>3)rejoin_offsets_.pop_front();rejoin_pair_=pair_epoch;
       if(rejoin_offsets_.size()==3&&*std::max_element(rejoin_offsets_.begin(),rejoin_offsets_.end())-*std::min_element(rejoin_offsets_.begin(),rejoin_offsets_.end())<=1.){
         auto current=free_?add(free_anchor_,mul(free_dir_,distance_-free_d0_)):add(routes_[route_].pose(offset_+distance_).position,add(map_shift_,gnss_xy_bias_));
         offset_=median(std::vector<double>(rejoin_offsets_.begin(),rejoin_offsets_.end()));
         auto target=add(routes_[route_].pose(offset_+distance_).position,map_shift_);
         auto bias=sub(current,target);bias[2]=0.;double error=norm(bias);
         gnss_xy_bias_=mul(bias,error>0.?std::max(0.,error-cfg_.gnss_max_step)/error:0.);
         free_=false;returning_=norm(gnss_xy_bias_)>=1.;applied_pair_=pair_epoch;++gnss_accepted_;return true;
       }
     }else if(!on_map)rejoin_offsets_.clear();
     if(returning_&&!on_map){
       // Both receivers must support a coherent course after an outage. Populate
       // both histories here: the second callback of an applied pair is skipped.
       recovery_track(channel,f);recovery_track(1-channel,*mate);
       auto direction=paired_body.forward;direction[2]=0.;direction=unit(direction);
       if(recovery_course(tracks_[0],direction)&&recovery_course(tracks_[1],direction)){
         auto mapped=add(routes_[route_].pose(offset_+distance_).position,map_shift_);
         auto current=add(mapped,gnss_xy_bias_);
         auto target=add(paired_body.position,mul(direction,distance_-d));target[2]=mapped[2];
         auto delta=sub(target,current);delta[2]=0.;double error=norm(delta);
         auto corrected=add(current,mul(delta,error>0.?std::min(1.,cfg_.gnss_max_step/error):0.));
         gnss_xy_bias_=sub(corrected,mapped);gnss_xy_bias_[2]=0.;
         applied_pair_=pair_epoch;++gnss_accepted_;
         // Re-anchor only when this bounded step actually reaches the new fix.
         if(error<=cfg_.gnss_max_step){
           free_anchor_=corrected;free_dir_=direction;free_d0_=distance_;
           free_=true;returning_=false;gnss_xy_bias_={};
         }
         return true;
       }
       ++gnss_rejected_;return true;
     }
     // During reacquisition, two initial witnesses cannot trigger the old phase
     // correction or start a new extrapolation from an obsolete route phase.
     if(returning_){++gnss_rejected_;return true;}
   }
   if(free_){auto expected=add(free_anchor_,mul(free_dir_,d-free_d0_+x));
     if(std::hypot(f.xyz[0]-expected[0],f.xyz[1]-expected[1])>5.){++gnss_rejected_;return true;}}
   w.push_back({f.stamp,f.xyz,d,departure});
   while(w.size()>64||(f.stamp-w.front().stamp)*1e-9>cfg_.offmap_horizon)w.pop_front();
   if(free_&&paired){
     free_anchor_=paired_body.position;free_anchor_[2]=add(routes_[route_].pose(offset_+d).position,map_shift_)[2];
     free_dir_=paired_body.forward;free_dir_[2]=0.;free_dir_=unit(free_dir_);free_d0_=d;applied_pair_=pair_epoch;++gnss_accepted_;return true;
   }
   if(w.size()<5||w.back().d-w.front().d<cfg_.offmap_min_span)return free_;
   double n=double(w.size()),md=0.,mx=0.,my=0.,vd=0.,cx=0.,cy=0.;
   for(const auto&t:w){md+=t.d/n;mx+=t.xyz[0]/n;my+=t.xyz[1]/n;}
   for(const auto&t:w){vd+=(t.d-md)*(t.d-md);cx+=(t.d-md)*(t.xyz[0]-mx);cy+=(t.d-md)*(t.xyz[1]-my);}
   if(vd<=0.)return free_;
   Vec3 g{{cx/vd,cy/vd,0.}};double gain=norm(g);if(gain<.8||gain>1.2)return free_;
   double sse=0.,max_error=0.;
   for(const auto&t:w){double e=std::hypot(t.xyz[0]-mx-g[0]*(t.d-md),t.xyz[1]-my-g[1]*(t.d-md));sse+=e*e;max_error=std::max(max_error,e);}
   if(max_error>1.||std::sqrt(sse/n)>.5)return free_;
   bool outside=true,inside=true;
   for(std::size_t i=w.size()-3;i<w.size();++i){outside=outside&&w[i].departure>cfg_.offmap_departure;inside=inside&&w[i].departure<5.;}
   if(free_&&inside){finish_free(distance_);return true;}
   if(!free_&&!outside)return false;
   auto a=antenna(routes_[route_].pose(offset_+w.front().d),x,cfg_.antenna_height),b=antenna(routes_[route_].pose(offset_+w.back().d),x,cfg_.antenna_height);
   Vec3 m{{b[0]-a[0],b[1]-a[1],0.}};if(!free_&&(norm(m)<1e-6||dot(unit(m),unit(g))>std::cos(cfg_.offmap_angle*.017453292519943295)))return false;
   free_dir_=unit(g);double z=add(routes_[route_].pose(offset_+w.back().d).position,map_shift_)[2];
   free_anchor_={{mx+g[0]*(w.back().d-md)-x*free_dir_[0],my+g[1]*(w.back().d-md)-x*free_dir_[1],z}};
   if(paired){free_anchor_[0]=paired_body.position[0];free_anchor_[1]=paired_body.position[1];free_dir_=paired_body.forward;free_dir_[2]=0.;free_dir_=unit(free_dir_);applied_pair_=pair_epoch;}
   rejoin_offsets_.clear();rejoin_pair_=0;
   free_d0_=w.back().d;free_=true;returning_=false;gnss_xy_bias_={};++gnss_accepted_;return true;
 }
 void correct_gnss(std::int64_t now){
   if(!cfg_.sparse_gnss_correction||!frozen_||!absolute_)return;
   // Merge receivers by sensor epoch; each fix uses its own known lever arm.
   for(;;){
     int channel=-1;
     for(int k=0;k<2;++k)if(!corrections_[k].empty()&&corrections_[k].front().stamp<=now&&
       (channel<0||corrections_[k].front().stamp<corrections_[channel].front().stamp))channel=k;
     if(channel<0)break;
     auto f=corrections_[channel].front();corrections_[channel].pop_front();
     if(history_.empty()||f.stamp<history_.front().first||(now-f.stamp)*1e-9>cfg_.gnss_max_age){++gnss_rejected_;continue;}
     std::int64_t last_received=0;
     for(const auto&h:pair_history_)if(!h.empty())last_received=std::max(last_received,h.back().stamp);
     if(last_received>0&&(f.stamp-last_received)*1e-9>cfg_.offmap_horizon){
       // An outage of BOTH streams starts a new episode; a live corrupt stream
       // never ages out its own conflict evidence merely by waiting.
       pair_conflict_=false;good_pairs_=0;for(auto&t:tracks_)t.clear();
       rejoin_offsets_.clear();rejoin_pair_=0;
     }
     auto&own_history=pair_history_[channel];own_history.push_back(f);
     while(own_history.size()>64||(now-own_history.front().stamp)*1e-9>cfg_.gnss_max_age)own_history.pop_front();
     const Fix*mate=nullptr;std::int64_t pair_gap=50000001;
     auto inspect_mate=[&](const Fix&v){auto gap=std::llabs(v.stamp-f.stamp);if(v.stamp>=history_.front().first&&v.stamp<=now&&(now-v.stamp)*1e-9<=cfg_.gnss_max_age&&gap<pair_gap){mate=&v;pair_gap=gap;}};
     for(const auto&v:pair_history_[1-channel])inspect_mate(v);for(const auto&v:corrections_[1-channel])inspect_mate(v);
     bool other_live=!pair_history_[1-channel].empty()&&(now-pair_history_[1-channel].back().stamp)*1e-9<cfg_.offmap_horizon;
     double base_s=offset_+travelled_at(f.stamp),s=base_s;
     double x=channel==0?cfg_.master_x:cfg_.rover_x;
     auto predicted=[&](double phase){return add(antenna(routes_[route_].pose(phase),x,cfg_.antenna_height),add(map_shift_,gnss_xy_bias_));};
     auto p=predicted(s);gnss_last_innovation_=std::hypot(f.xyz[0]-p[0],f.xyz[1]-p[1]);
     if(gnss_last_innovation_>cfg_.gnss_innovation_gate&&!free_&&!(returning_&&mate)){++gnss_rejected_;continue;}
     for(int iteration=0;iteration<5;++iteration){
       p=predicted(s);auto lo=predicted(s-.25),hi=predicted(s+.25);
       double dx=(hi[0]-lo[0])*2.,dy=(hi[1]-lo[1])*2.,den=dx*dx+dy*dy;
       if(den<.01)break;
       double delta=((f.xyz[0]-p[0])*dx+(f.xyz[1]-p[1])*dy)/den;
       s=std::clamp(s+std::clamp(delta,-10.,10.),base_s-cfg_.gnss_innovation_gate,base_s+cfg_.gnss_innovation_gate);
       if(!routes_[route_].closed())s=std::clamp(s,0.,routes_[route_].end());
     }
     p=predicted(s);
     if(cfg_.offmap_departure>0.&&departed(channel,f,x,std::hypot(f.xyz[0]-p[0]+gnss_xy_bias_[0],f.xyz[1]-p[1]+gnss_xy_bias_[1]),mate,other_live))continue;
     if(free_)continue;
     double residual=std::hypot(f.xyz[0]-p[0],f.xyz[1]-p[1]);
     double residual_gate=cfg_.gnss_xy_residual_correction?cfg_.gnss_innovation_gate:cfg_.gnss_residual_gate;
     if(residual>residual_gate||residual>gnss_last_innovation_+1e-9){++gnss_rejected_;continue;}
     offset_+=std::clamp(cfg_.gnss_gain*(s-base_s),-cfg_.gnss_max_step,cfg_.gnss_max_step);
     if(cfg_.gnss_xy_residual_correction){
       // Only the residual after the complete local phase projection translates
       // local XY bias. Initial map alignment, Z, core and distance are unchanged.
       double dx=cfg_.gnss_gain*(f.xyz[0]-p[0]),dy=cfg_.gnss_gain*(f.xyz[1]-p[1]);
       double step=std::hypot(dx,dy),scale=step>cfg_.gnss_max_step?cfg_.gnss_max_step/step:1.;
       gnss_xy_bias_[0]+=scale*dx;gnss_xy_bias_[1]+=scale*dy;
     }
     ++gnss_accepted_;
   }
 }
 void initialize(std::int64_t now){
   if(frozen_||retry_waiting_)return;
   auto end=init_header0_+std::int64_t(cfg_.init_seconds*1e9);
   // A fresh retry mate may arrive second with a slightly earlier sensor epoch.
   // Keep the original window end; only its pairing lower bound gains the skew.
   auto pair_begin=init_header0_-(retrying_?50000000LL:0LL);
   if(routes_.empty()){if(now>=end)frozen_=true;return;}
   for(auto&m:fixes_[0]){
     if(m.used||m.stamp<pair_begin||m.stamp>std::min(now,end))continue;
     if(retrying_&&(history_.empty()||m.stamp<history_.front().first||(now-m.stamp)*1e-9>cfg_.gnss_max_age)){m.used=true;continue;}
     const Fix*best=nullptr;std::int64_t gap=50000001;
     for(const auto&r:fixes_[1])if(!r.used&&r.stamp>=pair_begin&&r.stamp<=std::min(now,end)&&(!retrying_||(!history_.empty()&&r.stamp>=history_.front().first&&(now-r.stamp)*1e-9<=cfg_.gnss_max_age))&&std::llabs(r.stamp-m.stamp)<gap){best=&r;gap=std::llabs(r.stamp-m.stamp);}
     if(!best||gap>50000000)continue;
     m.used=true;double baseline=norm(sub(best->xyz,m.xyz));
     double baseline_error=std::abs(baseline-(cfg_.rover_x-cfg_.master_x));bool degraded=baseline_error>.5;
     if(degraded&&(!cfg_.allow_degraded_initialization||baseline_error>.25*(cfg_.rover_x-cfg_.master_x)))continue;
     auto body=from_antennas(m.xyz,best->xyz,cfg_.master_x,cfg_.antenna_height);
     auto stamp=m.stamp+(best->stamp-m.stamp)/2;double d=travelled_at(stamp);
     for(std::size_t j=0;j<routes_.size();++j){auto p=routes_[j].project_antennas(m.xyz,best->xyz,cfg_.master_x,cfg_.rover_x,cfg_.antenna_height,travelled_at(m.stamp)-d,travelled_at(best->stamp)-d);double offset=p.s-d;auto&list=degraded?degraded_candidates_[j]:initial_candidates_[j];
       if(routes_[j].closed()&&!list.empty()){double reference=list.front().first;offset+=routes_[j].end()*std::round((reference-offset)/routes_[j].end());}
       list.push_back({offset,p.cost});if(retrying_&&list.size()>512)list.erase(list.begin());if(!degraded){initial_shifts_[j].push_back(sub(body.position,routes_[j].at(p.s)));if(retrying_&&initial_shifts_[j].size()>512)initial_shifts_[j].erase(initial_shifts_[j].begin());}}
     last_pair_=stamp;
   }
   double bestcost=1e100;int best=-1;double off=0.;
   bool fallback=initial_candidates_[0].empty();
   for(std::size_t j=0;j<routes_.size();++j){auto&list=fallback?degraded_candidates_[j]:initial_candidates_[j];if(fallback&&(now<end||list.size()<3))continue;std::vector<double>offsets,costs;for(auto p:list){offsets.push_back(p.first);costs.push_back(p.second);}if(costs.empty())continue;double cost=median(costs);if(cost+1e-9<bestcost){bestcost=cost;best=int(j);off=median(offsets);}}
   if(best>=0&&bestcost<400.){route_=best;offset_=off;absolute_=true;degraded_=fallback;init_rms_=std::sqrt(bestcost);map_shift_={};if(cfg_.align_initial_position&&!fallback){for(int k=0;k<3;++k){std::vector<double>values;for(const auto&v:initial_shifts_[best])values.push_back(v[k]);map_shift_[k]=median(values);}}}
   if(now>=end){
     bool retry=cfg_.sparse_gnss_correction&&!absolute_;
     frozen_=!retry;retry_waiting_=retry;retrying_=retrying_||retry;
     fixes_[0].clear();fixes_[1].clear();
     if(retry){initial_candidates_.assign(routes_.size(),{});initial_shifts_.assign(routes_.size(),{});degraded_candidates_.assign(routes_.size(),{});}
     else{initial_candidates_.clear();initial_shifts_.clear();degraded_candidates_.clear();}
   }
 }
 public:
 explicit Pipeline(Settings cfg={},std::vector<Route>routes={}):cfg_(cfg),core_(limits(cfg)),features_(cfg.wheel_divisor),routes_(std::move(routes)){
   for(double v:{cfg_.wheel_divisor,cfg_.init_seconds,cfg_.command_timeout,cfg_.max_speed,cfg_.master_x,cfg_.rover_x,cfg_.antenna_height,cfg_.output_x,cfg_.output_z,cfg_.velocity_x,cfg_.velocity_z,cfg_.speed_scale})if(!std::isfinite(v))throw std::runtime_error("Nonfinite estimator setting");
   if(cfg_.wheel_divisor<=0.||cfg_.init_seconds<0.||cfg_.init_seconds>30.||cfg_.command_timeout<=0.||cfg_.max_speed<=0.||cfg_.rover_x<=cfg_.master_x||!finite(cfg_.origin)||std::abs(cfg_.origin[0])>=90.||std::abs(cfg_.origin[1])>180.)throw std::runtime_error("Invalid estimator settings");
   if(cfg_.speed_scale<.9||cfg_.speed_scale>1.1)throw std::runtime_error("Wheel scale outside supported calibration range");
   for(double v:{cfg_.gnss_gain,cfg_.gnss_max_step,cfg_.gnss_innovation_gate,cfg_.gnss_residual_gate,cfg_.gnss_max_age})if(!std::isfinite(v)||v<=0.)throw std::runtime_error("Invalid GNSS correction setting");
   for(double v:{cfg_.offmap_departure,cfg_.offmap_angle,cfg_.offmap_min_span,cfg_.offmap_horizon,cfg_.offmap_max_distance})if(!std::isfinite(v)||v<0.)throw std::runtime_error("Invalid off-map setting");
   for(double v:{cfg_.adhesion_accel_limit,cfg_.adhesion_decel_limit,cfg_.slip_release_rate,cfg_.slip_release_hold,cfg_.slip_max_latch})if(!std::isfinite(v)||v<0.)throw std::runtime_error("Invalid adhesion setting");
   if(cfg_.adhesion_accel_limit>0.&&(cfg_.adhesion_decel_limit<=0.||cfg_.slip_max_latch<=0.))throw std::runtime_error("Adhesion supervisor needs positive decel limit and latch");
   if(cfg_.gnss_gain>1.)throw std::runtime_error("GNSS correction gain must be <=1");
   initial_candidates_.resize(routes_.size());initial_shifts_.resize(routes_.size());degraded_candidates_.resize(routes_.size());
 }
 static AdhesionLimits limits(const Settings&c){return {c.adhesion_accel_limit,c.adhesion_decel_limit,c.slip_release_rate,c.slip_release_hold,c.slip_max_latch};}
 void reset(){core_=Core(limits(cfg_));features_=FeatureBuilder(cfg_.wheel_divisor);for(auto&f:fixes_)f.clear();for(auto&f:corrections_)f.clear();last_fix_stamp_={{0,0}};gnss_accepted_=gnss_rejected_=0;gnss_last_innovation_=0.;history_.clear();initial_candidates_.assign(routes_.size(),{});initial_shifts_.assign(routes_.size(),{});degraded_candidates_.assign(routes_.size(),{});map_shift_={};gnss_xy_bias_={};for(auto&t:tracks_)t.clear();free_=false;returning_=false;for(auto&h:pair_history_)h.clear();pair_conflict_=false;good_pairs_=0;evidence_pair_=applied_pair_=rejoin_pair_=0;rejoin_offsets_.clear();free_anchor_={};free_dir_={{1,0,0}};free_d0_=0.;degraded_=false;retry_waiting_=retrying_=false;init_header0_=init_arrival0_=0;started_=frozen_=absolute_=false;command_=0;route_=-1;t0_=last_=arrival0_=0;command_stamp_=last_pair_=-1;distance_=previous_v_=offset_=init_rms_=0.;}
 bool wheel(int channel,std::int64_t stamp,double value,std::int64_t arrival=0){return stamp>0&&value<=cfg_.max_speed*cfg_.wheel_divisor&&!(arrival>0&&stamp>arrival+50000000LL)&&features_.wheel(channel,stamp,value);}
 bool command(std::int64_t stamp,int value,std::int64_t arrival=0){if(stamp<=0||value<-15||value>15||stamp<=command_stamp_||(arrival>0&&stamp>arrival+50000000LL))return false;command_=value;command_stamp_=stamp;return true;}
 bool fix(int channel,std::int64_t stamp,std::int64_t arrival,const Vec3&lla,int status=0){
   if(channel<0||channel>1||stamp<=0||status<0||!finite(lla)||std::abs(lla[0])>90.||std::abs(lla[1])>180.||(arrival>0&&stamp>arrival+50000000LL))return false;
   if(frozen_){
     if(!cfg_.sparse_gnss_correction||!absolute_||stamp<=last_fix_stamp_[channel])return false;
     last_fix_stamp_[channel]=stamp;
     if((arrival>0&&(arrival-stamp)*1e-9>cfg_.gnss_max_age)||stamp<t0_){++gnss_rejected_;return false;}
     auto&q=corrections_[channel];q.push_back({stamp,arrival,enu(lla,cfg_.origin)});while(q.size()>32){q.pop_front();++gnss_rejected_;}return true;
   }
   if(retrying_){
     // Retry never resurrects stale history or an already seen receiver epoch.
     if(stamp<=last_fix_stamp_[channel]||history_.empty()||stamp<history_.front().first||
       (arrival>0&&(arrival-stamp)*1e-9>cfg_.gnss_max_age)||(last_-stamp)*1e-9>cfg_.gnss_max_age)return false;
     if(retry_waiting_){init_header0_=stamp;init_arrival0_=arrival>0?arrival:arrival0_+(last_-t0_);retry_waiting_=false;}
   }
   if(started_&&(stamp<init_header0_-(retrying_?50000000LL:0LL)||stamp>init_header0_+cfg_.init_seconds*1e9||arrival>init_arrival0_+cfg_.init_seconds*1e9))return false;
   auto&f=fixes_[channel];if(!f.empty()&&stamp<=f.back().stamp)return false;last_fix_stamp_[channel]=stamp;f.push_back({stamp,arrival,enu(lla,cfg_.origin)});while(f.size()>512)f.pop_front();return true;
 }
 bool step(std::int64_t stamp,std::int64_t arrival,Output&out){
   if(stamp<=0||(started_&&stamp<=last_))return false;
   if(!started_){t0_=stamp;arrival0_=arrival;init_header0_=stamp;init_arrival0_=arrival;history_.push_back({stamp,0.});}
   std::array<float,26>x;double t;int cmd=command_;bool stale=command_stamp_<0||(stamp-command_stamp_)*1e-9>cfg_.command_timeout;
   if(stale)cmd=0;if(!features_.build(stamp,cmd,x,t))return false;
   auto e=core_.step(t,x);e.velocity*=cfg_.speed_scale;e.acceleration*=cfg_.speed_scale;double dt=started_?(stamp-last_)*1e-9:0.;
   // A prolonged publication gap is explicitly flagged; ROS timer normally keeps
   // it below 0.05s. The trapezoid uses measured elapsed time, not a hidden clamp.
   double distance_before=distance_;
   distance_+=.5*(previous_v_+e.velocity)*dt;previous_v_=e.velocity;last_=stamp;started_=true;
   if(cfg_.sparse_gnss_correction&&cfg_.gnss_xy_residual_correction){
     // Fixed spatial support: no decay at standstill, no change to initial map alignment.
     double decay=std::exp(-std::max(0.,distance_-distance_before)/100.);
     gnss_xy_bias_[0]*=decay;gnss_xy_bias_[1]*=decay;
   }
   if(free_&&distance_-free_d0_>cfg_.offmap_max_distance)finish_free(distance_);
   if(returning_&&std::hypot(gnss_xy_bias_[0],gnss_xy_bias_[1])<1.)returning_=false;
   history_.push_back({stamp,distance_});while(history_.size()>1000)history_.pop_front();
   // Re-check arrival limits for pre-start queued fixes before using them.
   for(auto&f:fixes_)for(auto&v:f)if(v.arrival>init_arrival0_+cfg_.init_seconds*1e9)v.used=true;
   initialize(stamp);
   correct_gnss(stamp);
   double v=e.velocity;
   out=Output{};out.stamp=stamp;out.distance=distance_;out.acceleration=e.acceleration;out.bogie_velocity=e.velocity;out.velocity=v;out.route=route_;out.initialized=absolute_;out.init_rms=init_rms_;
   out.flags=e.flags|(absolute_?0:8)|(frozen_?0:16)|(degraded_?32:0)|(stale?64:0)|(dt>.5?128:0)|(e.slip?2048:0);
   out.slip=e.slip;out.slip_ratio=e.slip_ratio;out.slip_events=core_.slip_events();
   if(returning_)out.flags|=512;
   if(pair_conflict_)out.flags|=1024;
   out.gnss_accepted=gnss_accepted_;out.gnss_rejected=gnss_rejected_;out.gnss_last_innovation=gnss_last_innovation_;
   if(absolute_&&free_){out.route_s=offset_+distance_;out.body=body_pose(add(free_anchor_,mul(free_dir_,distance_-free_d0_)),free_dir_);out.position=antenna(out.body,cfg_.output_x,cfg_.output_z);out.flags|=256;out.body_velocity={{v,0,0}};}
   else if(absolute_){out.route_s=offset_+distance_;out.body=routes_[route_].pose(out.route_s);out.body.position=add(out.body.position,add(map_shift_,gnss_xy_bias_));out.position=antenna(out.body,cfg_.output_x,cfg_.output_z);if(routes_[route_].outside(out.route_s))out.flags|=4;if(cfg_.compensate_velocity_lever_arm)out.velocity*=routes_[route_].speed_factor(out.route_s,cfg_.velocity_x,cfg_.velocity_z);
     auto a=routes_[route_].pose(out.route_s-.05),b=routes_[route_].pose(out.route_s+.05);
     auto world_v=mul(sub(antenna(b,cfg_.output_x,cfg_.output_z),antenna(a,cfg_.output_x,cfg_.output_z)),10.*v);
     auto world_w=mul(add(add(cross(out.body.forward,sub(b.forward,a.forward)),cross(out.body.left,sub(b.left,a.left))),cross(out.body.up,sub(b.up,a.up))),5.*v);
     out.body_velocity={{dot(world_v,out.body.forward),dot(world_v,out.body.left),dot(world_v,out.body.up)}};out.body_angular_velocity={{dot(world_w,out.body.forward),dot(world_w,out.body.left),dot(world_w,out.body.up)}};
   }else{out.body=body_pose({{distance_,0,0}},{{1,0,0}});out.position=out.body.position;out.body_velocity={{v,0,0}};}
   return true;
 }
 bool started()const{return started_;}bool frozen()const{return frozen_;}std::int64_t last_stamp()const{return last_;}
};
}
