#define main unused_legacy_main
#include "native_sparse_gnss.cpp"
#undef main

struct Event {int tick,channel;I relative_stamp;int status=0;double baseline_delta=0.;bool after=false;};
std::vector<Event> paired(I stamp,int tick,double delta=0.,bool after=false){return {{tick,0,stamp,0,0.,after},{tick,1,stamp,0,delta,after}};}
std::vector<Event> concat(std::vector<Event>a,const std::vector<Event>&b){a.insert(a.end(),b.begin(),b.end());return a;}
std::vector<Output> reference_outputs;
int audit_failures=0;

void audit(const std::string&name,std::vector<Event>events,int expected_first,double start=50.){
 Settings c;c.sparse_gnss_correction=true;c.gnss_xy_residual_correction=true;
 Route route=line();Pipeline pipe(c,{route});std::vector<Output>outputs;
 int first=-1,queued=0,rejected=0;double max_error=0.;Output last;
 auto deliver=[&](int tick,bool after){for(const auto&e:events)if(e.tick==tick&&e.after==after){
   auto p=antenna(route.pose(start+5.*e.relative_stamp*1e-9),e.channel==0?-9.873:2.563,3.);p[0]+=e.baseline_delta;
   if(pipe.fix(e.channel,T0+e.relative_stamp,T0+tick*DT,observation_lla(p),e.status))++queued;else ++rejected;
 }};
 for(int i=0;i<=200;++i){I t=T0+i*DT;deliver(i,false);pipe.wheel(0,t,18.,t);pipe.wheel(1,t,18.,t);pipe.command(t,0,t);require(pipe.step(t,t,last),"synthetic output");deliver(i,true);outputs.push_back(last);
   if(last.initialized){if(first<0)first=i;max_error=std::max(max_error,norm(sub(last.position,route.pose(start+5.*i*.05).position)));}
 }
 bool core=true;if(reference_outputs.empty())reference_outputs=outputs;else for(std::size_t i=0;i<outputs.size();++i)core=core&&outputs[i].velocity==reference_outputs[i].velocity&&outputs[i].distance==reference_outputs[i].distance;
 bool passed=first==expected_first&&(pipe.frozen()==(expected_first>=0))&&core;
 if(expected_first>=0&&name.find("degraded")==std::string::npos)passed=passed&&max_error<1e-5;
 if(!passed)++audit_failures;
 std::cout<<std::setprecision(17)<<"{\"case\":\""<<name<<"\",\"first_absolute_tick\":"<<first<<",\"first_absolute_s\":"<<(first<0?-1:first*.05)<<",\"expected_tick\":"<<expected_first<<",\"final_initialized\":"<<(last.initialized?"true":"false")<<",\"final_frozen\":"<<(pipe.frozen()?"true":"false")<<",\"queue_accepted\":"<<queued<<",\"queue_rejected\":"<<rejected<<",\"max_analytic_position_error_m\":"<<max_error<<",\"final_flags\":"<<last.flags<<",\"core_distance_invariant\":"<<(core?"true":"false")<<",\"pass\":"<<(passed?"true":"false")<<"}\n";
}
int main(){
 audit("clean_pair",paired(100000000,2),2);
 audit("master_only_initial_later_both",concat({{2,0,100000000}},paired(4000000000LL,80)),80);
 audit("rover_only_initial_later_both",concat({{2,1,100000000}},paired(4000000000LL,80)),80);
 audit("async40ms",{{2,0,100000000},{3,1,140000000}},3);
 audit("async50ms_boundary",{{2,0,100000000},{3,1,150000000}},3);
 audit("async50ms_plus1ns",concat({{2,0,100000000},{4,1,150000001}},paired(4000000000LL,80)),80);
 audit("first_pair3point1s",concat(paired(3100000000LL,62),paired(4000000000LL,80)),62);
 audit("pair_header2point95_arrival3point1s",paired(2950000000LL,62),62);
 audit("pair_exact3s_before_step",paired(3000000000LL,60),60);
 audit("pair_exact3s_after_step",paired(3000000000LL,60,0.,true),61);
 audit("invalid_master_status",concat({{2,0,100000000,-1},{2,1,100000000}},paired(4000000000LL,80)),80);
 audit("precommand_pair_header",paired(-50000000,0),-1);
 audit("degraded_two_pairs",concat(paired(100000000,2,.7),paired(200000000,4,.7)),-1);
 audit("degraded_three_pairs",concat(concat(paired(100000000,2,.7),paired(200000000,4,.7)),paired(300000000,6,.7)),60);
 audit("midroute500m_clean_pair",paired(100000000,2),2,500.);
 std::cout<<"{\"audit_failures\":"<<audit_failures<<"}\n";return audit_failures?1:0;
}
