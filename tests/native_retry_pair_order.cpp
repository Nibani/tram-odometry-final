#include "retry_test_helpers.hpp"

int fixed_tick(int candidate_tick){
#ifdef RETRY_CANDIDATE
 return candidate_tick;
#else
 return -1;
#endif
}
void admission_and_reset(){
 Fixture f(true);for(int i=0;i<80;++i)f.tick(i,false);
 I now=T0+80*DT;
 require(f.pipe.fix(0,now,now,f.fix_lla(0,now)),"retry anchor rejected");
 require(!f.pipe.fix(0,now,now,f.fix_lla(0,now)),"duplicate anchor admitted");
 require(!f.pipe.fix(0,now-1,now,f.fix_lla(0,now-1)),"out-of-order anchor admitted");
 auto bad=f.fix_lla(1,now);bad[0]=std::numeric_limits<double>::quiet_NaN();
 require(!f.pipe.fix(1,now,now,bad),"nonfinite retry fix admitted");
 require(!f.pipe.fix(1,now,now,f.fix_lla(1,now),-1),"invalid status retry fix admitted");
 require(!f.pipe.fix(1,now+50000001,now,f.fix_lla(1,now+50000001)),"future>50ms admitted");
 require(!f.pipe.fix(1,now-1000000001,now,f.fix_lla(1,now-1000000001)),"stale retry fix admitted");
 require(f.pipe.fix(1,now,now,f.fix_lla(1,now)),"invalid fix poisoned receiver watermark");
 require(f.tick(80,false).initialized,"same-epoch control did not initialize");
 f.pipe.reset();f.index=-1;
 for(int i=0;i<80;++i)require(!f.tick(i,false).initialized,"reset leaked old absolute state or pair");
 require(f.pipe.fix(1,now,now,f.fix_lla(1,now)),"reset retained receiver watermark");
 require(!f.tick(80,false).initialized,"reset retained opposite receiver queue");
 require(f.pipe.fix(0,now+DT,now+DT,f.fix_lla(0,now+DT)),"reset clean mate rejected");
 require(f.tick(81,false).initialized,"reset failed normal later-mate recovery");
 near(f.last.distance,20.25,1e-9,"reset distance timeline");
 std::cout<<"{\"case\":\"admission_and_reset\",\"pass\":true}\n";
}
void finite_coverage(){
 for(int channel=0;channel<2;++channel){Fixture f(true);int first=-1;
  for(int i=0;i<=200;++i){I now=T0+i*DT;
   if(i==80)require(f.pipe.fix(channel,now,now,f.fix_lla(channel,now)),"coverage anchor rejected");
   if(i==81){I earlier=T0+3960000000LL;bool accepted=f.pipe.fix(1-channel,earlier,now,f.fix_lla(1-channel,earlier));
#ifdef RETRY_CANDIDATE
    require(accepted,"coverage earlier mate rejected");
#else
    require(!accepted,"baseline no longer reproduces earlier-mate defect");
#endif
   }
   auto out=f.tick(i,false);require(out.stamp==now,"missing or nonmonotonic output epoch");
   require(std::isfinite(out.velocity)&&std::isfinite(out.distance)&&finite(out.position)&&finite(out.body_velocity),"nonfinite output");
   near(out.velocity,5.,1e-12,"continuous speed");near(out.distance,.25*i,1e-10,"continuous full distance");
   if(out.initialized){if(first<0)first=i;near(norm(sub(out.position,f.route.pose(50.+.25*i).position)),0.,1e-5,"analytic moving-start position");}
  }
  require(first==fixed_tick(81),"finite-coverage first absolute epoch");
 }
 std::cout<<"{\"case\":\"finite_coverage_both_orders\",\"outputs_per_order\":201,\"pass\":true}\n";
}
int main(){
 audit("retry_same_epoch_control",paired(4000000000LL,80),80);
 audit("retry_later_mate_control",{{80,0,3960000000LL},{80,1,4000000000LL}},80);
 for(int channel=0;channel<2;++channel){
  std::string c=channel?"rover":"master";
  audit("retry_earlier40ms_"+c,{{80,channel,4000000000LL},{80,1-channel,3960000000LL}},fixed_tick(80));
  audit("retry_earlier50ms_"+c,{{80,channel,4000000000LL},{80,1-channel,3950000000LL}},fixed_tick(80));
  audit("retry_earlier40ms_next_arrival_"+c,{{80,channel,4000000000LL},{81,1-channel,3960000000LL}},fixed_tick(81));
  audit("retry_earlier40ms_after_timer_"+c,{{80,channel,4000000000LL},{80,1-channel,3960000000LL,0,0.,true}},fixed_tick(81));
  audit("retry_earlier50ms_plus1ns_"+c,{{80,channel,4000000000LL},{80,1-channel,3949999999LL}},-1);
  audit("retry_stale_mate_"+c,{{80,channel,4000000000LL},{104,1-channel,3960000000LL}},-1);
 }
 audit("retry_future_pair_waits_for_epoch",{{80,0,4050000000LL},{80,1,4010000000LL}},fixed_tick(81));
 audit("retry_future_rejection_does_not_poison",{{80,0,4050000001LL},{80,1,3960000000LL},{80,0,4000000000LL}},80);
 audit("old_window_queue_not_reused",{{80,1,4000000000LL},{140,0,6990000000LL},{141,1,7010000000LL}},-1);
 audit("old_window_epoch_duplicate_rejected",{{80,1,4000000000LL},{140,0,6990000000LL},{141,1,7010000000LL},{141,0,6990000000LL}},-1);
 try{admission_and_reset();}catch(const std::exception&e){++audit_failures;std::cout<<"FAIL admission_and_reset "<<e.what()<<'\n';}
 try{finite_coverage();}catch(const std::exception&e){++audit_failures;std::cout<<"FAIL finite_coverage "<<e.what()<<'\n';}
 std::cout<<"{\"audit_failures\":"<<audit_failures<<"}\n";return audit_failures?1:0;
}
