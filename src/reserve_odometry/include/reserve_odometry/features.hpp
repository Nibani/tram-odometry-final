#pragma once
#include <array>
#include <deque>
#include <cstdint>
#include <cmath>
#include <algorithm>
namespace reserve_odometry {
struct Sample {std::int64_t stamp;double value;};
class FeatureBuilder {
 std::array<std::deque<Sample>,2> wheels_;
 std::int64_t first_cmd_=0,last_cmd_=0;bool initialized_=false;
 std::array<double,3> ema_{};double divisor_=3.6;
 public:
 explicit FeatureBuilder(double divisor=3.6):divisor_(divisor){}
 bool wheel(int channel,std::int64_t stamp,double raw) {
   if(channel<0||channel>1||!std::isfinite(raw)||raw<0.||!std::isfinite(divisor_)||divisor_<=0.)return false;
   auto& w=wheels_[channel];if(!w.empty()&&stamp<=w.back().stamp)return false;
   w.push_back({stamp,raw/divisor_});while(w.size()>2048)w.pop_front();return true;
 }
 bool build(std::int64_t stamp,int command,std::array<float,26>& x,double& t) {
   if(command<-15||command>15||(initialized_&&stamp<=last_cmd_))return false;
   if(!initialized_) {first_cmd_=stamp;ema_.fill(command);initialized_=true;}
   else {double dt=(stamp-last_cmd_)*1e-9;const double tau[]={.3,1.,3.};for(int j=0;j<3;++j)ema_[j]+=(1.-std::exp(-dt/tau[j]))*(command-ema_[j]);}
   last_cmd_=stamp;t=(stamp-first_cmd_)*1e-9;x.fill(0.f);x[5]=x[6]=10.f;
   for(int c=0;c<2;++c) {
     const auto& w=wheels_[c];int i=int(w.size())-1;while(i>=0&&w[i].stamp>stamp)--i;if(i<0)continue;
     double age=(stamp-w[i].stamp)*1e-9,v=w[i].value;int p=std::max(0,i-1);
     double delta=(w[i].stamp-w[p].stamp)*1e-9;double short_s=delta>1e-6?(v-w[p].value)/delta:0.;
     std::array<double,3>slope{},lag{};const std::int64_t ns[]={300000000,1000000000,3000000000};
     for(int a=0;a<3;++a) {int j=i;auto target=w[i].stamp-ns[a];while(j>0&&w[j].stamp>target)--j;
       double dt=(w[i].stamp-w[j].stamp)*1e-9;slope[a]=std::clamp(dt>1e-6?(v-w[j].value)/dt:0.,-4.,4.);lag[a]=w[j].value;}
     x[c]=v;x[5+c]=std::min(age,10.);x[7+c]=std::clamp(short_s,-4.,4.);
     x[9+c]=slope[0];x[11+c]=slope[1];x[13+c]=slope[2];x[15+c]=std::max(0.,v+std::min(age,.3)*slope[0]);
     x[22+c]=lag[1];x[24+c]=lag[2];
   }
   // Match the Python extractor: derived means are computed BEFORE float32 cast.
   // These means/differences are unused by Core; raw precision differences cannot affect B6.
   x[2]=.5f*(x[0]+x[1]);x[3]=x[0]-x[1];x[4]=std::abs(x[3]);x[17]=.5f*(x[15]+x[16]);
   x[18]=command;for(int j=0;j<3;++j)x[19+j]=ema_[j];return true;
 }
};
}
