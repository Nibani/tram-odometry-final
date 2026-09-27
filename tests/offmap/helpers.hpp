#define main legacy_acceptance_main
#include "../native_sparse_gnss.cpp"
#undef main
struct Pose2{Vec3 p,d;};
Fixture configured(bool enabled){Fixture f(true);auto c=settings(true);c.gnss_xy_residual_correction=true;c.offmap_departure=enabled?10.:0.;c.offmap_max_distance=100.;f.pipe=Pipeline(c,{f.route});return f;}
Pose2 fake(double s){double a=PI/6;return {{{150.+s*std::cos(a),s*std::sin(a),0}},{{std::cos(a),std::sin(a),0}}};}
Vec3 obs(const Pose2&p,int channel){double x=channel==0?-9.873:2.563;return observation_lla(add(p.p,add(mul(p.d,x),Vec3{{0,0,3}})));}
struct Stats{double maxerr=0.,maxjump=0.;int first=-1,last=-1,freecount=0;Output prev;bool have=false;void addout(int i,const Output&o,const Vec3&truth){maxerr=std::max(maxerr,norm(sub(o.position,truth)));if(have)maxjump=std::max(maxjump,norm(sub(o.position,prev.position)));prev=o;have=true;if(o.flags&256){if(first<0)first=i;last=i;++freecount;}}};
void conflicting(int corrupt,bool enabled){auto f=configured(enabled);f.through(399);Stats st;for(int i=400;i<=1000;++i){I t=T0+i*DT;double s=(i-400)*.25;Pose2 good={{{150+s,0,0}},{{1,0,0}}};if(i%2==0)for(int ch=0;ch<2;++ch){auto p=(corrupt==2||ch==corrupt)?fake(s):good;require(f.pipe.fix(ch,t,t,obs(p,ch)),"fix queued");}auto o=f.tick(i);st.addout(i,o,good.p);if(i==500||i==700||i==1000)std::cout<<"conflict corrupt="<<corrupt<<" enabled="<<enabled<<" tick="<<i<<" position="<<o.position[0]<<","<<o.position[1]<<" error="<<norm(sub(o.position,good.p))<<" flags="<<o.flags<<" accepted="<<o.gnss_accepted<<" rejected="<<o.gnss_rejected<<"\n";}
std::cout<<"SUMMARY conflict corrupt="<<corrupt<<" enabled="<<enabled<<" maxerr="<<st.maxerr<<" maxjump="<<st.maxjump<<" first="<<st.first<<" last="<<st.last<<" freecount="<<st.freecount<<"\n";}
// A branch goes30deg out for40m, then turns clockwise through60deg atR25,
// follows -30deg back to y0, then turns instantly parallel for recovery check.
Pose2 returning(double s){constexpr double radius=25.;double a=PI/6;if(s<=40)return fake(s);auto start=fake(40);double u=s-40,arc=radius*PI/3;if(u<arc){double h=a-u/radius;return {add(start.p,{{radius*(std::sin(a)-std::sin(h)),radius*(std::cos(h)-std::cos(a)),0}}),{{std::cos(h),std::sin(h),0}}};}Vec3 end=add(start.p,{{2*radius*std::sin(a),0,0}});u-=arc;double down=end[1]/std::sin(a);if(u<down)return {add(end,{{u*std::cos(a),-u*std::sin(a),0}}),{{std::cos(a),-std::sin(a),0}}};return {add(end,{{down*std::cos(a)+(u-down),-end[1],0}}),{{1,0,0}}};}
void genuine(bool enabled,int rate,bool reentry){auto f=configured(enabled);f.through(399);Stats st;for(int i=400;i<=1040;++i){I t=T0+i*DT;double s=(i-400)*.25;auto truth=reentry?returning(s):fake(s);if((i-400)%rate==0)for(int ch=0;ch<2;++ch)require(f.pipe.fix(ch,t,t,obs(truth,ch)),"genuine fix queued");auto o=f.tick(i);st.addout(i,o,truth.p);if(i==560||i==680||i==840||i==1040)std::cout<<"genuine enabled="<<enabled<<" rate="<<rate<<" reentry="<<reentry<<" tick="<<i<<" error="<<norm(sub(o.position,truth.p))<<" flags="<<o.flags<<" accepted="<<o.gnss_accepted<<" rejected="<<o.gnss_rejected<<"\n";}
std::cout<<"SUMMARY genuine enabled="<<enabled<<" rate="<<rate<<" reentry="<<reentry<<" maxerr="<<st.maxerr<<" maxjump="<<st.maxjump<<" first="<<st.first<<" last="<<st.last<<" freecount="<<st.freecount<<" finalerr="<<norm(sub(st.prev.position,(reentry?returning(160):fake(160)).p))<<"\n";}
void inherited_probe_main(){std::cout<<std::setprecision(17);for(int bad:{0,1,2})for(bool active:{false,true})conflicting(bad,active);for(bool active:{false,true}){genuine(active,2,false);genuine(active,2,true);genuine(active,20,false);}}



