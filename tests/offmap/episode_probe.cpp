#define main legacy_acceptance_main
#include "../native_sparse_gnss.cpp"
#undef main
Fixture offmap(double departure,double limit=100.){Fixture f(true);auto c=settings(true);c.gnss_xy_residual_correction=true;c.offmap_departure=departure;c.offmap_max_distance=limit;f.pipe=Pipeline(c,{f.route});return f;}
Vec3 branch(const Fixture&f,I fork,I t,double x){double s=5.*double(t-fork)*1e-9,a=30.*PI/180.;auto base=f.route.pose(f.start+5.*double(fork-T0)*1e-9).position;Vec3 d{{std::cos(a),std::sin(a),0.}};return add(base,add(mul(d,s+x),Vec3{{0,0,x==0.?0.:3.}}));}
Output drive_branch(Fixture&f,int first,int last,bool fixes=true,int from=-1){I fork=T0+first*DT;if(from<0)from=first;Output out;for(int i=from;i<=last;++i){I t=T0+i*DT;if(fixes&&i%2==0)require(f.pipe.fix(1,t,t,observation_lla(branch(f,fork,t,2.563))),"branch fix");out=f.tick(i);}return out;}
int main(){
 test("new episode after both streams absent clears stale conflict",[]{auto f=offmap(10.);f.through(79);I t=T0+80*DT;for(int ch=0;ch<2;++ch)require(f.pipe.fix(ch,t,t,f.fix_lla(ch,t,0.,ch==0?12.:0.)),"conflict");require(f.tick(80).flags&1024,"missing conflict");for(int i=81;i<400;++i)f.tick(i);auto out=drive_branch(f,400,600);require(!(out.flags&1024),"stale conflict survived new episode");require(out.flags&256,"new clean branch blocked");});
 test("continuous bad receiver retains conflict",[]{auto f=offmap(10.);f.through(79);I t=T0+80*DT;for(int ch=0;ch<2;++ch)require(f.pipe.fix(ch,t,t,f.fix_lla(ch,t,0.,ch==0?12.:0.)),"conflict");require(f.tick(80).flags&1024,"missing conflict");for(int i=81;i<=400;++i){t=T0+i*DT;if(i%2==0)require(f.pipe.fix(0,t,t,f.fix_lla(0,t,0.,12.)),"bad stream");auto o=f.tick(i);require(o.flags&1024,"continuous stream aged out conflict");require(!(o.flags&256),"continuous bad receiver entered free mode");}});
 std::cout<<"EPISODE_RESULT failures="<<failures<<'\n';return failures?1:0;
}

