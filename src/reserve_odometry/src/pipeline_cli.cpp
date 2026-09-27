#include "reserve_odometry/pipeline.hpp"
#include "reserve_odometry/output_frame.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace reserve_odometry;
int main(int argc,char**argv){
 try{
  Settings cfg;std::vector<Route>routes;
  bool closed=false,body_velocity=false,transform=false;OutputFrame frame;std::vector<std::string>paths;
  for(int i=1;i<argc;++i){std::string a=argv[i];if(a=="--base-speed")cfg.compensate_velocity_lever_arm=false;else if(a=="--antenna-speed")cfg.compensate_velocity_lever_arm=true;else if(a=="--loop")closed=true;else if(a=="--strict-init")cfg.allow_degraded_initialization=false;else if(a=="--align-initial")cfg.align_initial_position=true;else if(a=="--sparse-gnss")cfg.sparse_gnss_correction=true;else if(a=="--xy-residual")cfg.gnss_xy_residual_correction=true;else if(a=="--body-velocity")body_velocity=true;else if(a=="--map-transform"&&i+4<argc){transform=true;frame.yaw=std::stod(argv[++i]);for(double&v:frame.translation)v=std::stod(argv[++i]);}else if(a=="--speed-scale"&&i+1<argc)cfg.speed_scale=std::stod(argv[++i]);else if(a=="--offmap-departure"&&i+1<argc)cfg.offmap_departure=std::stod(argv[++i]);else if(a=="--offmap-angle"&&i+1<argc)cfg.offmap_angle=std::stod(argv[++i]);else if(a=="--offmap-min-span"&&i+1<argc)cfg.offmap_min_span=std::stod(argv[++i]);else if(a=="--offmap-horizon"&&i+1<argc)cfg.offmap_horizon=std::stod(argv[++i]);else if(a=="--offmap-max-distance"&&i+1<argc)cfg.offmap_max_distance=std::stod(argv[++i]);else if(a=="--adhesion-accel"&&i+1<argc)cfg.adhesion_accel_limit=std::stod(argv[++i]);else if(a=="--adhesion-decel"&&i+1<argc)cfg.adhesion_decel_limit=std::stod(argv[++i]);else if(a=="--slip-release-rate"&&i+1<argc)cfg.slip_release_rate=std::stod(argv[++i]);else if(a=="--slip-release-hold"&&i+1<argc)cfg.slip_release_hold=std::stod(argv[++i]);else if(a=="--slip-max-latch"&&i+1<argc)cfg.slip_max_latch=std::stod(argv[++i]);else paths.push_back(a);}
  if(!std::isfinite(frame.yaw)||!finite(frame.translation))throw std::runtime_error("Invalid map transform");
  for(const auto&path:paths)routes.push_back(Route::load(path,7.55,closed));
  Pipeline engine(cfg,std::move(routes));std::string line;std::cout<<std::setprecision(17);
  while(std::getline(std::cin,line)){
   std::istringstream row(line);char kind;std::int64_t stamp,arrival;
   if(!(row>>kind>>stamp>>arrival))continue;
   if(kind=='X'){engine.reset();continue;}
   if(kind=='F'||kind=='R'){double value;if(row>>value)engine.wheel(kind=='R',stamp,value,arrival);}
   else if(kind=='M'||kind=='G'){Vec3 lla;int status=0;if(row>>lla[0]>>lla[1]>>lla[2]){row>>status;engine.fix(kind=='G',stamp,arrival,lla,status);}}
   else if(kind=='C'||kind=='T'){
    if(kind=='C'){int value;if(!(row>>value)||!engine.command(stamp,value,arrival))continue;}
    Output out;if(!engine.step(stamp,arrival,out))continue;
    auto master=antenna(out.body,-9.873,3.);
    if(body_velocity&&out.initialized)out.velocity=out.body_velocity[0];
    if(transform&&out.initialized){out.position=frame.position(out.position);master=frame.position(master);}
    std::cout<<out.stamp<<' '<<out.velocity<<' '<<out.bogie_velocity<<' '<<out.distance<<' '<<out.route_s<<' '<<out.position[0]<<' '<<out.position[1]<<' '<<out.position[2]<<' '<<out.flags<<' '<<out.route<<' '<<out.init_rms<<' '<<master[0]<<' '<<master[1]<<' '<<master[2];
    for(double v:out.body_velocity)std::cout<<' '<<v;for(double v:out.body_angular_velocity)std::cout<<' '<<v;std::cout<<'\n';
   }
  }
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}return 0;
}
