#include "reserve_odometry/core.hpp"
#include "reserve_odometry/features.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char**argv){
 if(argc!=3){std::cerr<<"Usage: stream_cli events.bin predictions.bin\n";return 2;}
 std::ifstream in(argv[1],std::ios::binary);std::ofstream out(argv[2],std::ios::binary);if(!in||!out)return 3;
 reserve_odometry::Core core;reserve_odometry::FeatureBuilder features;std::uint8_t kind;std::int64_t stamp;double val;std::size_t count=0;
 while(in.read(reinterpret_cast<char*>(&kind),1)) {
   if(!in.read(reinterpret_cast<char*>(&stamp),8)||!in.read(reinterpret_cast<char*>(&val),8))return 4;
   if(kind<2)features.wheel(kind,stamp,val);
   else if(kind==2){std::array<float,26>x;double t;if(features.build(stamp,int(val),x,t)){double v=core.step(t,x).velocity;out.write(reinterpret_cast<char*>(&v),8);++count;}}
   else return 5;
 }
 std::cerr<<"predictions="<<count<<"\n";return out?0:6;
}
