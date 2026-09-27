#include "reserve_odometry/core.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
struct Record{double t;std::array<float,26>x;};
int main(int argc,char**argv){
 if(argc!=3){std::cerr<<"Usage: core_cli features.bin velocity.bin\n";return 2;}
 std::ifstream in(argv[1],std::ios::binary);if(!in){std::cerr<<"Cannot open input\n";return 3;}
 std::vector<Record> records;Record r;
 while(in.read(reinterpret_cast<char*>(&r.t),8)){
  if(!in.read(reinterpret_cast<char*>(r.x.data()),104)){std::cerr<<"Truncated input\n";return 4;}records.push_back(r);
 }
 if(records.empty()){std::cerr<<"Empty input\n";return 5;}
 std::ofstream out(argv[2],std::ios::binary);if(!out)return 6;
 reserve_odometry::Core core;std::vector<double> ns;ns.reserve(records.size());double checksum=0.;
 for(const auto& x:records){auto a=std::chrono::steady_clock::now();auto y=core.step(x.t,x.x);auto b=std::chrono::steady_clock::now();
  ns.push_back(std::chrono::duration<double,std::nano>(b-a).count());out.write(reinterpret_cast<char*>(&y.velocity),8);checksum+=y.velocity;}
 std::sort(ns.begin(),ns.end());
 std::cerr<<std::setprecision(12)<<"{\"n\":"<<records.size()<<",\"step_median_us\":"<<ns[ns.size()/2]/1e3<<",\"step_p99_us\":"<<ns[std::min(ns.size()-1,ns.size()*99/100)]/1e3<<",\"step_max_us\":"<<ns.back()/1e3<<",\"checksum\":"<<checksum<<"}\n";
 return !out?7:0;
}
