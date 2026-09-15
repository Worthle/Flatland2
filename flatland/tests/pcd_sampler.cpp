#include <flatland_plugins/pcd_sampler.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace flatland_plugins;
// Regression check: parallel reduction must preserve serial projection,
// including equal-distance ties, and a changed pose must invalidate caching.
int main(int argc,char**argv) {
  if (argc != 2) throw std::runtime_error("usage: pcd_sampler_check warehouse.pcd");
  std::ifstream f(argv[1],std::ios::binary);std::string line;size_t count=0;
  while(std::getline(f,line)) {
    if(line.rfind("POINTS ",0)==0)count=std::stoul(line.substr(7));
    if(line=="DATA binary")break;
  }
  if (!f || count == 0) throw std::runtime_error("invalid PCD fixture");
  std::vector<float> map(count*3);
  f.read(reinterpret_cast<char*>(map.data()),map.size()*sizeof(float));
  if (!f) throw std::runtime_error("truncated PCD fixture");
  std::vector<double> e;for(int d=-15;d<=30;d+=3)e.push_back(d*M_PI/180.);
  PcdSampler serial(map,e,1.5*M_PI/180.,360,.5,50.,1);
  PcdSampler parallel(map,e,1.5*M_PI/180.,360,.5,50.,8);
  double ts=0,tp=0;
  for(int i=0;i<8;++i) {
    auto t=std::chrono::steady_clock::now();
    auto expected=serial.Sample(i*.02,-2,.6,i*.04);
    auto t1=std::chrono::steady_clock::now();
    auto actual=parallel.Sample(i*.02,-2,.6,i*.04);
    auto t2=std::chrono::steady_clock::now();
    const auto cached=parallel.Sample(i*.02,-2,.6,i*.04);
    if (cached.range_squared!=actual.range_squared || cached.points!=actual.points)
      throw std::runtime_error("cache changed the projection");
    ts+=std::chrono::duration<double>(t1-t).count();tp+=std::chrono::duration<double>(t2-t1).count();
    for(size_t j=0;j<expected.range_squared.size();++j) {
      if(expected.range_squared[j]!=actual.range_squared[j] || (expected.range_squared[j]!=std::numeric_limits<float>::max() && expected.points[j]!=actual.points[j]))throw std::runtime_error("projection differs");
    }
  }
  // Put equal-range points in separate worker chunks. The first point must
  // win even when later chunks finish first. Vary Z inside one elevation bin
  // to distinguish the two returns while preserving their squared range.
  std::vector<float> ties(20000*3, 100.f);
  ties[0]=2;ties[1]=0;ties[2]=.01f;
  ties[15000*3]=2;ties[15000*3+1]=0;ties[15000*3+2]=-.01f;
  PcdSampler tie_sampler(ties,{0},.1,8,.5,10.,4);
  const auto tie_result=tie_sampler.Sample(0,0,0,0);
  if(tie_result.points[4][2]!=.01f)throw std::runtime_error("equal-range ordering changed");
  std::cout << "PASS: exact serial/parallel returns, cached and changed poses, equal-distance ties; serial_ms " << ts/8*1000 << " parallel_ms " << tp/8*1000 << " exact_match true\n";
}
