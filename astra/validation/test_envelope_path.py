"""Global safety topology must find detours beyond the local search window."""
from pathlib import Path
import subprocess


def test_global_mask_detour_and_bounded_failure(tmp_path):
    package=Path(__file__).resolve().parents[1]/'packages/ego_planner_jazzy/plan_env'
    source=tmp_path/'path.cpp'
    source.write_text(r'''
#include <plan_env/envelope_path.h>
#include <cassert>
#include <cstdlib>
int main() {
  constexpr uint32_t w=80,h=80;
  std::vector<int8_t> cells(w*h,0);
  for(uint32_t y=0;y<65;++y)
    for(uint32_t x=30;x<=50;++x) cells[y*w+x]=100;
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  const auto p=astra::envelopePath(cells,w,h,10*w+20,10*w+60,deadline);
  assert(!p.empty() && p.front()==10*w+20 && p.back()==10*w+60);
  bool beyond=false;
  for(size_t i=0;i<p.size();++i) {
    assert(cells[p[i]]==0);
    beyond |= p[i]/w>=65;
    if(i) {
      const int dx=std::abs(int(p[i]%w)-int(p[i-1]%w));
      const int dy=std::abs(int(p[i]/w)-int(p[i-1]/w));
      assert(dx+dy==1);
    }
  }
  assert(beyond);
  const auto padded=astra::envelopeSearchCells(cells,w,h,2);
  assert(padded.size()==cells.size());
  assert(padded[10*w+28]==100 && padded[10*w+27]==0);
  assert(padded[1*w+20]==100 && padded[2*w+20]==0);
  const auto safe_path=astra::envelopePath(padded,w,h,10*w+20,10*w+60,deadline);
  assert(!safe_path.empty());
  for(auto v:safe_path) {
    const int x=v%w,y=v/w;
    for(int dy=-2;dy<=2;++dy) for(int dx=-2;dx<=2;++dx)
      assert(x+dx>=0 && y+dy>=0 && x+dx<int(w) && y+dy<int(h) && cells[(y+dy)*w+x+dx]==0);
  }
  auto unknown=cells;unknown[10*w+20]=-1;
  assert(astra::envelopeSearchCells(unknown,w,h,2)[10*w+21]==100);
  assert(astra::envelopeSearchCells(cells,w,h,-1).empty());
  assert(astra::envelopePath(cells,w,h,10*w+20,10*w+60,
      std::chrono::steady_clock::now()-std::chrono::seconds(1)).empty());
  for(uint32_t y=0;y<h;++y) cells[y*w+40]=100;
  assert(astra::envelopePath(cells,w,h,10*w+20,10*w+60,deadline).empty());
  assert(astra::envelopePath(cells,w,h,w*h+1,0,deadline).empty());
}
''')
    binary=tmp_path/'path'
    subprocess.run(['g++','-std=c++17','-O2','-I',str(package/'include'),str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
