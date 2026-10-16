#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

namespace astra {
// Frozen safety-mask erosion is prepared once, before mission search. Keep the
// same square margin as the per-goal version, closing unknown and map edges.
inline std::vector<int8_t> envelopeSearchCells(const std::vector<int8_t> &cells,
    uint32_t width, uint32_t height, int padding) {
  const size_t n=static_cast<size_t>(width)*height;
  if (!width || !height || n!=cells.size() || n>1440000 || padding<0) return {};
  auto search=cells;
  for (int y=0;y<int(height);++y)
    for (int x=0;x<int(width);++x) {
      if (cells[y*width+x]!=0) continue;
      bool blocked=false;
      for (int dy=-padding;dy<=padding && !blocked;++dy)
        for (int dx=-padding;dx<=padding;++dx)
          if (x+dx<0 || y+dy<0 || x+dx>=int(width) || y+dy>=int(height) ||
              cells[(y+dy)*width+x+dx]!=0) {blocked=true;break;}
      if (blocked) search[y*width+x]=100;
    }
  return search;
}

// Bounded global topology search on the independent safety mask. Four-connected
// edges stay inside adjacent certified squares; unknown/blocked cells are closed.
inline std::vector<uint32_t> envelopePath(const std::vector<int8_t> &cells,
    uint32_t width, uint32_t height, uint32_t start, uint32_t goal,
    std::chrono::steady_clock::time_point deadline) {
  const size_t n = static_cast<size_t>(width)*height;
  if (!width || !height || n != cells.size() || n > 1440000 ||
      start >= n || goal >= n || cells[start] != 0 || cells[goal] != 0) return {};
  std::vector<int32_t> parent(n, -2);
  std::vector<uint32_t> queue;
  queue.reserve(n);
  parent[start] = -1;
  queue.push_back(start);
  for (size_t head=0; head<queue.size(); ++head) {
    if (head % 512 == 0 && std::chrono::steady_clock::now() >= deadline) return {};
    const auto u = queue[head];
    if (u == goal) {
      std::vector<uint32_t> path;
      for (int32_t v=goal; v>=0; v=parent[v]) path.push_back(v);
      std::reverse(path.begin(),path.end());
      return path;
    }
    const uint32_t x=u%width, y=u/width;
    const auto visit = [&](uint32_t v) {
      if (parent[v] == -2 && cells[v] == 0) { parent[v]=u; queue.push_back(v); }
    };
    if (x) visit(u-1);
    if (x+1<width) visit(u+1);
    if (y) visit(u-width);
    if (y+1<height) visit(u+width);
  }
  return {};
}
} // namespace astra
