#ifndef VISIONPOINT_H
#define VISIONPOINT_H
#include <vector>
#include <inttypes.h>

using namespace std;
struct VisionPoint {
  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t lbIndex;
  bool isValid;
  
  VisionPoint* parent;
  //uint16_t size; // size of the "tree"
  int position; // y position of the run
  int number; // number of run in that row/column

  vector<VisionPoint *> children;
};
#endif
