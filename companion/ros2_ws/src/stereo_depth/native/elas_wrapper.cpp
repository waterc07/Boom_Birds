#include "elas.h"
#include <exception>
extern "C" int bb_elas(uint8_t *a, uint8_t *b, int w, int h, int n, float *da,
                       float *db) {
  if (!a || !b || !da || !db || w < 16 || h < 16 || n < 16 || n >= w)
    return -1;
  try {
    Elas::parameters p(Elas::ROBOTICS);
    p.disp_min = 0;
    p.disp_max = n - 1;
    p.postprocess_only_left = false;
    p.subsampling = false;
    Elas matcher(p);
    int32_t dims[3] = {w, h, w};
    matcher.process(a, b, da, db, dims);
    return 0;
  } catch (const std::exception &) {
    return -2;
  }
}
