#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

extern "C" int bb_has_neon() {
#ifdef __ARM_NEON
  return 1;
#else
  return 0;
#endif
}

// SGBM Q4 disparity; nearbyint preserves NumPy rint's ties-to-even convention.
extern "C" int bb_reconstruct(const int16_t *a, const int16_t *b, int w, int h,
                              int n, const double *q, const float *lut,
                              float *out, uint8_t *mask, int xyz) {
  if (!a || !b || !q || !lut || !out || !mask || w < 1 || h < 1 || n < 16)
    return -1;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      int i = y * w + x, d = a[i];
      int xb = static_cast<int>(std::nearbyint(x - d / 16.0));
      bool valid = d > 0 && d < 16 * (n - 1) && xb >= 0 && xb < w;
      if (valid)
        valid =
            b[y * w + xb] > -16 * n && std::abs(d + int(b[y * w + xb])) <= 16;
      float z = valid ? lut[d] : std::numeric_limits<float>::quiet_NaN();
      valid = valid && std::isfinite(z) && z > 0;
      mask[i] = valid;
      if (!valid)
        z = std::numeric_limits<float>::quiet_NaN();
      if (!xyz) {
        out[i] = z;
        continue;
      }
      double den = q[14] * (d / 16.0) + q[15];
      out[3 * i] = valid ? float((q[0] * x + q[1] * y + q[3]) / den) : z;
      out[3 * i + 1] = valid ? float((q[4] * x + q[5] * y + q[7]) / den) : z;
      out[3 * i + 2] = z;
      if (valid &&
          (!std::isfinite(out[3 * i]) || !std::isfinite(out[3 * i + 1]))) {
        mask[i] = 0;
        out[3 * i] = out[3 * i + 1] = out[3 * i + 2] =
            std::numeric_limits<float>::quiet_NaN();
      }
    }
  return 0;
}

// 5x5 Census, 24 bits. Radius-2 aggregation uses one integral image per
// disparity.
extern "C" int bb_census(const uint8_t *a, const uint8_t *b, int w, int h,
                         int n, const int16_t *lo, const int16_t *hi,
                         int16_t *out) {
  if (!a || !b || !out || w < 12 || h < 12 || n < 2 || n >= w)
    return -1;
  const int count = w * h, stride = w + 1;
  std::vector<uint32_t> ca(count), cb(count), integral((w + 1) * (h + 1));
  for (int y = 2; y < h - 2; ++y)
    for (int x = 2; x < w - 2; ++x) {
      uint32_t va = 0, vb = 0;
      for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx) {
          if (dx == 0 && dy == 0)
            continue;
          va = (va << 1) | (a[(y + dy) * w + x + dx] < a[y * w + x]);
          vb = (vb << 1) | (b[(y + dy) * w + x + dx] < b[y * w + x]);
        }
      ca[y * w + x] = va;
      cb[y * w + x] = vb;
    }
  std::vector<int> best(count, 100000), second(count, 100000), costs(count, 24);
  std::fill(out, out + count, int16_t(-16));
  for (int d = 0; d < n; ++d) {
    std::fill(costs.begin(), costs.end(), 24);
    for (int y = 2; y < h - 2; ++y) {
      int x = d + 2;
#ifdef __ARM_NEON
      for (; x + 4 <= w - 2; x += 4) {
        uint32x4_t v =
            veorq_u32(vld1q_u32(&ca[y * w + x]), vld1q_u32(&cb[y * w + x - d]));
        uint16x8_t s = vpaddlq_u8(vcntq_u8(vreinterpretq_u8_u32(v)));
        uint32x4_t sums = vpaddlq_u16(s);
        uint32_t tmp[4];
        vst1q_u32(tmp, sums);
        for (int j = 0; j < 4; ++j)
          costs[y * w + x + j] = tmp[j];
      }
#endif
      for (; x < w - 2; ++x)
        costs[y * w + x] =
            __builtin_popcount(ca[y * w + x] ^ cb[y * w + x - d]);
    }
    std::fill(integral.begin(), integral.end(), 0);
    for (int y = 0; y < h; ++y) {
      uint32_t sum = 0;
      for (int x = 0; x < w; ++x) {
        sum += costs[y * w + x];
        integral[(y + 1) * stride + x + 1] = sum + integral[y * stride + x + 1];
      }
    }
    for (int y = 4; y < h - 4; ++y)
      for (int x = d + 4; x < w - 4; ++x) {
        int i = y * w + x;
        if (lo && hi && (d < lo[i] || d > hi[i]))
          continue;
        int c = integral[(y + 3) * stride + x + 3] -
                integral[(y - 2) * stride + x + 3] -
                integral[(y + 3) * stride + x - 2] +
                integral[(y - 2) * stride + x - 2];
        if (c < best[i]) {
          second[i] = best[i];
          best[i] = c;
          out[i] = 16 * d;
        } else if (c < second[i])
          second[i] = c;
      }
  }
  for (int i = 0; i < count; ++i)
    if (best[i] > 300 || second[i] >= 100000 ||
        second[i] - best[i] <= std::max(4, best[i] / 10))
      out[i] = -16;
  return 0;
}
