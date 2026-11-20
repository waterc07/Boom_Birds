/**
 ******************************************************************************
 * @file           : kernels.cpp
 * @brief          : 解包 RAW10 图像、计算 Census 视差，并将视差重建为深度或三维坐标。
 ******************************************************************************
 * @author         : water_c
 ******************************************************************************
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
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

// SGBM 的 Q4 格式视差；nearbyint 保持与 NumPy rint 一致的中点取偶舍入规则。
extern "C" int bb_reconstruct(const int16_t *left_disparity,
                              const int16_t *right_disparity, int width,
                              int height, int num_disparities,
                              const double *reprojection,
                              const float *depth_lut, float *output,
                              uint8_t *valid_mask, int output_xyz) {
  if (!left_disparity || !right_disparity || !reprojection || !depth_lut ||
      !output || !valid_mask || width < 1 || height < 1 || num_disparities < 16)
    return -1;
  // 将左图视差映射到右图，检查左右匹配后重建深度。
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      int index = y * width + x, disparity_q4 = left_disparity[index];
      int right_x = static_cast<int>(std::nearbyint(x - disparity_q4 / 16.0));
      bool valid = disparity_q4 > 0 &&
                   disparity_q4 < 16 * (num_disparities - 1) && right_x >= 0 &&
                   right_x < width;
      if (valid)
        valid = right_disparity[y * width + right_x] > -16 * num_disparities &&
                std::abs(disparity_q4 +
                         int(right_disparity[y * width + right_x])) <= 16;
      float depth = valid ? depth_lut[disparity_q4]
                          : std::numeric_limits<float>::quiet_NaN();
      valid = valid && std::isfinite(depth) && depth > 0;
      valid_mask[index] = valid;
      if (!valid)
        depth = std::numeric_limits<float>::quiet_NaN();
      if (!output_xyz) {
        output[index] = depth;
        continue;
      }
      double denominator =
          reprojection[14] * (disparity_q4 / 16.0) + reprojection[15];
      output[3 * index] = valid
                              ? float((reprojection[0] * x +
                                       reprojection[1] * y + reprojection[3]) /
                                      denominator)
                              : depth;
      output[3 * index + 1] =
          valid ? float((reprojection[4] * x + reprojection[5] * y +
                         reprojection[7]) /
                        denominator)
                : depth;
      output[3 * index + 2] = depth;
      if (valid && (!std::isfinite(output[3 * index]) ||
                    !std::isfinite(output[3 * index + 1]))) {
        valid_mask[index] = 0;
        output[3 * index] = output[3 * index + 1] = output[3 * index + 2] =
            std::numeric_limits<float>::quiet_NaN();
      }
    }
  return 0;
}

// 5×5 Census 变换，生成 24 位特征。半径为 2
// 的代价聚合为每个视差使用一张积分图。
extern "C" int bb_census(const uint8_t *left, const uint8_t *right, int width,
                         int height, int num_disparities,
                         const int16_t *min_disparity,
                         const int16_t *max_disparity, int16_t *output) {
  if (!left || !right || !output || width < 12 || height < 12 ||
      num_disparities < 2 || num_disparities >= width)
    return -1;
  const int pixel_count = width * height, integral_stride = width + 1;
  std::vector<uint32_t> left_census(pixel_count), right_census(pixel_count),
      integral_image((width + 1) * (height + 1));
  // 比较邻域与中心像素的亮度，生成左右图的 Census 特征。
  for (int y = 2; y < height - 2; ++y)
    for (int x = 2; x < width - 2; ++x) {
      uint32_t left_feature = 0, right_feature = 0;
      for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx) {
          if (dx == 0 && dy == 0)
            continue;
          left_feature =
              (left_feature << 1) |
              (left[(y + dy) * width + x + dx] < left[y * width + x]);
          right_feature =
              (right_feature << 1) |
              (right[(y + dy) * width + x + dx] < right[y * width + x]);
        }
      left_census[y * width + x] = left_feature;
      right_census[y * width + x] = right_feature;
    }
  std::vector<int> best_cost(pixel_count, 100000),
      second_cost(pixel_count, 100000), pixel_costs(pixel_count, 24);
  std::fill(output, output + pixel_count, int16_t(-16));
  // 遍历候选视差，用汉明距离计算代价，并聚合 5×5 邻域。
  for (int disparity = 0; disparity < num_disparities; ++disparity) {
    std::fill(pixel_costs.begin(), pixel_costs.end(), 24);
    for (int y = 2; y < height - 2; ++y) {
      int x = disparity + 2;
#ifdef __ARM_NEON
      for (; x + 4 <= width - 2; x += 4) {
        uint32x4_t v =
            veorq_u32(vld1q_u32(&left_census[y * width + x]),
                      vld1q_u32(&right_census[y * width + x - disparity]));
        uint16x8_t s = vpaddlq_u8(vcntq_u8(vreinterpretq_u8_u32(v)));
        uint32x4_t sums = vpaddlq_u16(s);
        uint32_t tmp[4];
        vst1q_u32(tmp, sums);
        for (int j = 0; j < 4; ++j)
          pixel_costs[y * width + x + j] = tmp[j];
      }
#endif
      for (; x < width - 2; ++x)
        pixel_costs[y * width + x] =
            __builtin_popcount(left_census[y * width + x] ^
                               right_census[y * width + x - disparity]);
    }
    std::fill(integral_image.begin(), integral_image.end(), 0);
    for (int y = 0; y < height; ++y) {
      uint32_t sum = 0;
      for (int x = 0; x < width; ++x) {
        sum += pixel_costs[y * width + x];
        integral_image[(y + 1) * integral_stride + x + 1] =
            sum + integral_image[y * integral_stride + x + 1];
      }
    }
    for (int y = 4; y < height - 4; ++y)
      for (int x = disparity + 4; x < width - 4; ++x) {
        int index = y * width + x;
        if (min_disparity && max_disparity &&
            (disparity < min_disparity[index] ||
             disparity > max_disparity[index]))
          continue;
        int aggregated_cost =
            integral_image[(y + 3) * integral_stride + x + 3] -
            integral_image[(y - 2) * integral_stride + x + 3] -
            integral_image[(y + 3) * integral_stride + x - 2] +
            integral_image[(y - 2) * integral_stride + x - 2];
        if (aggregated_cost < best_cost[index]) {
          second_cost[index] = best_cost[index];
          best_cost[index] = aggregated_cost;
          output[index] = 16 * disparity;
        } else if (aggregated_cost < second_cost[index])
          second_cost[index] = aggregated_cost;
      }
  }
  // 剔除代价过高或最佳结果与次佳结果过于接近的匹配。
  for (int index = 0; index < pixel_count; ++index)
    if (best_cost[index] > 300 || second_cost[index] >= 100000 ||
        second_cost[index] - best_cost[index] <=
            std::max(4, best_cost[index] / 10))
      output[index] = -16;
  return 0;
}

// RAW10 每 5 字节存储 4 个像素；查找表将 10 位值映射到 8 位。
extern "C" int bb_unpack_raw10(const uint8_t *input, std::size_t bytes,
                               int width, int height, int stride,
                               const uint8_t *lookup_table, uint8_t *output) {
  if (!input || !lookup_table || !output || width < 4 || width > 16384 ||
      height < 2 || height > 16384 || width % 4 || stride < width * 5 / 4 ||
      stride > width * 5 / 4 + 256 ||
      bytes !=
          static_cast<std::size_t>(stride) * static_cast<std::size_t>(height))
    return -1;
  for (int y = 0; y < height; ++y) {
    const uint8_t *row = input + static_cast<std::size_t>(y) * stride;
    uint8_t *output_row = output + static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width; x += 4) {
      const uint8_t *group = row + x / 4 * 5;
      for (int i = 0; i < 4; ++i)
        output_row[x + i] =
            lookup_table[(static_cast<unsigned>(group[i]) << 2) |
                         ((group[4] >> (2 * i)) & 3)];
    }
  }
  return 0;
}
